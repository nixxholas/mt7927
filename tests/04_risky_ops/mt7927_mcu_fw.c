/*
 * MT7927 Driver with Proper MCU Command Protocol
 *
 * This implements the correct firmware loading sequence used by mt76:
 * 1. Initialize MCU queues (TX ring for commands, RX ring for responses)
 * 2. Send MCU commands for firmware download
 * 3. Use FW_SCATTER command to transfer firmware data
 *
 * Based on reverse-engineering of mt7925/mt76 drivers
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#define MT7927_VENDOR_ID    0x14c3
#define MT7927_DEVICE_ID    0x7927

/*
 * WFDMA registers - MT7925 uses remapped addresses
 * Physical base 0x54000000 -> remapped to 0x2000 in BAR0
 * So WFDMA0_GLO_CFG (offset 0x208) -> BAR0 + 0x2208
 */
#define MT_WFDMA0_BASE_REMAP        0x2000  /* Remapped WFDMA base in BAR0 */

#define MT_WFDMA0_GLO_CFG           (MT_WFDMA0_BASE_REMAP + 0x0208)
#define MT_WFDMA0_GLO_CFG_TX_DMA_EN     BIT(0)
#define MT_WFDMA0_GLO_CFG_RX_DMA_EN     BIT(2)
#define MT_WFDMA0_GLO_CFG_TX_WB_DDONE   BIT(6)
#define MT_WFDMA0_GLO_CFG_FIFO_LITTLE_ENDIAN BIT(12)
#define MT_WFDMA0_GLO_CFG_OMIT_RX_INFO  BIT(27)
#define MT_WFDMA0_GLO_CFG_OMIT_TX_INFO  BIT(28)

#define MT_WFDMA0_RST_DTX_PTR       (MT_WFDMA0_BASE_REMAP + 0x020c)
#define MT_WFDMA0_RST_DRX_PTR       (MT_WFDMA0_BASE_REMAP + 0x0280)

/* BUSY enable - needed for DMA to work (from mt7921) */
#define MT_WFDMA0_BUSY_ENA          (MT_WFDMA0_BASE_REMAP + 0x013c)
#define MT_WFDMA0_BUSY_ENA_TX_FIFO0 BIT(0)
#define MT_WFDMA0_BUSY_ENA_TX_FIFO1 BIT(1)
#define MT_WFDMA0_BUSY_ENA_RX_FIFO  BIT(2)

/* TX Ring base - MT7925 uses rings 15 (MCU) and 16 (FWDL) */
#define MT_TX_RING_BASE             (MT_WFDMA0_BASE_REMAP + 0x0300)
#define MT_TX_RING_SIZE             0x10  /* 16 bytes per ring config */

/* Ring indices - using lower indices since 15/16 registers aren't accessible
 * MT7925 officially uses 15/16 but those registers read back as 0 on MT7927.
 * Try rings 1/2 since config writes work there.
 */
#define MT_FWDL_RING_IDX            2     /* Firmware download ring */
#define MT_MCU_RING_IDX             1     /* MCU command ring */

/* Ring registers - matches mt76_queue_regs structure */
#define MT_RING_BASE(idx)           (MT_TX_RING_BASE + (idx) * MT_TX_RING_SIZE)
#define MT_RING_DESC_BASE(idx)      (MT_RING_BASE(idx) + 0x00)  /* desc_base */
#define MT_RING_CNT(idx)            (MT_RING_BASE(idx) + 0x04)  /* ring_size */
#define MT_RING_CIDX(idx)           (MT_RING_BASE(idx) + 0x08)  /* cpu_idx */
#define MT_RING_DIDX(idx)           (MT_RING_BASE(idx) + 0x0c)  /* dma_idx */

/* RX Ring base */
#define MT_RX_RING_BASE             0x0500
#define MT_RXQ_MCU                  0     /* MCU response ring */

/* Host interrupt */
#define MT_WFDMA0_HOST_INT_STA      0x0200
#define MT_WFDMA0_HOST_INT_ENA      0x0204

/* MCU registers */
#define MT_TOP_MISC2                0x1128
#define MT_TOP_MISC2_FW_N9_RDY      BIT(3)

#define MT_CONN_ON_MISC             0x7c0600f0
#define MT_HIF_REMAP_L1             0x7c500100

/* MCU command IDs - from mt76_connac_mcu.h */
#define MCU_CMD_TARGET_ADDRESS_LEN_REQ  0x01
#define MCU_CMD_FW_START_REQ            0x02
#define MCU_CMD_PATCH_START_REQ         0x05
#define MCU_CMD_PATCH_SEM_CONTROL       0x10
#define MCU_CMD_PATCH_FINISH_REQ        0x11
#define MCU_CMD_FW_SCATTER              0xee

/* Patch semaphore operations */
#define PATCH_SEM_GET                   1
#define PATCH_SEM_RELEASE               0
#define PATCH_NOT_DL_SEM_SUCCESS        0
#define PATCH_IS_DL                     1

/* Download modes */
#define DL_MODE_NEED_RSP                BIT(31)

/* TXD format */
#define MT_TXD0_TX_BYTES                GENMASK(15, 0)
#define MT_TXD0_PKT_FMT                 GENMASK(24, 23)
#define MT_TXD0_Q_IDX                   GENMASK(31, 25)
#define MT_TX_TYPE_CMD                  1
#define MT_TX_MCU_PORT_RX_Q0            0

/* DMA descriptor */
struct mt76_desc {
    __le32 buf0;
    __le32 buf1;
    __le32 ctrl;
    __le32 info;
} __packed __aligned(4);

#define MT_DMA_CTL_SD_LEN0      GENMASK(13, 0)
#define MT_DMA_CTL_LAST_SEC0    BIT(14)
#define MT_DMA_CTL_BURST        BIT(15)
#define MT_DMA_CTL_SD_LEN1      GENMASK(29, 16)
#define MT_DMA_CTL_LAST_SEC1    BIT(30)
#define MT_DMA_CTL_DMA_DONE     BIT(31)

/* MCU TXD header */
struct mt76_mcu_txd {
    __le32 txd[2];
    __le16 len;
    __le16 pq_id;
    u8 cid;
    u8 pkt_type;
    u8 set_query;
    u8 seq;
    u8 uc_d2b0_rev;
    u8 ext_cid;
    u8 s2d_index;
    u8 ext_cid_ack;
    __le32 reserved[5];
} __packed __aligned(4);

#define MCU_PQ_ID(port, q)      (((port) << 15) | ((q) << 10))
#define MCU_PKT_ID              0xa0

/* Ring size */
#define MT_FWDL_RING_SIZE       128
#define MT_MCU_RING_SIZE        32

/* Firmware files */
static const char *fw_patch = "mediatek/mt7925/WIFI_MT7925_PATCH_MCU_1_1_hdr.bin";
static const char *fw_ram = "mediatek/mt7925/WIFI_RAM_CODE_MT7925_1_1.bin";

struct mt7927_ring {
    struct mt76_desc *desc;
    dma_addr_t desc_dma;
    void **buf;
    dma_addr_t *buf_dma;
    int head;
    int tail;
    int size;
};

struct mt7927_dev {
    struct pci_dev *pdev;
    void __iomem *regs;         /* BAR0 - main registers */
    void __iomem *regs2;        /* BAR2 - control registers */

    struct mt7927_ring fwdl_ring;
    struct mt7927_ring mcu_ring;

    u8 mcu_seq;
    spinlock_t mcu_lock;
};

/*
 * Register access:
 * - BAR0 (dev->regs): Main register space including remapped WFDMA
 * - BAR2 (dev->regs2): Control registers (FW_STATUS, LPCTL, etc.)
 */

/* BAR0 access - WFDMA and main registers */
static u32 mt7927_rr(struct mt7927_dev *dev, u32 offset)
{
    return ioread32(dev->regs + offset);
}

static void mt7927_wr(struct mt7927_dev *dev, u32 offset, u32 val)
{
    iowrite32(val, dev->regs + offset);
}

/* BAR2 access - control registers */
static u32 mt7927_ctrl_rr(struct mt7927_dev *dev, u32 offset)
{
    return ioread32(dev->regs2 + offset);
}

static void mt7927_ctrl_wr(struct mt7927_dev *dev, u32 offset, u32 val)
{
    iowrite32(val, dev->regs2 + offset);
}

/* BAR0 memory access */
static u32 mt7927_mem_rr(struct mt7927_dev *dev, u32 offset)
{
    return ioread32(dev->regs + offset);
}

static void mt7927_rmw(struct mt7927_dev *dev, u32 offset, u32 mask, u32 val)
{
    u32 cur = mt7927_rr(dev, offset);
    mt7927_wr(dev, offset, (cur & ~mask) | val);
}

static void mt7927_set(struct mt7927_dev *dev, u32 offset, u32 val)
{
    mt7927_rmw(dev, offset, 0, val);
}

/* Initialize a TX ring */
static int mt7927_ring_alloc(struct mt7927_dev *dev, struct mt7927_ring *ring,
                             int size, int ring_idx)
{
    int i;

    ring->size = size;
    ring->head = 0;
    ring->tail = 0;

    /* Allocate descriptor ring */
    ring->desc = dma_alloc_coherent(&dev->pdev->dev,
                                    size * sizeof(struct mt76_desc),
                                    &ring->desc_dma, GFP_KERNEL);
    if (!ring->desc)
        return -ENOMEM;

    /* Initialize all descriptors with DMA_DONE set (mt76 requirement).
     * DMA_DONE=1 means "CPU owns this descriptor", DMA_DONE=0 means "pending".
     * When we queue a descriptor, we clear DMA_DONE. DMA sets it when done.
     */
    for (i = 0; i < size; i++)
        ring->desc[i].ctrl = cpu_to_le32(MT_DMA_CTL_DMA_DONE);

    /* Allocate buffer pointers */
    ring->buf = kcalloc(size, sizeof(void *), GFP_KERNEL);
    ring->buf_dma = kcalloc(size, sizeof(dma_addr_t), GFP_KERNEL);
    if (!ring->buf || !ring->buf_dma)
        return -ENOMEM;

    /* Pre-allocate buffers for each descriptor */
    for (i = 0; i < size; i++) {
        ring->buf[i] = dma_alloc_coherent(&dev->pdev->dev, 4096,
                                          &ring->buf_dma[i], GFP_KERNEL);
        if (!ring->buf[i])
            return -ENOMEM;
    }

    /* Configure ring in hardware - mt76 uses 32-bit desc_base */
    dev_info(&dev->pdev->dev, "Ring %d config: desc_base=0x%x cnt=0x%x cidx=0x%x\n",
             ring_idx, MT_RING_DESC_BASE(ring_idx), MT_RING_CNT(ring_idx),
             MT_RING_CIDX(ring_idx));

    mt7927_wr(dev, MT_RING_DESC_BASE(ring_idx), lower_32_bits(ring->desc_dma));
    mt7927_wr(dev, MT_RING_CNT(ring_idx), size);
    mt7927_wr(dev, MT_RING_CIDX(ring_idx), 0);
    mt7927_wr(dev, MT_RING_DIDX(ring_idx), 0);
    wmb();

    /* Verify ring config was written */
    {
        u32 base = mt7927_rr(dev, MT_RING_DESC_BASE(ring_idx));
        u32 cnt = mt7927_rr(dev, MT_RING_CNT(ring_idx));
        u32 cidx = mt7927_rr(dev, MT_RING_CIDX(ring_idx));
        u32 didx = mt7927_rr(dev, MT_RING_DIDX(ring_idx));
        dev_info(&dev->pdev->dev, "Ring %d verify: base=0x%08x cnt=%u cidx=%u didx=%u\n",
                 ring_idx, base, cnt, cidx, didx);
    }

    return 0;
}

static void mt7927_ring_free(struct mt7927_dev *dev, struct mt7927_ring *ring)
{
    int i;

    if (ring->buf) {
        for (i = 0; i < ring->size; i++) {
            if (ring->buf[i])
                dma_free_coherent(&dev->pdev->dev, 4096,
                                  ring->buf[i], ring->buf_dma[i]);
        }
        kfree(ring->buf);
        kfree(ring->buf_dma);
    }

    if (ring->desc)
        dma_free_coherent(&dev->pdev->dev,
                          ring->size * sizeof(struct mt76_desc),
                          ring->desc, ring->desc_dma);
}

/* Power management registers */
#define MT_CONN_HIF_ON_LPCTL        0x0000  /* In BAR2 */
#define MT_HIF_LPCTL_HOST_OWN       BIT(0)
#define MT_HIF_LPCTL_FW_OWN         BIT(1)
#define MT_HIF_LPCTL_DRV_OWN        BIT(2)

#define MT_WFSYS_SW_RST_B           0x0140
#define MT_TOP_WFSYS_WAKEUP         0x01a4
#define MT_TOP_MCU_WAKEUP           0x01a8

/* Try to take driver ownership from firmware (BAR2 control registers) */
static int mt7927_driver_own(struct mt7927_dev *dev)
{
    u32 val;
    int i;

    dev_info(&dev->pdev->dev, "Taking driver ownership...\n");

    /* Read current state from BAR2 */
    val = mt7927_ctrl_rr(dev, MT_CONN_HIF_ON_LPCTL);
    dev_info(&dev->pdev->dev, "LPCTL initial: 0x%08x\n", val);

    /* Set driver own bit */
    mt7927_ctrl_wr(dev, MT_CONN_HIF_ON_LPCTL, MT_HIF_LPCTL_DRV_OWN);
    wmb();

    /* Wait for driver own to be acknowledged */
    for (i = 0; i < 100; i++) {
        val = mt7927_ctrl_rr(dev, MT_CONN_HIF_ON_LPCTL);
        if (!(val & MT_HIF_LPCTL_FW_OWN)) {
            dev_info(&dev->pdev->dev, "Driver own acquired: 0x%08x\n", val);
            return 0;
        }
        msleep(10);
    }

    dev_warn(&dev->pdev->dev, "Driver own timeout: 0x%08x (continuing anyway)\n", val);
    return 0; /* Continue anyway - might work */
}

/* Wake up WFSYS (BAR2 control registers) */
static int mt7927_wfsys_wakeup(struct mt7927_dev *dev)
{
    u32 val;

    dev_info(&dev->pdev->dev, "Waking up WFSYS...\n");

    /* Assert wakeup via BAR2 */
    mt7927_ctrl_wr(dev, MT_TOP_WFSYS_WAKEUP, 0x1);
    wmb();
    msleep(10);

    mt7927_ctrl_wr(dev, MT_TOP_MCU_WAKEUP, 0x1);
    wmb();
    msleep(10);

    /* Check status */
    val = mt7927_ctrl_rr(dev, MT_TOP_WFSYS_WAKEUP);
    dev_info(&dev->pdev->dev, "WFSYS_WAKEUP: 0x%08x\n", val);

    val = mt7927_ctrl_rr(dev, MT_TOP_MCU_WAKEUP);
    dev_info(&dev->pdev->dev, "MCU_WAKEUP: 0x%08x\n", val);

    return 0;
}

/* Initialize WPDMA */
static int mt7927_dma_init(struct mt7927_dev *dev)
{
    int ret;
    u32 val;

    dev_info(&dev->pdev->dev, "Initializing WPDMA...\n");

    /* Take driver ownership first */
    ret = mt7927_driver_own(dev);
    if (ret)
        return ret;

    /* Wake up wireless subsystem */
    ret = mt7927_wfsys_wakeup(dev);
    if (ret)
        return ret;

    /* Test register access - read GLO_CFG before write */
    val = mt7927_rr(dev, MT_WFDMA0_GLO_CFG);
    dev_info(&dev->pdev->dev, "GLO_CFG before write: 0x%08x\n", val);

    /* First disable DMA (like mt7921 does) */
    mt7927_wr(dev, MT_WFDMA0_GLO_CFG, 0);
    wmb();
    msleep(10);

    /* Enable BUSY bits - required for DMA to work (mt7921 pattern) */
    mt7927_wr(dev, MT_WFDMA0_BUSY_ENA,
              MT_WFDMA0_BUSY_ENA_TX_FIFO0 |
              MT_WFDMA0_BUSY_ENA_TX_FIFO1 |
              MT_WFDMA0_BUSY_ENA_RX_FIFO);
    wmb();

    /* Reset DMA */
    mt7927_wr(dev, MT_WFDMA0_RST_DTX_PTR, ~0);
    wmb();
    msleep(10);

    /* Allocate FWDL ring */
    ret = mt7927_ring_alloc(dev, &dev->fwdl_ring, MT_FWDL_RING_SIZE, MT_FWDL_RING_IDX);
    if (ret) {
        dev_err(&dev->pdev->dev, "Failed to allocate FWDL ring\n");
        return ret;
    }

    /* Allocate MCU ring */
    ret = mt7927_ring_alloc(dev, &dev->mcu_ring, MT_MCU_RING_SIZE, MT_MCU_RING_IDX);
    if (ret) {
        dev_err(&dev->pdev->dev, "Failed to allocate MCU ring\n");
        return ret;
    }

    /* Enable DMA - use same config as mt7921/mt7925 */
    val = MT_WFDMA0_GLO_CFG_TX_DMA_EN |
          MT_WFDMA0_GLO_CFG_RX_DMA_EN |
          MT_WFDMA0_GLO_CFG_TX_WB_DDONE |
          MT_WFDMA0_GLO_CFG_FIFO_LITTLE_ENDIAN |
          MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
          MT_WFDMA0_GLO_CFG_OMIT_RX_INFO;
    dev_info(&dev->pdev->dev, "Writing GLO_CFG: 0x%08x\n", val);
    mt7927_wr(dev, MT_WFDMA0_GLO_CFG, val);
    wmb();
    msleep(10);

    /* Verify write took effect */
    val = mt7927_rr(dev, MT_WFDMA0_GLO_CFG);
    dev_info(&dev->pdev->dev, "GLO_CFG after write: 0x%08x\n", val);

    if (val == 0x00000000) {
        dev_warn(&dev->pdev->dev, "GLO_CFG write failed! Trying alternative offsets...\n");

        /* Try offset 0x0208 in BAR0 (non-remapped) */
        iowrite32(0x00001041, dev->regs + 0x0208);
        wmb();
        msleep(10);
        val = ioread32(dev->regs + 0x0208);
        dev_info(&dev->pdev->dev, "GLO_CFG BAR0+0x208: 0x%08x\n", val);

        /* Also try BAR2+0x0208 */
        val = ioread32(dev->regs2 + 0x0208);
        dev_info(&dev->pdev->dev, "GLO_CFG BAR2+0x208: 0x%08x\n", val);
    }

    dev_info(&dev->pdev->dev, "WPDMA initialized, GLO_CFG=0x%08x\n", val);

    /* Dump additional DMA status registers for debugging */
    {
        u32 int_sta = mt7927_rr(dev, MT_WFDMA0_BASE_REMAP + 0x0200);
        u32 int_ena = mt7927_rr(dev, MT_WFDMA0_BASE_REMAP + 0x0204);
        u32 busy = mt7927_rr(dev, MT_WFDMA0_BASE_REMAP + 0x0234);
        u32 busy_ena = mt7927_rr(dev, MT_WFDMA0_BUSY_ENA);
        dev_info(&dev->pdev->dev, "DMA: INT_STA=0x%08x INT_ENA=0x%08x BUSY=0x%08x BUSY_ENA=0x%08x\n",
                 int_sta, int_ena, busy, busy_ena);

        /* Also check alternative WFDMA locations */
        dev_info(&dev->pdev->dev, "GLO_CFG locations: BAR0+0x2208=0x%08x BAR0+0x208=0x%08x BAR2+0x208=0x%08x\n",
                 ioread32(dev->regs + 0x2208),
                 ioread32(dev->regs + 0x0208),
                 ioread32(dev->regs2 + 0x0208));
    }

    return 0;
}

/* Send MCU command */
static int mt7927_mcu_send_msg(struct mt7927_dev *dev, int cmd,
                               const void *data, int len, bool wait_resp)
{
    struct mt7927_ring *ring = &dev->fwdl_ring;
    struct mt76_desc *desc;
    struct mt76_mcu_txd *txd;
    void *buf;
    dma_addr_t buf_dma;
    int idx;
    u32 val;
    u8 seq;

    if (len > 4096 - sizeof(*txd)) {
        dev_err(&dev->pdev->dev, "MCU message too large: %d\n", len);
        return -EINVAL;
    }

    spin_lock(&dev->mcu_lock);

    idx = ring->head;
    ring->head = (ring->head + 1) % ring->size;

    seq = ++dev->mcu_seq & 0xf;
    if (!seq)
        seq = ++dev->mcu_seq & 0xf;

    spin_unlock(&dev->mcu_lock);

    buf = ring->buf[idx];
    buf_dma = ring->buf_dma[idx];
    desc = &ring->desc[idx];

    /* Build TXD header */
    txd = buf;
    memset(txd, 0, sizeof(*txd));

    /* TXD word 0 */
    val = FIELD_PREP(MT_TXD0_TX_BYTES, sizeof(*txd) + len) |
          FIELD_PREP(MT_TXD0_PKT_FMT, MT_TX_TYPE_CMD) |
          FIELD_PREP(MT_TXD0_Q_IDX, MT_TX_MCU_PORT_RX_Q0);
    txd->txd[0] = cpu_to_le32(val);

    /* TXD word 1 - long format */
    txd->txd[1] = cpu_to_le32(BIT(31)); /* LONG_FORMAT */

    txd->len = cpu_to_le16(sizeof(*txd) + len - 8); /* exclude txd[0..1] */
    txd->pq_id = cpu_to_le16(MCU_PQ_ID(0, 0));
    txd->cid = cmd & 0xff;
    txd->pkt_type = MCU_PKT_ID;
    txd->seq = seq;

    /* Copy data after TXD */
    if (len > 0 && data)
        memcpy(buf + sizeof(*txd), data, len);

    /* Set up descriptor */
    desc->buf0 = cpu_to_le32(lower_32_bits(buf_dma));
    desc->buf1 = cpu_to_le32(upper_32_bits(buf_dma));
    desc->ctrl = cpu_to_le32(FIELD_PREP(MT_DMA_CTL_SD_LEN0, sizeof(*txd) + len) |
                             MT_DMA_CTL_LAST_SEC0);
    desc->info = 0;
    wmb();

    /* Update CPU index to trigger DMA */
    mt7927_wr(dev, MT_RING_CIDX(MT_FWDL_RING_IDX), ring->head);

    dev_info(&dev->pdev->dev, "MCU cmd 0x%02x seq=%d len=%d idx=%d CIDX=%d\n",
            cmd, seq, len, idx, ring->head);

    /* Wait for DMA completion */
    msleep(10);

    /* Check if DMA processed the descriptor */
    {
        /* Read back CIDX and DIDX using proper register offsets */
        u32 cidx = mt7927_rr(dev, MT_RING_CIDX(MT_FWDL_RING_IDX));
        u32 didx = mt7927_rr(dev, MT_RING_DIDX(MT_FWDL_RING_IDX));
        u32 dma_done = le32_to_cpu(desc->ctrl) & MT_DMA_CTL_DMA_DONE;

        if (idx < 3) { /* Only print first few to avoid spam */
            dev_info(&dev->pdev->dev, "  CIDX=0x%x: wrote %d, read %u\n",
                     MT_RING_CIDX(MT_FWDL_RING_IDX), ring->head, cidx);
            dev_info(&dev->pdev->dev, "  DIDX=0x%x: %u, DMA_DONE=%d\n",
                     MT_RING_DIDX(MT_FWDL_RING_IDX), didx, dma_done ? 1 : 0);
        }
    }

    if (wait_resp) {
        /* TODO: implement response waiting via RX ring */
        msleep(50);
    }

    return 0;
}

/* Send firmware scatter command */
static int mt7927_mcu_send_firmware(struct mt7927_dev *dev, const u8 *data, int len)
{
    int max_len = 4096 - sizeof(struct mt76_mcu_txd);
    int ret;

    while (len > 0) {
        int cur_len = min(len, max_len);

        ret = mt7927_mcu_send_msg(dev, MCU_CMD_FW_SCATTER, data, cur_len, false);
        if (ret)
            return ret;

        data += cur_len;
        len -= cur_len;
    }

    return 0;
}

/* Get/release patch semaphore */
static int mt7927_mcu_patch_sem_ctrl(struct mt7927_dev *dev, bool get)
{
    struct {
        __le32 op;
    } req = {
        .op = cpu_to_le32(get ? PATCH_SEM_GET : PATCH_SEM_RELEASE),
    };

    return mt7927_mcu_send_msg(dev, MCU_CMD_PATCH_SEM_CONTROL,
                               &req, sizeof(req), true);
}

/* Initialize download */
static int mt7927_mcu_init_download(struct mt7927_dev *dev, u32 addr, u32 len, u32 mode)
{
    struct {
        __le32 addr;
        __le32 len;
        __le32 mode;
    } req = {
        .addr = cpu_to_le32(addr),
        .len = cpu_to_le32(len),
        .mode = cpu_to_le32(mode),
    };
    int cmd;

    /* Use PATCH_START_REQ for patch addresses */
    if (addr == 0x200000 || addr == 0x900000 || addr == 0xe0002800)
        cmd = MCU_CMD_PATCH_START_REQ;
    else
        cmd = MCU_CMD_TARGET_ADDRESS_LEN_REQ;

    dev_info(&dev->pdev->dev, "Init download: addr=0x%08x len=%u mode=0x%x cmd=0x%x\n",
             addr, len, mode, cmd);

    return mt7927_mcu_send_msg(dev, cmd, &req, sizeof(req), true);
}

/* Start patch */
static int mt7927_mcu_start_patch(struct mt7927_dev *dev)
{
    struct {
        u8 check_crc;
        u8 reserved[3];
    } req = {
        .check_crc = 0,
    };

    return mt7927_mcu_send_msg(dev, MCU_CMD_PATCH_FINISH_REQ,
                               &req, sizeof(req), true);
}

/* Start firmware */
static int mt7927_mcu_start_firmware(struct mt7927_dev *dev, u32 addr, u32 option)
{
    struct {
        __le32 option;
        __le32 addr;
    } req = {
        .option = cpu_to_le32(option),
        .addr = cpu_to_le32(addr),
    };

    return mt7927_mcu_send_msg(dev, MCU_CMD_FW_START_REQ,
                               &req, sizeof(req), true);
}

/* Patch header structures */
struct mt76_connac2_patch_hdr {
    char build_date[16];
    char platform[4];
    __be32 hw_sw_ver;
    __be32 patch_ver;
    __be16 checksum;
    u8 reserved[2];
    struct {
        __be32 patch_ver;
        __be32 subsys;
        __be32 feature;
        __be32 n_region;
        __be32 crc;
        u8 reserved[4];
    } desc;
} __packed;

struct mt76_connac2_patch_sec {
    __be32 type;
    char reserved1[4];
    __be32 offs;
    union {
        struct {
            __be32 sec_key_idx;
            __be32 align_len;
            __be32 reserved2[2];
        };
        struct {
            __be32 addr;
            __be32 len;
            __be32 sec_key_idx;
            __be32 reserved3;
        } info;
    };
} __packed;

#define PATCH_SEC_TYPE_MASK     0x03
#define PATCH_SEC_TYPE_INFO     0x2

/* Load patch firmware */
static int mt7927_load_patch(struct mt7927_dev *dev)
{
    const struct mt76_connac2_patch_hdr *hdr;
    const struct firmware *fw = NULL;
    int ret, i;

    dev_info(&dev->pdev->dev, "Loading patch firmware...\n");

    /* Get semaphore */
    ret = mt7927_mcu_patch_sem_ctrl(dev, true);
    if (ret) {
        dev_err(&dev->pdev->dev, "Failed to get patch semaphore\n");
        return ret;
    }

    ret = request_firmware(&fw, fw_patch, &dev->pdev->dev);
    if (ret) {
        dev_err(&dev->pdev->dev, "Failed to load patch: %s (%d)\n", fw_patch, ret);
        goto out_sem;
    }

    if (!fw || !fw->data || fw->size < sizeof(*hdr)) {
        dev_err(&dev->pdev->dev, "Invalid patch firmware\n");
        ret = -EINVAL;
        goto out;
    }

    hdr = (const void *)fw->data;
    dev_info(&dev->pdev->dev, "Patch: HW/SW Ver=0x%x, Build=%.16s\n",
             be32_to_cpu(hdr->hw_sw_ver), hdr->build_date);

    /* Process each region */
    for (i = 0; i < be32_to_cpu(hdr->desc.n_region); i++) {
        struct mt76_connac2_patch_sec *sec;
        u32 len, addr;
        const u8 *dl;

        sec = (void *)(fw->data + sizeof(*hdr) + i * sizeof(*sec));
        if ((be32_to_cpu(sec->type) & PATCH_SEC_TYPE_MASK) != PATCH_SEC_TYPE_INFO) {
            dev_warn(&dev->pdev->dev, "Skipping non-info section %d\n", i);
            continue;
        }

        addr = be32_to_cpu(sec->info.addr);
        len = be32_to_cpu(sec->info.len);
        dl = fw->data + be32_to_cpu(sec->offs);

        dev_info(&dev->pdev->dev, "Patch region %d: addr=0x%08x len=%u\n",
                 i, addr, len);

        ret = mt7927_mcu_init_download(dev, addr, len, DL_MODE_NEED_RSP);
        if (ret) {
            dev_err(&dev->pdev->dev, "Download init failed\n");
            goto out;
        }

        ret = mt7927_mcu_send_firmware(dev, dl, len);
        if (ret) {
            dev_err(&dev->pdev->dev, "Failed to send patch data\n");
            goto out;
        }
    }

    ret = mt7927_mcu_start_patch(dev);
    if (ret)
        dev_err(&dev->pdev->dev, "Failed to start patch\n");
    else
        dev_info(&dev->pdev->dev, "Patch started successfully\n");

out:
    release_firmware(fw);
out_sem:
    mt7927_mcu_patch_sem_ctrl(dev, false);
    return ret;
}

/* RAM firmware trailer */
struct mt76_connac2_fw_trailer {
    u8 chip_id;
    u8 eco_code;
    u8 n_region;
    u8 format_ver;
    u8 format_flag;
    u8 reserved[2];
    char fw_ver[10];
    char build_date[15];
    u32 crc;
} __packed;

struct mt76_connac2_fw_region {
    __le32 decomp_crc;
    __le32 decomp_len;
    __le32 decomp_blk_sz;
    u8 reserved[4];
    __le32 addr;
    __le32 len;
    u8 feature_set;
    u8 reserved2[15];
} __packed;

#define FW_FEATURE_NON_DL           BIT(2)
#define FW_FEATURE_OVERRIDE_ADDR    BIT(3)
#define FW_START_OVERRIDE           BIT(0)

/* Load RAM firmware */
static int mt7927_load_ram(struct mt7927_dev *dev)
{
    const struct mt76_connac2_fw_trailer *hdr;
    const struct firmware *fw = NULL;
    int ret, i, offset = 0;
    u32 override = 0;

    dev_info(&dev->pdev->dev, "Loading RAM firmware...\n");

    ret = request_firmware(&fw, fw_ram, &dev->pdev->dev);
    if (ret) {
        dev_err(&dev->pdev->dev, "Failed to load RAM: %s (%d)\n", fw_ram, ret);
        return ret;
    }

    if (!fw || !fw->data || fw->size < sizeof(*hdr)) {
        dev_err(&dev->pdev->dev, "Invalid RAM firmware\n");
        ret = -EINVAL;
        goto out;
    }

    hdr = (const void *)(fw->data + fw->size - sizeof(*hdr));
    dev_info(&dev->pdev->dev, "RAM: Ver=%.10s, Build=%.15s, regions=%d\n",
             hdr->fw_ver, hdr->build_date, hdr->n_region);

    /* Process each region */
    for (i = 0; i < hdr->n_region; i++) {
        const struct mt76_connac2_fw_region *region;
        u32 len, addr;

        region = (const void *)((const u8 *)hdr -
                                (hdr->n_region - i) * sizeof(*region));

        len = le32_to_cpu(region->len);
        addr = le32_to_cpu(region->addr);

        dev_info(&dev->pdev->dev, "RAM region %d: addr=0x%08x len=%u features=0x%02x\n",
                 i, addr, len, region->feature_set);

        if (region->feature_set & FW_FEATURE_NON_DL) {
            offset += len;
            continue;
        }

        if (region->feature_set & FW_FEATURE_OVERRIDE_ADDR)
            override = addr;

        ret = mt7927_mcu_init_download(dev, addr, len, DL_MODE_NEED_RSP);
        if (ret) {
            dev_err(&dev->pdev->dev, "Download init failed for region %d\n", i);
            goto out;
        }

        ret = mt7927_mcu_send_firmware(dev, fw->data + offset, len);
        if (ret) {
            dev_err(&dev->pdev->dev, "Failed to send RAM data\n");
            goto out;
        }

        offset += len;
    }

    /* Start firmware */
    ret = mt7927_mcu_start_firmware(dev, override,
                                    override ? FW_START_OVERRIDE : 0);
    if (ret)
        dev_err(&dev->pdev->dev, "Failed to start firmware\n");
    else
        dev_info(&dev->pdev->dev, "Firmware started, override=0x%x\n", override);

out:
    release_firmware(fw);
    return ret;
}

/* FW_STATUS register in BAR2 */
#define MT_FW_STATUS                0x0200
#define MT_FW_STATUS_READY          0x0     /* Firmware running */
#define MT_FW_STATUS_WAITING        0xffff10f1  /* Waiting for firmware */

/* Check if firmware is ready */
static bool mt7927_fw_ready(struct mt7927_dev *dev)
{
    u32 val, fw_status;
    int i;

    for (i = 0; i < 150; i++) {
        /* Check FW_STATUS register in BAR2 */
        fw_status = mt7927_ctrl_rr(dev, MT_FW_STATUS);

        /* Also check TOP_MISC2 for N9 ready bit - try both BARs */
        val = mt7927_ctrl_rr(dev, MT_TOP_MISC2);

        if (i % 10 == 0) {
            dev_info(&dev->pdev->dev, "FW check %d: FW_STATUS=0x%08x TOP_MISC2=0x%08x\n",
                     i, fw_status, val);
        }

        /* Check if firmware changed from waiting state */
        if (fw_status != MT_FW_STATUS_WAITING && fw_status != 0x00000001) {
            dev_info(&dev->pdev->dev, "FW_STATUS changed: 0x%08x\n", fw_status);
        }

        if (val & MT_TOP_MISC2_FW_N9_RDY) {
            dev_info(&dev->pdev->dev, "Firmware ready! FW_STATUS=0x%08x TOP_MISC2=0x%08x\n",
                     fw_status, val);
            return true;
        }
        msleep(10);
    }

    dev_err(&dev->pdev->dev, "Firmware not ready: FW_STATUS=0x%08x TOP_MISC2=0x%08x\n",
            fw_status, val);

    /* Check memory state */
    val = mt7927_mem_rr(dev, 0);
    dev_info(&dev->pdev->dev, "Memory[0x0]: 0x%08x\n", val);

    return false;
}

static int mt7927_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct mt7927_dev *dev;
    int ret;
    u32 val;

    dev_info(&pdev->dev, "MT7927 MCU Firmware Driver\n");

    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    dev->pdev = pdev;
    spin_lock_init(&dev->mcu_lock);
    pci_set_drvdata(pdev, dev);

    ret = pci_enable_device(pdev);
    if (ret)
        goto err_free;

    pci_set_master(pdev);

    ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
    if (ret)
        goto err_disable;

    ret = pci_request_regions(pdev, "mt7927");
    if (ret)
        goto err_disable;

    dev->regs = pci_iomap(pdev, 0, 0);
    dev->regs2 = pci_iomap(pdev, 2, 0);
    if (!dev->regs || !dev->regs2) {
        ret = -ENOMEM;
        goto err_release;
    }

    /* Check chip state */
    val = mt7927_ctrl_rr(dev, 0x0200);  /* FW_STATUS in BAR2 */
    dev_info(&pdev->dev, "Initial FW_STATUS (BAR2+0x200): 0x%08x\n", val);

    val = mt7927_rr(dev, MT_WFDMA0_GLO_CFG);  /* GLO_CFG in BAR0 (remapped) */
    dev_info(&pdev->dev, "Initial GLO_CFG (BAR0+0x%x): 0x%08x\n", MT_WFDMA0_GLO_CFG, val);

    val = mt7927_mem_rr(dev, 0);
    dev_info(&pdev->dev, "Initial Memory (BAR0+0x0): 0x%08x\n", val);

    if (val == 0xffffffff) {
        dev_warn(&pdev->dev, "Memory reads 0xffffffff (may be OK before FW)\n");
    }

    /* Initialize DMA */
    ret = mt7927_dma_init(dev);
    if (ret)
        goto err_unmap;

    /* Load patch */
    ret = mt7927_load_patch(dev);
    if (ret) {
        dev_warn(&pdev->dev, "Patch load failed (may be OK)\n");
    }

    /* Load RAM firmware */
    ret = mt7927_load_ram(dev);
    if (ret) {
        dev_err(&pdev->dev, "RAM load failed\n");
        goto err_dma;
    }

    /* Check if firmware started */
    if (mt7927_fw_ready(dev)) {
        dev_info(&pdev->dev, "MT7927 firmware loaded successfully!\n");

        /* Check memory activation */
        val = ioread32(dev->regs);
        dev_info(&pdev->dev, "Memory state: 0x%08x\n", val);
    }

    return 0;

err_dma:
    mt7927_ring_free(dev, &dev->fwdl_ring);
    mt7927_ring_free(dev, &dev->mcu_ring);
err_unmap:
    if (dev->regs2)
        pci_iounmap(pdev, dev->regs2);
    if (dev->regs)
        pci_iounmap(pdev, dev->regs);
err_release:
    pci_release_regions(pdev);
err_disable:
    pci_disable_device(pdev);
err_free:
    kfree(dev);
    return ret;
}

static void mt7927_remove(struct pci_dev *pdev)
{
    struct mt7927_dev *dev = pci_get_drvdata(pdev);

    dev_info(&pdev->dev, "Removing MT7927 device\n");

    mt7927_ring_free(dev, &dev->fwdl_ring);
    mt7927_ring_free(dev, &dev->mcu_ring);

    if (dev->regs2)
        pci_iounmap(pdev, dev->regs2);
    if (dev->regs)
        pci_iounmap(pdev, dev->regs);

    pci_release_regions(pdev);
    pci_disable_device(pdev);
    kfree(dev);
}

static struct pci_device_id mt7927_ids[] = {
    { PCI_DEVICE(MT7927_VENDOR_ID, MT7927_DEVICE_ID) },
    { },
};
MODULE_DEVICE_TABLE(pci, mt7927_ids);

static struct pci_driver mt7927_driver = {
    .name = "mt7927_mcu_fw",
    .id_table = mt7927_ids,
    .probe = mt7927_probe,
    .remove = mt7927_remove,
};

module_pci_driver(mt7927_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT7927 MCU Firmware Loading Driver");
MODULE_AUTHOR("MT7927 Linux Driver Project");
MODULE_FIRMWARE("mediatek/mt7925/WIFI_MT7925_PATCH_MCU_1_1_hdr.bin");
MODULE_FIRMWARE("mediatek/mt7925/WIFI_RAM_CODE_MT7925_1_1.bin");
