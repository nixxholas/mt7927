/*
 * MT7927 PCI ID Shim Module
 *
 * This module adds the MT7927 PCI device ID to the mt7925e driver
 * at runtime using the kernel's dynamic PCI ID mechanism.
 *
 * The MT7927 is architecturally identical to MT7925, just with
 * 320MHz channel width support added.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define MT7927_VENDOR_ID    0x14c3
#define MT7927_DEVICE_ID    0x7927
#define MT7925_DRIVER_NAME  "mt7925e"

static struct pci_driver *mt7925_driver = NULL;

static int __init mt7927_shim_init(void)
{
    struct device_driver *drv;
    struct pci_dev *pdev;
    int ret = 0;

    pr_info("mt7927_shim: Loading MT7927 PCI ID shim\n");

    /* Find the mt7925e driver */
    drv = driver_find(MT7925_DRIVER_NAME, &pci_bus_type);
    if (!drv) {
        pr_err("mt7927_shim: mt7925e driver not found. Load mt7925e first.\n");
        return -ENODEV;
    }

    mt7925_driver = to_pci_driver(drv);
    pr_info("mt7927_shim: Found mt7925e driver at %p\n", mt7925_driver);

    /* Add MT7927 device ID to the driver */
    ret = pci_add_dynid(mt7925_driver, MT7927_VENDOR_ID, MT7927_DEVICE_ID,
                        PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0);
    if (ret) {
        pr_err("mt7927_shim: Failed to add dynamic ID: %d\n", ret);
        return ret;
    }

    pr_info("mt7927_shim: Added MT7927 (14c3:7927) to mt7925e driver\n");

    /* Try to find and bind any MT7927 devices */
    pdev = pci_get_device(MT7927_VENDOR_ID, MT7927_DEVICE_ID, NULL);
    if (pdev) {
        pr_info("mt7927_shim: Found MT7927 device at %s\n", pci_name(pdev));

        /* If device isn't bound, try to trigger a rescan */
        if (!pdev->driver) {
            pr_info("mt7927_shim: Device not bound, triggering driver probe\n");
            /* The dynamic ID should cause automatic binding on next bus scan */
            pci_dev_put(pdev);

            /* Trigger a rescan */
            if (pci_rescan_bus(pdev->bus->parent ? pdev->bus->parent : pdev->bus)) {
                pr_info("mt7927_shim: Bus rescan triggered\n");
            }
        } else {
            pr_info("mt7927_shim: Device already bound to %s\n",
                    pdev->driver->name);
            pci_dev_put(pdev);
        }
    } else {
        pr_warn("mt7927_shim: No MT7927 device found in system\n");
    }

    pr_info("mt7927_shim: Initialization complete\n");
    return 0;
}

static void __exit mt7927_shim_exit(void)
{
    pr_info("mt7927_shim: Unloading (dynamic ID remains registered)\n");
    /* Note: The dynamic ID remains registered with the driver */
}

module_init(mt7927_shim_init);
module_exit(mt7927_shim_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("MT7927 Linux Driver Project");
MODULE_DESCRIPTION("MT7927 PCI ID shim for mt7925e driver");
MODULE_SOFTDEP("pre: mt7925e");
