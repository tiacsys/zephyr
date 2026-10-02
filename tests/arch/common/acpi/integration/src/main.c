/*
 * Copyright (c) 2023 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/acpi/acpi.h>

#define APCI_TEST_DEV ACPI_DT_HAS_HID(DT_ALIAS(acpi_dev))

#if APCI_TEST_DEV
#define DEV_HID ACPI_DT_HID(DT_ALIAS(acpi_dev))
#define DEV_UID ACPI_DT_UID(DT_ALIAS(acpi_dev))
#else
#define DEV_HID NULL
#define DEV_UID NULL
#endif

/**
 * @brief Verify that acpi_table_get() returns the MCFG table.
 *
 * @details
 * The MCFG table gives the base addresses of the PCI Express configuration
 * space. The test reads the instance 0 of this table from the ACPI tables of
 * the platform.
 *
 * Test steps:
 * - Call acpi_table_get() with the signature "MCFG" and the instance 0.
 *
 * Expected result:
 * - acpi_table_get() returns a pointer that is not NULL.
 *
 * @testid{TSPEC-ARCHCOMMON-033}
 * @draft
 * @see acpi_table_get()
 */
ZTEST(acpi, test_mcfg_table)
{
	struct acpi_mcfg *mcfg;

	mcfg = acpi_table_get("MCFG", 0);

	zassert_not_null(mcfg, "Failed to get MCFG table");
}

/**
 * @brief Verify that the ACPI device of the acpi_dev devicetree alias exists
 * and has current resource settings.
 *
 * @details
 * The test gets the ACPI device with the HID and the UID of the acpi_dev
 * devicetree alias. Then it reads the current resource settings of this device.
 * If the acpi_dev alias has no HID, ztest skips the test.
 *
 * Test steps:
 * - Call acpi_device_get() with the HID and the UID of the acpi_dev alias.
 * - Call acpi_current_resource_get() with the path of the device.
 *
 * Expected result:
 * - acpi_device_get() returns a device that is not NULL.
 * - acpi_current_resource_get() returns 0.
 *
 * @testid{TSPEC-ARCHCOMMON-034}
 * @draft
 * @see acpi_device_get(), acpi_current_resource_get()
 */
ZTEST(acpi, test_dev_enum)
{
	struct acpi_dev *dev;
	ACPI_RESOURCE *res_lst;
	int ret;

	Z_TEST_SKIP_IFNDEF(APCI_TEST_DEV);

	dev = acpi_device_get(DEV_HID, DEV_UID);

	zassert_not_null(dev, "Failed to get acpi device with given HID");

	ret = acpi_current_resource_get(dev->path, &res_lst);

	zassert_ok(ret, "Failed to get current resource setting");
}

/**
 * @brief Verify that the MMIO and the IRQ resources of the ACPI test device are
 * available.
 *
 * @details
 * The test gets the ACPI device of the acpi_dev devicetree alias. Then it reads
 * the MMIO regions and the IRQ vectors of this device into local arrays. If the
 * acpi_dev alias has no HID, ztest skips the test.
 *
 * Test steps:
 * - Call acpi_device_get() with the HID and the UID of the acpi_dev alias.
 * - Call acpi_device_mmio_get() with an array of CONFIG_ACPI_MMIO_ENTRIES_MAX
 *   entries.
 * - Call acpi_device_irq_get() with an array of CONFIG_ACPI_IRQ_VECTOR_MAX
 *   entries.
 *
 * Expected result:
 * - acpi_device_get() returns a device that is not NULL.
 * - acpi_device_mmio_get() returns 0.
 * - acpi_device_irq_get() returns 0.
 *
 * @testid{TSPEC-ARCHCOMMON-035}
 * @draft
 * @see acpi_device_get(), acpi_device_mmio_get(), acpi_device_irq_get()
 */
ZTEST(acpi, test_resource_enum)
{
	struct acpi_dev *dev;
	struct acpi_irq_resource irq_res;
	struct acpi_mmio_resource mmio_res;
	uint16_t irqs[CONFIG_ACPI_IRQ_VECTOR_MAX];
	struct acpi_reg_base reg_base[CONFIG_ACPI_MMIO_ENTRIES_MAX];
	int ret;

	Z_TEST_SKIP_IFNDEF(APCI_TEST_DEV);

	dev = acpi_device_get(DEV_HID, DEV_UID);

	zassert_not_null(dev, "Failed to get acpi device with given HID");

	mmio_res.mmio_max = ARRAY_SIZE(reg_base);
	mmio_res.reg_base = reg_base;
	ret = acpi_device_mmio_get(dev, &mmio_res);

	zassert_ok(ret, "Failed to get MMIO resources");

	irq_res.irq_vector_max = ARRAY_SIZE(irqs);
	irq_res.irqs = irqs;
	ret = acpi_device_irq_get(dev, &irq_res);

	zassert_ok(ret, "Failed to get IRQ resources");
}

ZTEST_SUITE(acpi, NULL, NULL, NULL, NULL, NULL);
