/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <string.h>
#include <inttypes.h>

#include "mock.h"

#include <zephyr/acpi/acpi.h>
#include <arch/common/acpi/acpi.c>
#include "assert.h"

#include <zephyr/fff.h>
DEFINE_FFF_GLOBALS;

struct DMAR {
	ACPI_TABLE_DMAR header;

	/* Hardware Unit 0 */
	struct {
		ACPI_DMAR_HARDWARE_UNIT header;

		struct {
			ACPI_DMAR_DEVICE_SCOPE header;
			ACPI_DMAR_PCI_PATH path0;
		} ds0;

		struct {
			ACPI_DMAR_DEVICE_SCOPE header;
			ACPI_DMAR_PCI_PATH path0;
		} ds1;
	} unit0;

	/* Hardware Unit 1 */
	struct {
		ACPI_DMAR_HARDWARE_UNIT header;

		struct {
			ACPI_DMAR_DEVICE_SCOPE header;
			ACPI_DMAR_PCI_PATH path0;
		} ds0;

		struct {
			ACPI_DMAR_DEVICE_SCOPE header;
			ACPI_DMAR_PCI_PATH path0;
		} ds1;
	} unit1;
};

static struct DMAR dmar0;

static void dmar_initialize(struct DMAR *dmar)
{
	dmar->header.Header.Length = sizeof(struct DMAR);

	dmar->unit0.header.Header.Length = sizeof(dmar->unit0);
	dmar->unit0.ds0.header.Length = sizeof(dmar->unit0.ds0);
	dmar->unit0.ds1.header.Length = sizeof(dmar->unit0.ds1);

	dmar->unit1.header.Header.Length = sizeof(dmar->unit1);
	dmar->unit1.ds0.header.Length = sizeof(dmar->unit1.ds0);
	dmar->unit1.ds1.header.Length = sizeof(dmar->unit1.ds1);
}

/**
 * @brief Verify that the lib_acpi test suite builds and runs an empty test.
 *
 * @details
 * The test body is empty and has no assertions. The test passes when the test
 * image with the mocked ACPI library builds and starts.
 *
 * Test steps:
 * - Run the empty test body.
 *
 * Expected result:
 * - The test passes.
 *
 * @testid{TSPEC-ARCHCOMMON-036}
 * @draft
 */
ZTEST(lib_acpi, test_nop)
{
}

static void count_subtables(ACPI_DMAR_HEADER *subtable, void *arg)
{
	uint8_t *count = arg;

	(*count)++;
}

FAKE_VOID_FUNC(subtable_nop, ACPI_DMAR_HEADER *, void *);

/**
 * @brief Verify that acpi_dmar_foreach_subtable() calls the callback one time
 * for each hardware unit.
 *
 * @details
 * The test uses a static DMAR table with two hardware units. Each hardware unit
 * has two device scopes. The callback counts the subtables that it gets.
 *
 * Test steps:
 * - Initialize the static DMAR table with the correct lengths.
 * - Call acpi_dmar_foreach_subtable() with a callback that increments a
 *   counter.
 *
 * Expected result:
 * - The counter is 2.
 *
 * @testid{TSPEC-ARCHCOMMON-037}
 * @draft
 * @see acpi_dmar_foreach_subtable()
 */
ZTEST(lib_acpi, test_dmar_foreach_subtable)
{
	uint8_t count = 0;

	dmar_initialize(&dmar0);

	acpi_dmar_foreach_subtable((void *)&dmar0, count_subtables, &count);
	zassert_equal(count, 2);

	TC_PRINT("Counted %u hardware units\n", count);
}

/**
 * @brief Verify that acpi_dmar_foreach_subtable() asserts on a hardware unit
 * with a length of 0.
 *
 * @details
 * The test uses a static DMAR table with two hardware units. Each hardware unit
 * has two device scopes. A subtable must be at least as long as its header.
 *
 * Test steps:
 * - Initialize the static DMAR table with the correct lengths.
 * - Set the length of the second hardware unit to 0.
 * - Call expect_assert() to tell the mocked assert handler to expect an
 *   assertion.
 * - Call acpi_dmar_foreach_subtable() with a callback that does nothing.
 *
 * Expected result:
 * - An assertion in acpi_dmar_foreach_subtable() fails. The mocked assert
 *   handler then marks the test as passed and stops it.
 * - The code after the call does not run.
 *
 * @testid{TSPEC-ARCHCOMMON-038}
 * @draft
 * @see acpi_dmar_foreach_subtable()
 */
ZTEST(lib_acpi, test_dmar_foreach_subtable_invalid_unit_size_zero)
{
	ACPI_DMAR_HARDWARE_UNIT *hu = &dmar0.unit1.header;

	dmar_initialize(&dmar0);

	/* Set invalid hardware unit size */
	hu->Header.Length = 0;

	expect_assert();

	/* Expect assert, use fake void function as a callback */
	acpi_dmar_foreach_subtable((void *)&dmar0, subtable_nop, NULL);

	zassert_unreachable("Missed assert catch");
}

/**
 * @brief Verify that acpi_dmar_foreach_subtable() asserts on a hardware unit
 * that goes past the end of the DMAR table.
 *
 * @details
 * The test uses a static DMAR table with two hardware units. Each hardware unit
 * has two device scopes. A subtable must not be longer than the rest of the
 * DMAR table.
 *
 * Test steps:
 * - Initialize the static DMAR table with the correct lengths.
 * - Set the length of the second hardware unit to its size plus 1.
 * - Call expect_assert() to tell the mocked assert handler to expect an
 *   assertion.
 * - Call acpi_dmar_foreach_subtable() with a callback that does nothing.
 *
 * Expected result:
 * - An assertion in acpi_dmar_foreach_subtable() fails. The mocked assert
 *   handler then marks the test as passed and stops it.
 * - The code after the call does not run.
 *
 * @testid{TSPEC-ARCHCOMMON-039}
 * @draft
 * @see acpi_dmar_foreach_subtable()
 */
ZTEST(lib_acpi, test_dmar_foreach_subtable_invalid_unit_size_big)
{
	ACPI_DMAR_HARDWARE_UNIT *hu = &dmar0.unit1.header;

	dmar_initialize(&dmar0);

	/* Set invalid hardware unit size */
	hu->Header.Length = sizeof(dmar0.unit1) + 1;

	expect_assert();

	/* Expect assert, use fake void function as a callback */
	acpi_dmar_foreach_subtable((void *)&dmar0, subtable_nop, NULL);

	zassert_unreachable("Missed assert catch");
}

static void count_devscopes(ACPI_DMAR_DEVICE_SCOPE *devscope, void *arg)
{
	uint8_t *count = arg;

	(*count)++;
}

FAKE_VOID_FUNC(devscope_nop, ACPI_DMAR_DEVICE_SCOPE *, void *);

/**
 * @brief Verify that acpi_dmar_foreach_devscope() calls the callback one time
 * for each device scope of a hardware unit.
 *
 * @details
 * The test uses a static DMAR table with two hardware units. Each hardware unit
 * has two device scopes. The callback counts the device scopes that it gets.
 *
 * Test steps:
 * - Initialize the static DMAR table with the correct lengths.
 * - Call acpi_dmar_foreach_devscope() on the first hardware unit, with a
 *   callback that increments a counter.
 *
 * Expected result:
 * - The counter is 2.
 *
 * @testid{TSPEC-ARCHCOMMON-040}
 * @draft
 * @see acpi_dmar_foreach_devscope()
 */
ZTEST(lib_acpi, test_dmar_foreach_devscope)
{
	ACPI_DMAR_HARDWARE_UNIT *hu = &dmar0.unit0.header;
	uint8_t count = 0;

	dmar_initialize(&dmar0);

	acpi_dmar_foreach_devscope(hu, count_devscopes, &count);
	zassert_equal(count, 2);

	TC_PRINT("Counted %u device scopes\n", count);
}

/**
 * @brief Verify that acpi_dmar_foreach_devscope() asserts on a hardware unit
 * with a length of 0.
 *
 * @details
 * The test uses a static DMAR table with two hardware units. Each hardware unit
 * has two device scopes. A hardware unit must be at least as long as its
 * header.
 *
 * Test steps:
 * - Initialize the static DMAR table with the correct lengths.
 * - Set the length of the first hardware unit to 0.
 * - Call expect_assert() to tell the mocked assert handler to expect an
 *   assertion.
 * - Call acpi_dmar_foreach_devscope() with a callback that does nothing.
 *
 * Expected result:
 * - An assertion in acpi_dmar_foreach_devscope() fails. The mocked assert
 *   handler then marks the test as passed and stops it.
 * - The code after the call does not run.
 *
 * @testid{TSPEC-ARCHCOMMON-041}
 * @draft
 * @see acpi_dmar_foreach_devscope()
 */
ZTEST(lib_acpi, test_dmar_foreach_devscope_invalid_unit_size)
{
	ACPI_DMAR_HARDWARE_UNIT *hu = &dmar0.unit0.header;

	dmar_initialize(&dmar0);

	/* Set invalid hardware unit size */
	hu->Header.Length = 0;

	expect_assert();

	/* Expect assert, use fake void function as a callback */
	acpi_dmar_foreach_devscope(hu, devscope_nop, NULL);

	zassert_unreachable("Missed assert catch");
}

/**
 * @brief Verify that acpi_dmar_foreach_devscope() asserts on a device scope
 * with a length of 0.
 *
 * @details
 * The test uses a static DMAR table with two hardware units. Each hardware unit
 * has two device scopes. A device scope must be at least as long as its header.
 *
 * Test steps:
 * - Initialize the static DMAR table with the correct lengths.
 * - Set the length of the first device scope of the first hardware unit to 0.
 * - Call expect_assert() to tell the mocked assert handler to expect an
 *   assertion.
 * - Call acpi_dmar_foreach_devscope() with a callback that does nothing.
 *
 * Expected result:
 * - An assertion in acpi_dmar_foreach_devscope() fails. The mocked assert
 *   handler then marks the test as passed and stops it.
 * - The code after the call does not run.
 *
 * @testid{TSPEC-ARCHCOMMON-042}
 * @draft
 * @see acpi_dmar_foreach_devscope()
 */
ZTEST(lib_acpi, test_dmar_foreach_devscope_invalid_devscope_size_zero)
{
	ACPI_DMAR_HARDWARE_UNIT *hu = &dmar0.unit0.header;
	ACPI_DMAR_DEVICE_SCOPE *devscope = &dmar0.unit0.ds0.header;

	dmar_initialize(&dmar0);

	/* Set invalid device scope size */
	devscope->Length = 0;

	expect_assert();

	/* Expect assert, use fake void function as a callback */
	acpi_dmar_foreach_devscope(hu, devscope_nop, NULL);

	zassert_unreachable("Missed assert catch");
}

/**
 * @brief Verify that acpi_dmar_foreach_devscope() asserts on a device scope
 * that goes past the end of its hardware unit.
 *
 * @details
 * The test uses a static DMAR table with two hardware units. Each hardware unit
 * has two device scopes. A device scope must not be longer than the rest of its
 * hardware unit.
 *
 * Test steps:
 * - Initialize the static DMAR table with the correct lengths.
 * - Set the length of the last device scope of the second hardware unit to its
 *   size plus 1.
 * - Call expect_assert() to tell the mocked assert handler to expect an
 *   assertion.
 * - Call acpi_dmar_foreach_devscope() with a callback that does nothing.
 *
 * Expected result:
 * - An assertion in acpi_dmar_foreach_devscope() fails. The mocked assert
 *   handler then marks the test as passed and stops it.
 * - The code after the call does not run.
 *
 * @testid{TSPEC-ARCHCOMMON-043}
 * @draft
 * @see acpi_dmar_foreach_devscope()
 */
ZTEST(lib_acpi, test_dmar_foreach_devscope_invalid_devscope_size_big)
{
	ACPI_DMAR_HARDWARE_UNIT *hu = &dmar0.unit1.header;
	ACPI_DMAR_DEVICE_SCOPE *devscope = &dmar0.unit1.ds1.header;

	dmar_initialize(&dmar0);

	/* Set invalid device scope size */
	devscope->Length = sizeof(dmar0.unit1.ds1) + 1;

	expect_assert();

	/* Expect assert, use fake void function as a callback */
	acpi_dmar_foreach_devscope(hu, devscope_nop, NULL);

	zassert_unreachable("Missed assert catch");
}

/**
 * Redefine AcpiGetTable to provide our static table
 */
DECLARE_FAKE_VALUE_FUNC(ACPI_STATUS, AcpiGetTable, char *, UINT32,
			struct acpi_table_header **);
static ACPI_STATUS dmar_custom_get_table(char *Signature, UINT32 Instance,
				  ACPI_TABLE_HEADER **OutTable)
{
	*OutTable = (ACPI_TABLE_HEADER *)&dmar0;

	return AE_OK;
}

/**
 * @brief Verify that acpi_dmar_ioapic_get() returns the PCI ID of the IOAPIC
 * device scope in the DMAR table.
 *
 * @details
 * The test uses a static DMAR table with two hardware units. Each hardware unit
 * has two device scopes. A fake AcpiGetTable() returns this table. The test
 * makes the last device scope an IOAPIC scope with a known bus, device and
 * function.
 *
 * Test steps:
 * - Initialize the static DMAR table with the correct lengths.
 * - Set the type of the last device scope to ACPI_DMAR_SCOPE_TYPE_IOAPIC.
 * - Set the bus to 0xab, the device to 0xc and the function to 0b101.
 * - Make the fake AcpiGetTable() return the static DMAR table.
 * - Call acpi_dmar_ioapic_get().
 *
 * Expected result:
 * - Before the call, the call count of AcpiGetTable() is 0.
 * - acpi_dmar_ioapic_get() returns 0.
 * - After the call, the call count of AcpiGetTable() is 1.
 * - The IOAPIC ID is equal to the raw value of the bus, device and function.
 *
 * @testid{TSPEC-ARCHCOMMON-044}
 * @draft
 * @see acpi_dmar_ioapic_get()
 */
ZTEST(lib_acpi, test_dmar_ioapic_get)
{
	union acpi_dmar_id fake_path = {
		.bits.bus = 0xab,
		.bits.device = 0xc,
		.bits.function = 0b101,
	};
	uint16_t ioapic;
	int ret;

	dmar_initialize(&dmar0);

	/* Set IOAPIC device scope */
	dmar0.unit1.ds1.header.EntryType = ACPI_DMAR_SCOPE_TYPE_IOAPIC;

	/* Set some arbitrary Bus and PCI path */
	dmar0.unit1.ds1.header.Bus = fake_path.bits.bus;
	dmar0.unit1.ds1.path0.Device = fake_path.bits.device;
	dmar0.unit1.ds1.path0.Function = fake_path.bits.function;

	/* Return our dmar0 table */
	AcpiGetTable_fake.custom_fake = dmar_custom_get_table;

	zassert_equal(AcpiGetTable_fake.call_count, 0);

	ret = acpi_dmar_ioapic_get(&ioapic);
	zassert_ok(ret, "Failed getting ioapic");

	/* Verify AcpiGetTable called */
	zassert_equal(AcpiGetTable_fake.call_count, 1);

	zassert_equal(ioapic, fake_path.raw, "Got wrong ioapic");

	TC_PRINT("Found ioapic id 0x%x\n", ioapic);
}

static void test_before(void *data)
{
	ASSERT_FFF_FAKES_LIST(RESET_FAKE);
}

ZTEST_SUITE(lib_acpi, NULL, NULL, test_before, NULL, NULL);
