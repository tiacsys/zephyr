/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT altr_sysid

#include <zephyr.h>
#include <ztest.h>
#include <device.h>

#include "altera_avalon_sysid_regs.h"

#define SYSID_NODE		DT_NODELABEL(sysid)

BUILD_ASSERT(DT_NODE_HAS_COMPAT_STATUS(SYSID_NODE, altr_sysid, okay),
		"Configured system ID is not compatible with this sourcecode "
		"or disabled or has no node label sysid in the device tree.");

#undef SYSID_BASE
#undef SYSID_ID
#undef SYSID_TIMESTAMP

#define SYSID_BASE		DT_REG_ADDR(SYSID_NODE)
#define SYSID_ID		DT_PROP(SYSID_NODE, system_id)
#define SYSID_TIMESTAMP		DT_PROP(SYSID_NODE, system_ts)

static int32_t altera_avalon_sysid_test(void)
{
	int32_t hwid = IORD_ALTERA_AVALON_SYSID_ID(SYSID_BASE);
	int32_t hwts = IORD_ALTERA_AVALON_SYSID_TIMESTAMP(SYSID_BASE);

	if ((SYSID_TIMESTAMP == hwts) && (SYSID_ID == hwid))
		return 0;

	if (hwts - SYSID_TIMESTAMP > 0)
		return 1;

	else
		return -1;
}

void test_sysid(void)
{
	int32_t sysid, status = TC_FAIL;

	sysid = altera_avalon_sysid_test();
	if (!sysid) {
		status = TC_PASS;
		TC_PRINT("[SysID] hardware and software appear to be in sync\n");
	} else if (sysid > 0) {
		TC_PRINT("[SysID] software appears to be older than hardware\n");
	} else {
		TC_PRINT("[SysID] hardware appears to be older than software\n");
	}

	zassert_equal(status, TC_PASS, "SysID test failed");
}

void test_main(void)
{
	ztest_test_suite(nios2_sysid_test_suite,
			ztest_unit_test(test_sysid));
	ztest_run_test_suite(nios2_sysid_test_suite);
}
