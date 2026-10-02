/*
 * Copyright (c) 2019 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/internal/syscall_handler.h>

static volatile int test_flag;

__ramfunc static void ram_function(void)
{
	test_flag = 1;
}

/**
 * @brief Verify that a __ramfunc function is in the .ramfunc section in RAM,
 * and that it runs from there.
 *
 * @details
 * The __ramfunc attribute puts ram_function() into the .ramfunc linker section.
 * ram_function() sets test_flag to 1. If CONFIG_USERSPACE is enabled, user mode
 * must also have read access to the section.
 *
 * Test steps:
 * - Read test_flag.
 * - Compare __ramfunc_start, __ramfunc_end and __ramfunc_size with the limits
 *   of the RAM image.
 * - Compare the address of ram_function() with the limits of the .ramfunc
 *   section.
 * - If CONFIG_USERSPACE is enabled, call arch_buffer_validate() on the .ramfunc
 *   section for read access.
 * - Call ram_function(). Then read test_flag again.
 *
 * Expected result:
 * - test_flag is 0 before the call.
 * - The .ramfunc section is not empty, and it is inside the RAM image.
 * - ram_function() is inside the .ramfunc section.
 * - If CONFIG_USERSPACE is enabled, arch_buffer_validate() returns 0.
 * - test_flag is 1 after the call.
 *
 * @see __ramfunc
 */
ZTEST(ramfunc, test_ramfunc)
{
	int init_flag, post_flag;

	init_flag = test_flag;
	zassert_true(init_flag == 0, "Test flag not initialized to zero");

	/* Verify that the .ramfunc section is not empty, it is located
	 * inside SRAM, and that ram_function(.) is located inside
	 * the .ramfunc section.
	 */
	zassert_true((uint32_t)&__ramfunc_size != 0,
		".ramfunc linker section is empty");
	zassert_true(((uint32_t)&__ramfunc_start >= (uint32_t)&_image_ram_start)
			&& ((uint32_t)&__ramfunc_end < (uint32_t)&_image_ram_end),
			".ramfunc linker section not in RAM");
	zassert_true(
		(((uint32_t)&__ramfunc_start) <= (uint32_t)ram_function) &&
		(((uint32_t)&__ramfunc_end) > (uint32_t)ram_function),
		"ram_function not loaded into .ramfunc");

	/* If we build with User Mode support, verify that the
	 * ram_function(.) is user (read) accessible.
	 */
#if defined(CONFIG_USERSPACE)
	zassert_true(arch_buffer_validate((void *)&__ramfunc_start,
			(size_t)&__ramfunc_size, 0) == 0 /* Success */,
		".ramfunc section not user accessible");
#endif /* CONFIG_USERSPACE */

	/* Execute the function from SRAM. */
	ram_function();

	/* Verify that the function is executed successfully. */
	post_flag = test_flag;
	zassert_true(post_flag == 1,
		"ram_function() execution failed.");
}
