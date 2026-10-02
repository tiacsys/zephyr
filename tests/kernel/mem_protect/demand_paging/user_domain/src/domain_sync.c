/*
 * Copyright (c) 2026 Process Mission
 *
 * Demand paging must preserve domain-private partition mappings: a
 * partition added to a memory domain with user permissions must remain
 * user-accessible after the kernel pages its backing frames out and
 * back in.
 *
 * Boot image pages are pinned at boot (kernel/mmu.c: z_mem_manage_init()),
 * so the partition is placed over a page-aligned static buffer that
 * k_mem_unpin() makes evictable.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/app_memory/mem_domain.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/kernel/mm/demand_paging.h>
#include <kernel_arch_interface.h>

#include "common.h"

static char part_page[CONFIG_MMU_PAGE_SIZE] __aligned(CONFIG_MMU_PAGE_SIZE);
K_MEM_PARTITION_DEFINE(test_part, part_page, sizeof(part_page), K_MEM_PARTITION_P_RW_U_RW);

static void domain_sync_before(void *data)
{
	struct k_mem_domain *domain = k_current_get()->mem_domain_info.mem_domain;

	ARG_UNUSED(data);

	zassert_not_null(domain, "test thread has no memory domain");

	/* write a marker so we can verify the contents survive the cycle */
	part_page[0] = 0x5a;

	/* hand the page to the test domain as a private U_RW partition */
	zassert_equal(k_mem_domain_add_partition(domain, &test_part), 0,
		      "k_mem_domain_add_partition failed");

	/* boot pages are pinned; make this one evictable */
	k_mem_unpin(part_page, CONFIG_MMU_PAGE_SIZE);

	/* simulate an eviction + refill cycle, as demand paging would do */
	zassert_equal(k_mem_page_out(part_page, CONFIG_MMU_PAGE_SIZE), 0, "k_mem_page_out failed");
	k_mem_page_in(part_page, CONFIG_MMU_PAGE_SIZE);
}

/**
 * @brief Verify that a user partition of a memory domain stays readable and
 * writable after a page-out and a page-in.
 *
 * @details
 * Demand paging must keep the mappings of the partitions that are private to a
 * memory domain. The boot code pins the pages of the boot image. Thus the
 * before function unpins the page of the partition first.
 *
 * Test steps:
 * - Write the marker 0x5a to the first byte of a page-aligned buffer.
 * - Add the buffer to the domain of the test thread as a
 *   K_MEM_PARTITION_P_RW_U_RW partition.
 * - Unpin the buffer with k_mem_unpin().
 * - Page out the buffer with k_mem_page_out(). Then page it in with
 *   k_mem_page_in().
 * - In user mode, read the first byte. Then write 0xa5 to the second byte and
 *   read it.
 *
 * Expected result:
 * - k_mem_domain_add_partition() and k_mem_page_out() return 0.
 * - The first byte is 0x5a.
 * - The second byte is 0xa5, and no fault occurs.
 *
 * @see k_mem_domain_add_partition(), k_mem_unpin(), k_mem_page_out(),
 * k_mem_page_in()
 */
ZTEST_USER(demand_paging_domain_sync, test_partition_perms_after_page_out_in)
{
	/* the private U_RW mapping must still be there */
	zassert_equal(part_page[0], 0x5a, "partition contents lost, got 0x%hhx", part_page[0]);

	part_page[1] = 0xa5;
	zassert_equal(part_page[1], (char)0xa5, "partition no longer writable");

	part_page[0] = 0;
	part_page[1] = 0;
}

ZTEST_SUITE(demand_paging_domain_sync, NULL, NULL, domain_sync_before, NULL, NULL);

static char ro_part_page[CONFIG_MMU_PAGE_SIZE] __aligned(CONFIG_MMU_PAGE_SIZE);
K_MEM_PARTITION_DEFINE(ro_test_part, ro_part_page, sizeof(ro_part_page),
		       K_MEM_PARTITION_P_RO_U_RO);
K_THREAD_STACK_DEFINE(ro_bad_stack, 2048);
static struct k_thread ro_bad_thread;
static volatile bool ro_write_succeeded;

static void ro_bad_entry(void *p1, void *p2, void *p3)
{
	volatile char *page = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	(void)page[0];
	page[1] = 0xa5;
	ro_write_succeeded = true;
}

static void read_only_before(void *data)
{
	struct k_mem_domain *domain = k_current_get()->mem_domain_info.mem_domain;

	ARG_UNUSED(data);

	ro_part_page[0] = 0x5a;
	zassert_equal(k_mem_domain_add_partition(domain, &ro_test_part), 0,
		      "k_mem_domain_add_partition failed");

	k_mem_unpin(ro_part_page, CONFIG_MMU_PAGE_SIZE);
	zassert_equal(k_mem_page_out(ro_part_page, CONFIG_MMU_PAGE_SIZE), 0,
		      "k_mem_page_out failed");
	k_mem_page_in(ro_part_page, CONFIG_MMU_PAGE_SIZE);

	k_thread_create(&ro_bad_thread, ro_bad_stack, K_THREAD_STACK_SIZEOF(ro_bad_stack),
			ro_bad_entry, ro_part_page, NULL, NULL, K_PRIO_PREEMPT(1), K_USER,
			K_FOREVER);
	zassert_equal(k_mem_domain_add_thread(domain, &ro_bad_thread), 0,
		      "k_mem_domain_add_thread failed");
}

/**
 * @brief Verify that a user write to a read-only partition is fatal after a
 * page-out and a page-in.
 *
 * @details
 * The page-in must not make a K_MEM_PARTITION_P_RO_U_RO partition writable. A
 * user thread in the same memory domain reads the page and then writes to it.
 * On arm64, the write must also not mark the page as dirty.
 *
 * Test steps:
 * - Add a K_MEM_PARTITION_P_RO_U_RO partition to the memory domain of the test
 *   thread.
 * - Unpin the page of the partition. Then page it out and page it in.
 * - Create a user thread and add it to the same memory domain.
 * - Tell the fatal error handler to expect a fault. Then start the user thread.
 * - Join the user thread with a timeout of 10 seconds.
 * - On arm64, get the page flags with arch_page_info_get().
 *
 * Expected result:
 * - k_mem_domain_add_partition(), k_mem_page_out() and
 *   k_mem_domain_add_thread() return 0.
 * - The write causes a fatal error. The kernel aborts the user thread, and
 *   k_thread_join() returns 0.
 * - The user thread does not complete the write.
 * - On arm64, ARCH_DATA_PAGE_DIRTY is not set for the page.
 *
 * @see k_mem_domain_add_partition(), k_mem_domain_add_thread(), k_mem_unpin(),
 * k_mem_page_out(), k_mem_page_in()
 */
ZTEST(demand_paging_read_only, test_user_write_to_read_only_partition_is_fatal)
{
	ro_write_succeeded = false;
	expect_fault = true;
	k_thread_start(&ro_bad_thread);
	zassert_ok(k_thread_join(&ro_bad_thread, K_SECONDS(10)),
		   "faulting thread was not aborted");

	expect_fault = false;
	zassert_false(ro_write_succeeded, "read-only partition accepted a user write");

#if defined(CONFIG_ARM64)
	zassert_false(arch_page_info_get(ro_part_page, NULL, false) & ARCH_DATA_PAGE_DIRTY,
		      "read-only partition write marked the page dirty");
#endif
}

ZTEST_SUITE(demand_paging_read_only, NULL, NULL, read_only_before, NULL, NULL);
