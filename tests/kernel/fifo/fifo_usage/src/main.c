/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file
 * @brief FIFO usage-scenario tests
 */

/**
 * @defgroup kernel_fifo_usage_module Kernel FIFO Usage Test Module
 * @ingroup kernel_fifo_tests
 * @brief Test module at tests/kernel/fifo/fifo_usage -- thread and ISR usage scenarios.
 */

#include <zephyr/ztest.h>
#include <zephyr/irq_offload.h>

#define STACK_SIZE	(1024 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define LIST_LEN	4

struct fdata_t {
	sys_snode_t snode;
	uint32_t data;
};

static K_FIFO_DEFINE(fifo1);
static K_FIFO_DEFINE(fifo2);

/* Data to put into FIFO */
static struct fdata_t data1[LIST_LEN];
static struct fdata_t data2[LIST_LEN];
static struct fdata_t data_isr[LIST_LEN];

static K_THREAD_STACK_DEFINE(tstack, STACK_SIZE);
static struct k_thread tdata;
static struct k_sem end_sema;

/*entry of contexts*/
/**
 * @brief ISR entry: enqueue LIST_LEN items into a fifo and assert it is non-empty.
 *
 * @details
 * Puts @p data_isr[0..LIST_LEN-1] into @p p one by one via k_fifo_put(), then
 * asserts via zassert_false() that k_fifo_is_empty() returns false, confirming
 * that the insertions are visible from ISR context.
 *
 * @param p Pointer to the destination fifo, cast to @c const @c void*.
 *
 * @see k_fifo_put(), k_fifo_is_empty()
 *
 * @ingroup fifo_usage_procedures
 */
static void tIsr_entry_put(const void *p)
{
	uint32_t i;

	/* Put items into fifo */
	for (i = 0U; i < LIST_LEN; i++) {
		k_fifo_put((struct k_fifo *)p, (void *)&data_isr[i]);
	}
	zassert_false(k_fifo_is_empty((struct k_fifo *)p));
}

/**
 * @brief ISR entry: dequeue LIST_LEN items from a fifo and verify identity and
 * order, then assert the fifo is empty.
 *
 * @details
 * Gets @p data_isr[0..LIST_LEN-1] from @p p via k_fifo_get() with K_NO_WAIT,
 * asserting via zassert_equal() that each returned pointer matches the expected
 * source element in insertion order.  After draining, asserts via zassert_true()
 * that k_fifo_is_empty() returns true.
 *
 * @pre @p p must have been populated by @c tIsr_entry_put().
 *
 * @param p Pointer to the source fifo, cast to @c const @c void*.
 *
 * @see k_fifo_get(), k_fifo_is_empty()
 *
 * @ingroup fifo_usage_procedures
 */
static void tIsr_entry_get(const void *p)
{
	void *rx_data;
	uint32_t i;

	/* Get items from fifo */
	for (i = 0U; i < LIST_LEN; i++) {
		rx_data = k_fifo_get((struct k_fifo *)p, K_NO_WAIT);
		zassert_equal(rx_data, (void *)&data_isr[i]);
	}
	zassert_true(k_fifo_is_empty((struct k_fifo *)p));
}

/**
 * @brief Consumer thread: drain @p data1[] from a fifo, re-enqueue @p data2[],
 * then signal completion.
 *
 * @details
 * Gets @p data1[0..LIST_LEN-1] from @p p1 via k_fifo_get() with K_NO_WAIT,
 * asserting via zassert_equal() that each returned pointer matches the expected
 * @p data1[] element.  Then puts @p data2[0..LIST_LEN-1] back into @p p1 via
 * k_fifo_put() and signals @c end_sema so the test thread can continue.
 *
 * @pre @p p1 must have been populated with @p data1[] by the test thread.
 *
 * @param p1 Pointer to the fifo under test, cast to @c void*.
 * @param p2 Unused.
 * @param p3 Unused.
 *
 * @see k_fifo_get(), k_fifo_put()
 *
 * @ingroup fifo_usage_procedures
 */
static void thread_entry_fn_single(void *p1, void *p2, void *p3)
{
	void *rx_data;
	uint32_t i;

	/* Get items from fifo */
	for (i = 0U; i < LIST_LEN; i++) {
		rx_data = k_fifo_get((struct k_fifo *)p1, K_NO_WAIT);
		zassert_equal(rx_data, (void *)&data1[i]);
	}

	/* Put items into fifo */
	for (i = 0U; i < LIST_LEN; i++) {
		k_fifo_put((struct k_fifo *)p1, (void *)&data2[i]);
	}

	/* Give control back to Test thread */
	k_sem_give(&end_sema);
}

/**
 * @brief Relay thread: for each item the test thread puts into @p fifo2, get
 * it and forward a corresponding @p data1[] item to @p fifo1.
 *
 * @details
 * Loops LIST_LEN times.  In each iteration gets one item from @p p2 via
 * k_fifo_get() with K_FOREVER, asserts via zassert_equal() that the returned
 * pointer matches @p data2[i], then puts @p data1[i] into @p p1 via
 * k_fifo_put().  The alternating put-get pattern between the test thread and
 * this thread exercises two-fifo ping-pong data passing.
 *
 * @param p1 Pointer to @p fifo1 (the reply fifo), cast to @c void*.
 * @param p2 Pointer to @p fifo2 (the request fifo), cast to @c void*.
 * @param p3 Unused.
 *
 * @see k_fifo_get(), k_fifo_put()
 *
 * @ingroup fifo_usage_procedures
 */
static void thread_entry_fn_dual(void *p1, void *p2, void *p3)
{
	void *rx_data;
	uint32_t i;

	for (i = 0U; i < LIST_LEN; i++) {
		/* Get items from fifo2 */
		rx_data = k_fifo_get((struct k_fifo *)p2, K_FOREVER);
		zassert_equal(rx_data, (void *)&data2[i]);

		/* Put items into fifo1 */
		k_fifo_put((struct k_fifo *)p1, (void *)&data1[i]);
	}
}

/**
 * @brief ISR-context relay thread: drain @p fifo2 via ISR, fill @p fifo1 via
 * ISR, then signal completion.
 *
 * @details
 * Invokes @c tIsr_entry_get() on @p p2 via irq_offload() to drain @p fifo2
 * (asserting pointer identity and empty state from ISR context), then invokes
 * @c tIsr_entry_put() on @p p1 via irq_offload() to fill @p fifo1 (asserting
 * non-empty from ISR context).  Signals @c end_sema on exit.
 *
 * @pre @p p2 must have been populated with @p data_isr[] by the test thread
 *      via @c tIsr_entry_put() before this thread runs.
 *
 * @param p1 Pointer to @p fifo1 (to be filled), cast to @c void*.
 * @param p2 Pointer to @p fifo2 (to be drained), cast to @c void*.
 * @param p3 Unused.
 *
 * @see k_fifo_get(), k_fifo_put(), k_fifo_is_empty()
 *
 * @ingroup fifo_usage_procedures
 */
static void thread_entry_fn_isr(void *p1, void *p2, void *p3)
{
	/* Get items from fifo2 */
	irq_offload(tIsr_entry_get, (const void *)p2);

	/* Put items into fifo1 */
	irq_offload(tIsr_entry_put, (const void *)p1);

	/* Give control back to Test thread */
	k_sem_give(&end_sema);
}

/**
 * @brief A single fifo correctly relays items between two threads using put
 * and get.
 *
 * @details
 * The test thread enqueues @p data1[0..LIST_LEN-1] into @p fifo1 via
 * k_fifo_put(), then creates a consumer thread (@c thread_entry_fn_single) that
 * drains @p fifo1 (asserting pointer identity) and re-enqueues @p data2[].
 * After waiting for the consumer to signal @c end_sema, the test thread drains
 * @p fifo1 again, asserting that each received pointer matches @p data2[i].
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-24-3`
 * - :external+req:ref:`zep-srs-24-7`
 * @endverbatim
 *
 * @see k_fifo_put(), k_fifo_get()
 * @draft
 */
ZTEST(fifo_usage, test_single_fifo_play)
{
	void *rx_data;
	uint32_t i;

	/** @par Arrange
	 * -# Initialise @c end_sema to 0 via k_sem_init().
	 * -# Enqueue @p data1[0..LIST_LEN-1] into @p fifo1 via k_fifo_put().
	 * -# Create consumer thread (@c thread_entry_fn_single) at preemptive
	 *    priority 0 with K_INHERIT_PERMS.
	 */
	/* Init kernel objects */
	k_sem_init(&end_sema, 0, 1);

	/* Put items into fifo */
	for (i = 0U; i < LIST_LEN; i++) {
		k_fifo_put(&fifo1, (void *)&data1[i]);
	}

	k_tid_t tid = k_thread_create(&tdata, tstack, STACK_SIZE,
				thread_entry_fn_single, &fifo1, NULL, NULL,
				K_PRIO_PREEMPT(0), K_INHERIT_PERMS, K_NO_WAIT);

	/** @par Act
	 * -# Wait for @c end_sema (consumer drains @p data1[], re-enqueues
	 *    @p data2[], then signals).
	 * -# Drain @p fifo1 via k_fifo_get() with K_NO_WAIT.
	 */

	/** @par Assert
	 * -# @c thread_entry_fn_single completes without assertion failure:
	 *    each @p data1[i] pointer it received matched the expected element.
	 * -# Each pointer returned by the test thread's drain equals @p data2[i],
	 *    confirming that the consumer's re-enqueue preserved identity and order.
	 */
	/* Let the child thread run */
	k_sem_take(&end_sema, K_FOREVER);

	/* Get items from fifo */
	for (i = 0U; i < LIST_LEN; i++) {
		rx_data = k_fifo_get(&fifo1, K_NO_WAIT);
		zassert_equal(rx_data, (void *)&data2[i]);
	}

	/** @par Teardown
	 * -# Abort the consumer thread.
	 */
	/* Clear the spawn thread to avoid side effect */
	k_thread_abort(tid);
}

/**
 * @brief Two fifos correctly relay items between two threads in a ping-pong
 * pattern.
 *
 * @details
 * A relay thread (@c thread_entry_fn_dual) blocks on @p fifo2 waiting for each
 * item in turn.  The test thread drives LIST_LEN rounds: each round puts
 * @p data2[i] into @p fifo2 and then gets from @p fifo1 with K_FOREVER.  The
 * relay thread gets @p data2[i] from @p fifo2 (asserting identity) and puts
 * @p data1[i] into @p fifo1.  The test thread asserts via zassert_equal() that
 * each item received from @p fifo1 equals @p data1[i], confirming two-fifo
 * alternating data passing.
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-24-3`
 * - :external+req:ref:`zep-srs-24-7`
 * @endverbatim
 *
 * @see k_fifo_put(), k_fifo_get()
 * @draft
 */
ZTEST(fifo_usage, test_dual_fifo_play)
{
	void *rx_data;
	uint32_t i;

	/** @par Arrange
	 * -# Create the relay thread (@c thread_entry_fn_dual) at preemptive
	 *    priority 0 with K_INHERIT_PERMS; it blocks on @p fifo2 immediately.
	 */
	k_tid_t tid = k_thread_create(&tdata, tstack, STACK_SIZE,
				thread_entry_fn_dual, &fifo1, &fifo2, NULL,
				K_PRIO_PREEMPT(0), K_INHERIT_PERMS, K_NO_WAIT);

	/** @par Act
	 * -# For each i in [0, LIST_LEN): put @p data2[i] into @p fifo2, then
	 *    get from @p fifo1 with K_FOREVER.
	 */

	/** @par Assert
	 * -# @c thread_entry_fn_dual completes without assertion failure: each
	 *    @p data2[i] it received matched the expected element.
	 * -# Each pointer the test thread receives from @p fifo1 equals
	 *    @p data1[i], confirming the relay forwarded items correctly.
	 */
	for (i = 0U; i < LIST_LEN; i++) {
		/* Put item into fifo */
		k_fifo_put(&fifo2, (void *)&data2[i]);

		/* Get item from fifo */
		rx_data = k_fifo_get(&fifo1, K_FOREVER);
		zassert_equal(rx_data, (void *)&data1[i]);
	}

	/** @par Teardown
	 * -# Abort the relay thread.
	 */
	/* Clear the spawn thread to avoid side effect */
	k_thread_abort(tid);

}

/**
 * @brief k_fifo_put() and k_fifo_get() work correctly when called exclusively
 * from ISR context across two threads.
 *
 * @details
 * All enqueue and dequeue operations execute inside irq_offload() handlers.
 * The test thread ISR-puts @p data_isr[] into @p fifo2, then yields to a child
 * thread (@c thread_entry_fn_isr) which ISR-drains @p fifo2 (asserting pointer
 * identity and empty state) and ISR-fills @p fifo1 (asserting non-empty).  After
 * the child signals @c end_sema, the test thread ISR-drains @p fifo1, asserting
 * pointer identity and empty state from ISR context.
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-24-3`
 * - :external+req:ref:`zep-srs-24-7`
 * - :external+req:ref:`zep-srs-24-8`
 * @endverbatim
 *
 * @see k_fifo_put(), k_fifo_get(), k_fifo_is_empty()
 * @draft
 */
ZTEST(fifo_usage, test_isr_fifo_play)
{
	/** @par Arrange
	 * -# Initialise @c end_sema to 0 via k_sem_init().
	 * -# Create the ISR relay thread (@c thread_entry_fn_isr) at preemptive
	 *    priority 0 with K_INHERIT_PERMS.
	 * -# ISR-put @p data_isr[] into @p fifo2 via irq_offload(@c tIsr_entry_put).
	 */
	/* Init kernel objects */
	k_sem_init(&end_sema, 0, 1);

	k_tid_t tid = k_thread_create(&tdata, tstack, STACK_SIZE,
				thread_entry_fn_isr, &fifo1, &fifo2, NULL,
				K_PRIO_PREEMPT(0), K_INHERIT_PERMS, K_NO_WAIT);

	/* Put item into fifo */
	irq_offload(tIsr_entry_put, (const void *)&fifo2);

	/** @par Act
	 * -# Wait for @c end_sema (child drains @p fifo2 via ISR and fills
	 *    @p fifo1 via ISR, then signals).
	 * -# ISR-drain @p fifo1 via irq_offload(@c tIsr_entry_get).
	 */

	/** @par Assert
	 * -# @c thread_entry_fn_isr completes without assertion failure:
	 *    @c tIsr_entry_get verified @p data_isr[] identity and empty state
	 *    on @p fifo2; @c tIsr_entry_put asserted @p fifo1 is non-empty.
	 * -# The test thread's @c tIsr_entry_get on @p fifo1 asserts pointer
	 *    identity for all @p data_isr[] elements and that @p fifo1 is
	 *    empty after the drain.
	 */
	/* Let the child thread run */
	k_sem_take(&end_sema, K_FOREVER);

	/* Get item from fifo */
	irq_offload(tIsr_entry_get, (const void *)&fifo1);

	/** @par Teardown
	 * -# Abort the relay thread.
	 */
	/* Clear the spawn thread to avoid side effect */
	k_thread_abort(tid);
}

/**
 * @defgroup fifo_usage FIFO Usage ZTest suite
 * @ingroup kernel_fifo_usage_module
 * @brief k_fifo put/get scenario tests requiring single-CPU pinning -- single-fifo
 *        thread exchange, two-fifo ping-pong, and all-ISR-context data passing.
 *
 * @details
 * Contains tests pinned to a single CPU via ztest_simple_1cpu_before/after.
 * Covers a single-fifo round-trip where the test thread enqueues data, a child
 * thread drains and re-enqueues, and the test thread drains again; a two-fifo
 * ping-pong pattern where the test thread and a relay thread alternate puts and
 * gets across @p fifo1 and @p fifo2; and an all-ISR scenario where all put/get
 * operations on both fifos are performed inside irq_offload() handlers.
 */
ZTEST_SUITE(fifo_usage, NULL, NULL,
		ztest_simple_1cpu_before, ztest_simple_1cpu_after, NULL);
