/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Timeout-behaviour FIFO tests
 */

/**
 * @defgroup kernel_fifo_timeout_module Kernel FIFO Timeout Test Module
 * @ingroup kernel_fifo_tests
 * @brief Test module at tests/kernel/fifo/fifo_timeout -- k_fifo_get() timeout tests.
 */

/**
 * @defgroup fifo_timeout_procedures Shared Test Procedures
 * @ingroup fifo_timeout
 * @brief Reusable helper procedures invoked by fifo_timeout test cases.
 */

/**
 * @defgroup fifo_timeout_1cpu_procedures Shared Test Procedures
 * @ingroup fifo_timeout_1cpu
 * @brief Reusable helper procedures invoked by fifo_timeout_1cpu test cases.
 */

#include <zephyr/ztest.h>
#include <zephyr/irq_offload.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(test);

struct scratch_fifo_packet {
	void *link_in_fifo;
	void *data_if_needed;
};

struct reply_packet {
	void *link_in_fifo;
	int32_t reply;
};

struct timeout_order_data {
	void *link_in_fifo;
	struct k_fifo *fifo;
	uint32_t timeout;
	int32_t timeout_order;
	int32_t q_order;
};


#define NUM_SCRATCH_FIFO_PACKETS 20
struct scratch_fifo_packet scratch_fifo_packets[NUM_SCRATCH_FIFO_PACKETS];

struct k_fifo scratch_fifo_packets_fifo;

static struct k_fifo fifo_timeout[2];
struct k_fifo timeout_order_fifo;

struct timeout_order_data timeout_order_data[] = {
	{0, &fifo_timeout[0], 20, 2, 0},
	{0, &fifo_timeout[0], 40, 4, 1},
	{0, &fifo_timeout[0], 0, 0, 2},
	{0, &fifo_timeout[0], 10, 1, 3},
	{0, &fifo_timeout[0], 30, 3, 4},
};

struct timeout_order_data timeout_order_data_mult_fifo[] = {
	{0, &fifo_timeout[1], 0, 0, 0},
	{0, &fifo_timeout[0], 30, 3, 1},
	{0, &fifo_timeout[0], 50, 5, 2},
	{0, &fifo_timeout[1], 80, 8, 3},
	{0, &fifo_timeout[1], 70, 7, 4},
	{0, &fifo_timeout[0], 10, 1, 5},
	{0, &fifo_timeout[0], 60, 6, 6},
	{0, &fifo_timeout[0], 20, 2, 7},
	{0, &fifo_timeout[1], 40, 4, 8},
};

#define TIMEOUT_ORDER_NUM_THREADS	ARRAY_SIZE(timeout_order_data_mult_fifo)
#define TSTACK_SIZE			(512 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define FIFO_THREAD_PRIO		-5

static K_THREAD_STACK_ARRAY_DEFINE(ttstack,
		TIMEOUT_ORDER_NUM_THREADS, TSTACK_SIZE);
static struct k_thread ttdata[TIMEOUT_ORDER_NUM_THREADS];
static k_tid_t tid[TIMEOUT_ORDER_NUM_THREADS];

/**
 * @brief Allocate one scratch packet from the pool and assert it is non-null.
 *
 * @details
 * Calls k_fifo_get() with K_NO_WAIT on @c scratch_fifo_packets_fifo and
 * asserts via zassert_true() that the returned pointer is non-null, ensuring
 * that the scratch pool has not been exhausted before the test begins.
 *
 * @return Pointer to the allocated @c scratch_fifo_packet.
 *
 * @see k_fifo_get()
 *
 * @ingroup fifo_timeout_procedures
 * @ingroup fifo_timeout_1cpu_procedures
 */
static void *get_scratch_packet(void)
{
	void *packet = k_fifo_get(&scratch_fifo_packets_fifo, K_NO_WAIT);

	zassert_true(packet != NULL);
	return packet;
}

/** @cond INTERNAL */
static void put_scratch_packet(void *packet)
{
	k_fifo_put(&scratch_fifo_packets_fifo, packet);
}

static bool is_timeout_in_range(uint32_t start_time, uint32_t timeout)
{
	uint32_t stop_time, diff;

	stop_time = k_cycle_get_32();
	diff = (uint32_t)k_cyc_to_ns_floor64(stop_time -
			start_time) / NSEC_PER_USEC;
	diff = diff / USEC_PER_MSEC;
	return timeout <= diff;
}

/* a thread sleeps then puts data on the fifo */
static void test_thread_put_timeout(void *p1, void *p2, void *p3)
{
	uint32_t timeout = *((uint32_t *)p2);

	k_msleep(timeout);
	k_fifo_put((struct k_fifo *)p1, get_scratch_packet());
}
/** @endcond */

/**
 * @brief Block on a fifo with a finite timeout and assert that it expires
 * without delivering data.
 *
 * @details
 * Aligns to the next tick via k_msleep(1), records the start cycle count,
 * then calls k_fifo_get() with K_MSEC(d->timeout) on d->fifo.  Asserts via
 * zassert_true() that the returned packet is NULL (timeout expired without
 * data) and that the elapsed time is at least d->timeout milliseconds.
 * Posts @p d to @c timeout_order_fifo so the caller can collect completions
 * in arrival order.
 *
 * @param p1 Pointer to a @c timeout_order_data entry describing the fifo and
 *           timeout for this thread, cast to @c void*.
 * @param p2 Unused.
 * @param p3 Unused.
 *
 * @see k_fifo_get()
 *
 * @ingroup fifo_timeout_1cpu_procedures
 */
/* a thread pends on a fifo then times out */
static void test_thread_pend_and_timeout(void *p1, void *p2, void *p3)
{
	struct timeout_order_data *d = (struct timeout_order_data *)p1;
	uint32_t start_time;
	void *packet;

	k_msleep(1); /* Align to ticks */

	start_time = k_cycle_get_32();
	packet = k_fifo_get(d->fifo, K_MSEC(d->timeout));
	zassert_true(packet == NULL);
	zassert_true(is_timeout_in_range(start_time, d->timeout));

	k_fifo_put(&timeout_order_fifo, d);
}
/**
 * @brief Spawn multiple threads that each pend on a fifo with a distinct
 * timeout and verify they complete in timeout-ascending order.
 *
 * @details
 * Creates @p test_data_size threads, each running @c test_thread_pend_and_timeout
 * with its own fifo and timeout from @p test_data.  Collects their completions
 * via @c timeout_order_fifo in arrival order and verifies that each thread's
 * @c timeout_order field matches its expected position.  A one-tick tolerance
 * is allowed to accommodate tick-boundary effects.  Returns TC_PASS if the
 * order is correct, TC_FAIL otherwise.
 *
 * @param test_data     Array of @c timeout_order_data entries, one per thread.
 * @param test_data_size Number of entries in @p test_data.
 *
 * @return TC_PASS if all threads timed out in the expected order; TC_FAIL otherwise.
 *
 * @see k_fifo_get()
 *
 * @ingroup fifo_timeout_1cpu_procedures
 */
/* Spins several threads that pend and timeout on fifos */
static int test_multiple_threads_pending(struct timeout_order_data *test_data,
					 int test_data_size)
{
	int ii, j;
	uint32_t diff_ms;

	for (ii = 0; ii < test_data_size; ii++) {
		tid[ii] = k_thread_create(&ttdata[ii], ttstack[ii], TSTACK_SIZE,
				test_thread_pend_and_timeout,
				&test_data[ii], NULL, NULL,
				FIFO_THREAD_PRIO, K_INHERIT_PERMS, K_NO_WAIT);
	}

	/* In general, there is no guarantee of wakeup order when multiple
	 * threads are woken up on the same tick. This can especially happen
	 * when the system is loaded. However, in this particular test, we
	 * are controlling the system state and hence we can make a reasonable
	 * estimation of a timeout occurring with the max deviation of an
	 * additional tick. Hence the timeout order may slightly be different
	 * from what we normally expect.
	 */
	for (ii = 0; ii < test_data_size; ii++) {
		struct timeout_order_data *data =
			k_fifo_get(&timeout_order_fifo, K_FOREVER);

		zassert_not_null(data, NULL);
		if (data->timeout_order == ii) {
			LOG_DBG(" thread (q order: %d, t/o: %d, fifo %p)",
				data->q_order, data->timeout, data->fifo);
		} else {
			/* Get the index of the thread which should have
			 * actually timed out.
			 */
			for (j = 0; j < test_data_size; j++) {
				if (test_data[j].timeout_order == ii) {
					break;
				}
			}

			if (data->timeout > test_data[j].timeout) {
				diff_ms = data->timeout - test_data[j].timeout;
			} else {
				diff_ms = test_data[j].timeout - data->timeout;
			}

			if (k_ms_to_ticks_ceil32(diff_ms) == 1) {
				LOG_DBG(
				" thread (q order: %d, t/o: %d, fifo %p)",
				data->q_order, data->timeout, data->fifo);
			} else {
				TC_ERROR(
				" *** thread %d woke up, expected %d\n",
				data->q_order, j);
				return TC_FAIL;
			}
		}
	}

	return TC_PASS;
}

/**
 * @brief Block on a fifo with a finite timeout and assert that data arrives
 * before the timeout expires.
 *
 * @details
 * Calls k_fifo_get() with K_MSEC(d->timeout) on d->fifo and asserts via
 * zassert_true() that the returned packet is non-null, confirming that a
 * scratch packet was delivered in time.  Returns the packet to the scratch
 * pool and posts @p d to @c timeout_order_fifo so the caller can track
 * completion order.
 *
 * @param p1 Pointer to a @c timeout_order_data entry describing the fifo and
 *           timeout for this thread, cast to @c void*.
 * @param p2 Unused.
 * @param p3 Unused.
 *
 * @see k_fifo_get()
 *
 * @ingroup fifo_timeout_1cpu_procedures
 */
/* a thread pends on a fifo with a timeout and gets the data in time */
static void test_thread_pend_and_get_data(void *p1, void *p2, void *p3)
{
	struct timeout_order_data *d = (struct timeout_order_data *)p1;
	void *packet;

	packet = k_fifo_get(d->fifo, K_MSEC(d->timeout));
	zassert_true(packet != NULL);

	put_scratch_packet(packet);
	k_fifo_put(&timeout_order_fifo, d);
}

/**
 * @brief Spawn multiple threads that receive data in time, except the last
 * which times out, and verify each completes in queue order.
 *
 * @details
 * Creates @p test_data_size-1 threads running @c test_thread_pend_and_get_data
 * and one final thread running @c test_thread_pend_and_timeout.  Feeds scratch
 * packets one by one to the first N-1 threads in queue order and verifies via
 * @c timeout_order_fifo that each wakes in the expected sequence.  The last
 * thread receives no data and must time out.  Returns TC_PASS if all threads
 * complete in the expected order, TC_FAIL otherwise.
 *
 * @param test_data      Array of @c timeout_order_data entries, one per thread.
 * @param test_data_size Number of entries in @p test_data.
 *
 * @return TC_PASS if completion order is correct; TC_FAIL otherwise.
 *
 * @see k_fifo_get(), k_fifo_put()
 *
 * @ingroup fifo_timeout_1cpu_procedures
 */
/* Spins child threads that get fifo data in time, except the last one */
static int test_multiple_threads_get_data(struct timeout_order_data *test_data,
					 int test_data_size)
{
	struct timeout_order_data *data;
	int ii;

	for (ii = 0; ii < test_data_size-1; ii++) {
		tid[ii] = k_thread_create(&ttdata[ii], ttstack[ii], TSTACK_SIZE,
				test_thread_pend_and_get_data,
				&test_data[ii], NULL, NULL,
				K_PRIO_PREEMPT(0), K_INHERIT_PERMS, K_NO_WAIT);
	}

	tid[ii] = k_thread_create(&ttdata[ii], ttstack[ii], TSTACK_SIZE,
				test_thread_pend_and_timeout,
				&test_data[ii], NULL, NULL,
				K_PRIO_PREEMPT(0), K_INHERIT_PERMS, K_NO_WAIT);

	for (ii = 0; ii < test_data_size-1; ii++) {
		k_fifo_put(test_data[ii].fifo, get_scratch_packet());

		data = k_fifo_get(&timeout_order_fifo, K_FOREVER);
		if (!data) {
			TC_ERROR("thread %d got NULL value from fifo\n", ii);
			return TC_FAIL;
		}

		if (data->q_order != ii) {
			TC_ERROR(" *** thread %d woke up, expected %d\n",
				 data->q_order, ii);
			return TC_FAIL;
		}

		if (data->q_order == ii) {
			LOG_DBG(" thread (q order: %d, t/o: %d, fifo %p)",
				data->q_order, data->timeout, data->fifo);
		}
	}

	data = k_fifo_get(&timeout_order_fifo, K_FOREVER);
	if (!data) {
		TC_ERROR("thread %d got NULL value from fifo\n", ii);
		return TC_FAIL;
	}

	if (data->q_order != ii) {
		TC_ERROR(" *** thread %d woke up, expected %d\n",
			 data->q_order, ii);
		return TC_FAIL;
	}

	LOG_DBG(" thread (q order: %d, t/o: %d, fifo %p)",
		data->q_order, data->timeout, data->fifo);

	return TC_PASS;
}

/** @cond INTERNAL */
/* try getting data on fifo with special timeout value, return result in fifo */
static void test_thread_timeout_reply_values(void *p1, void *p2, void *p3)
{
	struct reply_packet *reply_packet = (struct reply_packet *)p1;

	reply_packet->reply =
		!!k_fifo_get(&fifo_timeout[0], K_NO_WAIT);

	k_fifo_put(&timeout_order_fifo, reply_packet);
}

static void test_thread_timeout_reply_values_wfe(void *p1, void *p2, void *p3)
{
	struct reply_packet *reply_packet = (struct reply_packet *)p1;

	reply_packet->reply =
		!!k_fifo_get(&fifo_timeout[0], K_FOREVER);

	k_fifo_put(&timeout_order_fifo, reply_packet);
}
/** @endcond */

/**
 * @brief k_fifo_get() on an empty fifo returns NULL after the timeout
 * elapses and immediately with K_NO_WAIT.
 *
 * @details
 * Two sub-scenarios are verified on the pre-initialised empty @c fifo_timeout[0]:
 * -# With a 10 ms finite timeout: the call returns NULL and the measured
 *    elapsed time is at least 10 ms, confirming the timeout was observed.
 * -# With K_NO_WAIT: the call returns NULL immediately without blocking.
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-24-7`
 * @endverbatim
 *
 * @see k_fifo_get()
 * @draft
 */
ZTEST(fifo_timeout_1cpu, test_timeout_empty_fifo)
{
	void *packet;
	uint32_t start_time, timeout;

	/** @par Arrange
	 * -# Sleep 1 ms to align to a tick boundary.
	 * -# @c fifo_timeout[0] is empty (initialised in the suite setup).
	 */
	k_msleep(1); /* Align to ticks */

	/** @par Act -- finite timeout on empty fifo
	 * -# Record the start cycle count.
	 * -# Call k_fifo_get() on @c fifo_timeout[0] with a 10 ms timeout.
	 */
	/** @par Assert
	 * -# Return value is NULL (no data arrived).
	 * -# Elapsed time is at least 10 ms, confirming the timeout was observed.
	 */
	/* Test empty fifo with timeout */
	timeout = 10U;
	start_time = k_cycle_get_32();
	packet = k_fifo_get(&fifo_timeout[0], K_MSEC(timeout));
	zassert_true(packet == NULL);
	zassert_true(is_timeout_in_range(start_time, timeout));

	/** @par Act -- K_NO_WAIT on empty fifo
	 * -# Call k_fifo_get() on @c fifo_timeout[0] with K_NO_WAIT.
	 */
	/** @par Assert
	 * -# Return value is NULL (fifo is still empty, no blocking occurred).
	 */
	/* Test empty fifo with timeout of K_NO_WAIT */
	packet = k_fifo_get(&fifo_timeout[0], K_NO_WAIT);
	zassert_true(packet == NULL);
}

/**
 * @brief k_fifo_get() successfully retrieves an available item with both
 * K_NO_WAIT and K_FOREVER.
 *
 * @details
 * Two sub-scenarios are verified on @c fifo_timeout[0] after an item is
 * enqueued:
 * -# K_NO_WAIT: item is put then immediately retrieved without blocking.
 * -# K_FOREVER: item is put then retrieved with an indefinite wait.
 * Both sub-scenarios confirm that k_fifo_get() returns a non-null pointer
 * when data is present regardless of the timeout value.
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-24-3`
 * - :external+req:ref:`zep-srs-24-7`
 * @endverbatim
 *
 * @see k_fifo_get(), k_fifo_put()
 * @draft
 */
ZTEST(fifo_timeout, test_timeout_non_empty_fifo)
{
	void *packet, *scratch_packet;

	/** @par Arrange -- K_NO_WAIT sub-scenario
	 * -# Allocate a scratch packet via @c get_scratch_packet().
	 * -# Put the packet into @c fifo_timeout[0] via k_fifo_put().
	 */
	/** @par Act
	 * -# Call k_fifo_get() on @c fifo_timeout[0] with K_NO_WAIT.
	 */
	/** @par Assert
	 * -# Return value is non-null, confirming the item was retrieved.
	 */
	 /* Test k_fifo_get with K_NO_WAIT */
	scratch_packet = get_scratch_packet();
	k_fifo_put(&fifo_timeout[0], scratch_packet);
	packet = k_fifo_get(&fifo_timeout[0], K_NO_WAIT);
	zassert_true(packet != NULL);
	put_scratch_packet(scratch_packet);

	/** @par Arrange -- K_FOREVER sub-scenario
	 * -# Allocate a scratch packet and put it into @c fifo_timeout[0].
	 */
	/** @par Act
	 * -# Call k_fifo_get() on @c fifo_timeout[0] with K_FOREVER.
	 */
	/** @par Assert
	 * -# Return value is non-null, confirming the item was retrieved.
	 */
	 /* Test k_fifo_get with K_FOREVER */
	scratch_packet = get_scratch_packet();
	k_fifo_put(&fifo_timeout[0], scratch_packet);
	packet = k_fifo_get(&fifo_timeout[0], K_FOREVER);
	zassert_true(packet != NULL);
	put_scratch_packet(scratch_packet);
}

/**
 * @brief k_fifo_get() correctly handles a finite timeout, K_NO_WAIT, and
 * K_FOREVER when data availability is controlled by a child thread.
 *
 * @details
 * Three sub-scenarios exercise k_fifo_get() on @c fifo_timeout[0]:
 * -# **Data arrives in time**: a child thread sleeps 10 ms then enqueues a
 *    scratch packet; the main thread calls k_fifo_get() with a 20 ms timeout
 *    and receives the packet, asserting non-null and timing validity.
 * -# **K_NO_WAIT, fifo empty**: a child thread calls k_fifo_get() with
 *    K_NO_WAIT on an empty fifo and reports the result (false) via a reply
 *    packet posted to @c timeout_order_fifo.
 * -# **K_NO_WAIT, fifo non-empty**: same child pattern but the fifo is
 *    pre-loaded; the child reports true.  Then a fourth sub-case tests
 *    K_FOREVER on a pre-loaded fifo via @c test_thread_timeout_reply_values_wfe,
 *    which also reports true.
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-24-3`
 * - :external+req:ref:`zep-srs-24-7`
 * @endverbatim
 *
 * @see k_fifo_get(), k_fifo_put()
 * @draft
 */
ZTEST(fifo_timeout_1cpu, test_timeout_fifo_thread)
{
	void *packet, *scratch_packet;
	struct reply_packet reply_packet;
	uint32_t start_time, timeout;

	/** @par Arrange
	 * -# Sleep 1 ms to align to a tick boundary.
	 */
	k_msleep(1); /* Align to ticks */

	/** @par Act -- data arrives in time
	 * -# Spawn a child thread that sleeps 10 ms then enqueues a scratch
	 *    packet via @c test_thread_put_timeout.
	 * -# Call k_fifo_get() with a 20 ms timeout while recording the start time.
	 */
	/** @par Assert
	 * -# Return value is non-null (packet arrived before timeout).
	 * -# Elapsed time is at least 10 ms (data arrived after the sleep).
	 */
	/*
	 * Test fifo with some timeout and child thread that puts
	 * data on the fifo on time
	 */
	timeout = 10U;
	start_time = k_cycle_get_32();

	tid[0] = k_thread_create(&ttdata[0], ttstack[0], TSTACK_SIZE,
				test_thread_put_timeout, &fifo_timeout[0],
				&timeout, NULL,
				FIFO_THREAD_PRIO, K_INHERIT_PERMS, K_NO_WAIT);

	packet = k_fifo_get(&fifo_timeout[0], K_MSEC(timeout + 10));
	zassert_true(packet != NULL);
	zassert_true(is_timeout_in_range(start_time, timeout));
	put_scratch_packet(packet);

	/** @par Act -- K_NO_WAIT, fifo empty
	 * -# Spawn @c test_thread_timeout_reply_values which calls k_fifo_get()
	 *    with K_NO_WAIT on the empty @c fifo_timeout[0] and posts the result.
	 * -# Yield to let the child run.
	 */
	/** @par Assert
	 * -# Reply packet arrives on @c timeout_order_fifo (non-null).
	 * -# @c reply_packet.reply is false: K_NO_WAIT on empty fifo returned NULL.
	 */
	/*
	 * Test k_fifo_get with timeout of K_NO_WAIT and the fifo
	 * should be filled be filled by the child thread based on
	 * the data availability on another fifo. In this test child
	 * thread does not find data on fifo.
	 */
	tid[0] = k_thread_create(&ttdata[0], ttstack[0], TSTACK_SIZE,
				test_thread_timeout_reply_values,
				&reply_packet, NULL, NULL,
				FIFO_THREAD_PRIO, K_INHERIT_PERMS, K_NO_WAIT);

	k_yield();
	packet = k_fifo_get(&timeout_order_fifo, K_NO_WAIT);
	zassert_true(packet != NULL);
	zassert_false(reply_packet.reply);

	/** @par Act -- K_NO_WAIT, fifo non-empty
	 * -# Pre-load a scratch packet into @c fifo_timeout[0].
	 * -# Spawn @c test_thread_timeout_reply_values again; the child finds
	 *    data and reports true.
	 * -# Yield to let the child run.
	 */
	/** @par Assert
	 * -# Reply packet arrives on @c timeout_order_fifo.
	 * -# @c reply_packet.reply is true: K_NO_WAIT found data and returned it.
	 */
	/*
	 * Test k_fifo_get with timeout of K_NO_WAIT and the fifo
	 * should be filled be filled by the child thread based on
	 * the data availability on another fifo. In this test child
	 * thread does find data on fifo.
	 */
	scratch_packet = get_scratch_packet();
	k_fifo_put(&fifo_timeout[0], scratch_packet);

	tid[0] = k_thread_create(&ttdata[0], ttstack[0], TSTACK_SIZE,
				test_thread_timeout_reply_values,
				&reply_packet, NULL, NULL,
				FIFO_THREAD_PRIO, K_INHERIT_PERMS, K_NO_WAIT);

	k_yield();
	packet = k_fifo_get(&timeout_order_fifo, K_NO_WAIT);
	zassert_true(packet != NULL);
	zassert_true(reply_packet.reply);
	put_scratch_packet(scratch_packet);

	/** @par Act -- K_FOREVER, fifo non-empty
	 * -# Pre-load a scratch packet into @c fifo_timeout[0].
	 * -# Spawn @c test_thread_timeout_reply_values_wfe which calls
	 *    k_fifo_get() with K_FOREVER and posts the result.
	 * -# Wait on @c timeout_order_fifo with K_FOREVER.
	 */
	/** @par Assert
	 * -# Reply packet arrives on @c timeout_order_fifo.
	 * -# @c reply_packet.reply is true: K_FOREVER received the available item.
	 */
	/*
	 * Test k_fifo_get with timeout of K_FOREVER and the fifo
	 * should be filled be filled by the child thread based on
	 * the data availability on another fifo. In this test child
	 * thread does find data on fifo.
	 */
	scratch_packet = get_scratch_packet();
	k_fifo_put(&fifo_timeout[0], scratch_packet);

	tid[0] = k_thread_create(&ttdata[0], ttstack[0], TSTACK_SIZE,
				test_thread_timeout_reply_values_wfe,
				&reply_packet, NULL, NULL,
				FIFO_THREAD_PRIO, K_INHERIT_PERMS, K_NO_WAIT);

	packet = k_fifo_get(&timeout_order_fifo, K_FOREVER);
	zassert_true(packet != NULL);
	zassert_true(reply_packet.reply);
	put_scratch_packet(scratch_packet);
}

/**
 * @brief Threads blocking on the same fifo with distinct timeouts expire in
 * timeout-ascending order.
 *
 * @details
 * Spawns ARRAY_SIZE(timeout_order_data) threads each pending on
 * @c fifo_timeout[0] with a distinct timeout.  Verifies via
 * @c test_multiple_threads_pending() that completions are collected from
 * @c timeout_order_fifo in the expected order (one-tick tolerance allowed).
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-24-7`
 * @endverbatim
 *
 * @see k_fifo_get()
 * @draft
 */
ZTEST(fifo_timeout_1cpu, test_timeout_threads_pend_on_fifo)
{
	int32_t rv, test_data_size;

	/** @par Arrange
	 * -# @c fifo_timeout[0] is initialised and empty (suite setup).
	 * -# @c timeout_order_data[] provides per-thread fifo and timeout values.
	 */

	/** @par Act
	 * -# Invoke @c test_multiple_threads_pending() with @c timeout_order_data.
	 */

	/** @par Assert
	 * -# Return value equals TC_PASS: all threads expired in timeout-ascending
	 *    order as verified by the collected @c timeout_order fields.
	 */
	/*
	 * Test multiple threads pending on the same
	 * fifo with different timeouts
	 */
	test_data_size = ARRAY_SIZE(timeout_order_data);
	rv = test_multiple_threads_pending(timeout_order_data, test_data_size);
	zassert_equal(rv, TC_PASS);
}

/**
 * @brief Threads blocking on two different fifos with distinct timeouts expire
 * in timeout-ascending order.
 *
 * @details
 * Spawns ARRAY_SIZE(timeout_order_data_mult_fifo) threads distributed across
 * @c fifo_timeout[0] and @c fifo_timeout[1], each with a distinct timeout.
 * Verifies via @c test_multiple_threads_pending() that completions arrive in
 * the expected order regardless of which fifo each thread pended on.
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-24-7`
 * @endverbatim
 *
 * @see k_fifo_get()
 * @draft
 */
ZTEST(fifo_timeout_1cpu, test_timeout_threads_pend_on_dual_fifos)
{
	int32_t rv, test_data_size;

	/** @par Arrange
	 * -# @c fifo_timeout[0] and @c fifo_timeout[1] are initialised and
	 *    empty (suite setup).
	 * -# @c timeout_order_data_mult_fifo[] distributes threads across both fifos.
	 */

	/** @par Act
	 * -# Invoke @c test_multiple_threads_pending() with
	 *    @c timeout_order_data_mult_fifo.
	 */

	/** @par Assert
	 * -# Return value equals TC_PASS: all threads expired in timeout-ascending
	 *    order regardless of which fifo they pended on.
	 */
	/*
	 * Test multiple threads pending on different
	 * fifos with different timeouts
	 */
	test_data_size = ARRAY_SIZE(timeout_order_data_mult_fifo);
	rv = test_multiple_threads_pending(timeout_order_data_mult_fifo,
							test_data_size);
	zassert_equal(rv, TC_PASS);

}

/**
 * @brief All but the last of several waiting threads receive data in time;
 * the last thread times out correctly.
 *
 * @details
 * Spawns ARRAY_SIZE(timeout_order_data)-1 threads via
 * @c test_thread_pend_and_get_data (each receives a scratch packet before its
 * timeout) and one final thread via @c test_thread_pend_and_timeout (receives
 * no data and must time out).  Verifies via @c test_multiple_threads_get_data()
 * that each thread completes in queue order, confirming that timeout recomputation
 * is correct when earlier waiters are satisfied.
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-24-3`
 * - :external+req:ref:`zep-srs-24-7`
 * @endverbatim
 *
 * @see k_fifo_get(), k_fifo_put()
 * @draft
 */
ZTEST(fifo_timeout_1cpu, test_timeout_threads_pend_fail_on_fifo)
{
	int32_t rv, test_data_size;

	/** @par Arrange
	 * -# @c fifo_timeout[0] is initialised and empty (suite setup).
	 * -# @c timeout_order_data[] provides per-thread fifo and timeout values.
	 */

	/** @par Act
	 * -# Invoke @c test_multiple_threads_get_data() with @c timeout_order_data.
	 *    The first N-1 threads are fed scratch packets in queue order; the
	 *    last thread receives nothing and times out.
	 */

	/** @par Assert
	 * -# Return value equals TC_PASS: the first N-1 threads completed in
	 *    queue order (each received a packet), and the last thread timed out
	 *    as asserted internally by @c test_thread_pend_and_timeout.
	 */
	/*
	 * Test multiple threads pending on same
	 * fifo with different timeouts, but getting
	 * the data in time, except the last one.
	 */
	test_data_size = ARRAY_SIZE(timeout_order_data);
	rv = test_multiple_threads_get_data(timeout_order_data, test_data_size);
	zassert_equal(rv, TC_PASS);
}

/**
 * @brief Test fifo init
 * @see k_fifo_init(), k_fifo_put()
 */
static void *test_timeout_setup(void)
{
	intptr_t ii;

	/* Init kernel objects */
	k_fifo_init(&fifo_timeout[0]);
	k_fifo_init(&fifo_timeout[1]);
	k_fifo_init(&timeout_order_fifo);
	k_fifo_init(&scratch_fifo_packets_fifo);

	/* Fill scratch fifo */
	for (ii = 0; ii < NUM_SCRATCH_FIFO_PACKETS; ii++) {
		scratch_fifo_packets[ii].data_if_needed = (void *)ii;
		k_fifo_put(&scratch_fifo_packets_fifo,
				(void *)&scratch_fifo_packets[ii]);
	}

	return NULL;
}

/**
 * @defgroup fifo_timeout FIFO Timeout ZTest suite
 * @ingroup kernel_fifo_timeout_module
 * @brief k_fifo_get() tests on a non-empty fifo that do not require single-CPU
 *        pinning.
 *
 * @details
 * Contains tests that can run without CPU pinning.  Verifies that k_fifo_get()
 * returns a non-null item immediately with K_NO_WAIT and blocks correctly with
 * K_FOREVER when an item is already present, confirming basic retrieval
 * behaviour independent of timeout mechanics.
 */
ZTEST_SUITE(fifo_timeout, NULL, test_timeout_setup, NULL, NULL, NULL);

/**
 * @defgroup fifo_timeout_1cpu FIFO Timeout 1CPU ZTest suite
 * @ingroup kernel_fifo_timeout_module
 * @brief k_fifo_get() timeout tests requiring single-CPU pinning -- expiry on
 *        empty fifos, data-in-time scenarios, and multi-thread timeout ordering.
 *
 * @details
 * Contains tests pinned to a single CPU via ztest_simple_1cpu_before/after.
 * Covers timeout expiry and K_NO_WAIT behaviour on empty fifos, correct item
 * delivery when a child thread enqueues data within a finite timeout or the
 * caller uses K_NO_WAIT/K_FOREVER on a pre-loaded fifo, timeout-order
 * correctness when multiple threads pend on one fifo or across two fifos, and
 * timeout recomputation correctness when N-1 threads receive data before the
 * last waiter times out.
 */
ZTEST_SUITE(fifo_timeout_1cpu, NULL, test_timeout_setup,
		ztest_simple_1cpu_before, ztest_simple_1cpu_after, NULL);
