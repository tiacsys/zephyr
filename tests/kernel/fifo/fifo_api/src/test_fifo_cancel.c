/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_fifo_cancel.c k_fifo_cancel_wait() tests for the FIFO API
 */

#include "test_fifo.h"

#define STACK_SIZE (512 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define LIST_LEN 2
/**TESTPOINT: init via K_FIFO_DEFINE*/
K_FIFO_DEFINE(kfifo_c);

struct k_fifo fifo_c;

static K_THREAD_STACK_DEFINE(tstack_cancel, STACK_SIZE);
static struct k_thread thread;

/** @cond INTERNAL */
static void t_cancel_wait_entry(void *p1, void *p2, void *p3)
{
	k_sleep(K_MSEC(50));
	k_fifo_cancel_wait((struct k_fifo *)p1);
}
/** @endcond */

/**
 * @brief Run the cancel-wait scenario and verify k_fifo_get() returns NULL
 * within the expected timeframe.
 *
 * @details
 * Creates a helper thread (@c t_cancel_wait_entry) at preemptive priority 0
 * that sleeps 50 ms then calls k_fifo_cancel_wait().  The current thread
 * immediately calls k_fifo_get() with a 500 ms timeout on the empty @p pfifo
 * and records the elapsed time.  Asserts via zassert_is_null() that the return
 * value is NULL and via zassert_true() that the elapsed time is under 80 ms,
 * confirming that the cancel arrived well before the 500 ms timeout.
 *
 * @param pfifo Non-null pointer to an initialised, empty fifo.
 *
 * @see k_fifo_get(), k_fifo_cancel_wait()
 *
 * @ingroup fifo_api_procedures
 */
static void tfifo_thread_thread(struct k_fifo *pfifo)
{
	k_tid_t tid = k_thread_create(&thread, tstack_cancel, STACK_SIZE,
				      t_cancel_wait_entry, pfifo, NULL, NULL,
				      K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	uint32_t start_t = k_uptime_get_32();
	void *ret = k_fifo_get(pfifo, K_MSEC(500));
	uint32_t dur = k_uptime_get_32() - start_t;

	/* While we observed the side effect of the last statement
	 * ( call to k_fifo_cancel_wait) of the thread, it's not fact
	 * that it returned, within the thread. Then it may happen
	 * that the test runner below will try to create another
	 * thread in the same stack space, then 1st thread returns
	 * from the call, leading to crash.
	 */
	k_thread_abort(tid);
	zassert_is_null(ret,
			"k_fifo_get didn't get 'timeout expired' status");
	/* 80 includes generous fuzz factor as k_sleep() will add an extra
	 * tick for non-tickless systems, and we may cross another tick
	 * boundary while doing this. We just want to ensure we didn't
	 * hit the timeout anyway.
	 */
	zassert_true(dur < 80,
		     "k_fifo_get didn't get cancelled in expected timeframe");
}

/**
 * @brief k_fifo_cancel_wait() causes a blocked k_fifo_get() to return NULL
 * well before its timeout expires.
 *
 * @details
 * Runs the cancel-wait scenario via @c tfifo_thread_thread() on both a
 * runtime-initialised fifo and a compile-time-defined fifo.  In each case a
 * helper thread calls k_fifo_cancel_wait() after 50 ms while the current
 * thread blocks on k_fifo_get() with a 500 ms timeout.  The call must return
 * NULL in under 80 ms, confirming that cancellation takes effect promptly.
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-24-2`
 * @endverbatim
 *
 * @see k_fifo_init(), k_fifo_get(), k_fifo_cancel_wait()
 * @draft
 */
ZTEST(fifo_api_1cpu, test_fifo_cancel_wait)
{
	/** @par Arrange
	 * -# Initialise @p fifo_c at runtime via k_fifo_init().
	 * -# @p kfifo_c is already available as a compile-time-defined fifo.
	 */
	/**TESTPOINT: init via k_fifo_init*/
	k_fifo_init(&fifo_c);

	/** @par Act
	 * -# Run @c tfifo_thread_thread() on the runtime-initialised fifo:
	 *    a helper thread cancels the wait after 50 ms.
	 * -# Run @c tfifo_thread_thread() on the compile-time-defined fifo.
	 */

	/** @par Assert
	 * -# Both invocations complete without assertion failure: k_fifo_get()
	 *    returns NULL and the elapsed time is under 80 ms in each case.
	 */
	tfifo_thread_thread(&fifo_c);

	/**TESTPOINT: test K_FIFO_DEFINEed fifo*/
	tfifo_thread_thread(&kfifo_c);
}
