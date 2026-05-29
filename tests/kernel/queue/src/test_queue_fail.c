/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_queue_fail.c Error-path and failure-mode tests for queue APIs
 */

#include "test_queue.h"

#define TIMEOUT K_MSEC(100)
#define STACK_SIZE (512 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define LIST_LEN 2

static K_THREAD_STACK_DEFINE(tstack, STACK_SIZE);
static struct k_thread tdata;
K_SEM_DEFINE(sem, 0, 1);

/*test cases*/
/**
 * @brief k_queue_get() returns NULL on an empty queue with both K_NO_WAIT and a finite timeout.
 *
 * @details
 * Verifies that k_queue_get() does not block indefinitely and returns NULL
 * when no item is available -- both when called with K_NO_WAIT (immediate
 * return) and with a finite timeout (TIMEOUT = 100 ms, after which it expires).
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-20-6`
 * @endverbatim
 *
 * @see k_queue_init(), k_queue_get()
 * @draft
 */
ZTEST(queue_api_1cpu, test_queue_get_fail)
{
	static struct k_queue queue;

	/** @par Arrange
	 * -# Initialise an empty @p queue via k_queue_init().
	 */
	k_queue_init(&queue);

	/** @par Act
	 * -# Call k_queue_get() with K_NO_WAIT on the empty queue.
	 * -# Call k_queue_get() with TIMEOUT (100 ms) on the still-empty queue.
	 */

	/** @par Assert
	 * -# Both calls return NULL, confirming that k_queue_get() does not
	 *    block past its timeout and correctly signals an empty queue.
	 */
	/**TESTPOINT: queue get returns NULL*/
	zassert_is_null(k_queue_get(&queue, K_NO_WAIT), NULL);
	zassert_is_null(k_queue_get(&queue, TIMEOUT), NULL);
}

/**
 * @brief Signal readiness, then receive one item and assert it equals the expected pointer.
 *
 * @details
 * Signals @c sem immediately to notify the creating thread that this thread
 * has started and is about to block.  Then calls k_queue_get() with K_FOREVER
 * until an item arrives, and asserts via zassert_equal() that the received
 * pointer equals @p p2, confirming that the item delivered by the subsequent
 * k_queue_append_list() call is the exact node that was passed as the list head.
 *
 * @param p1 Pointer to the queue under test, cast to @c void*.
 * @param p2 Expected pointer value for the received item, cast to @c void*.
 * @param p3 Unused.
 *
 * @see k_queue_get()
 *
 * @ingroup queue_api_procedures
 */
static void tThread_entry(void *p1, void *p2, void *p3)
{
	k_sem_give(&sem);

	/* wait the queue for data */
	qdata_t *p = k_queue_get((struct k_queue *)p1, K_FOREVER);

	zassert_equal(p, p2, "Failed to append a unnormal list");
}

/**
 * @brief k_queue_append_list() returns -EINVAL for a NULL head or tail, and
 * correctly wakes a waiting thread when given a valid single-node list.
 *
 * @details
 * Verifies three error and edge conditions of k_queue_append_list():
 * -# A NULL head pointer causes the call to return -EINVAL.
 * -# A NULL tail pointer causes the call to return -EINVAL.
 * -# A single-node list (head == tail, next == NULL) is accepted and
 *    delivered to a thread blocked on k_queue_get() with K_FOREVER; the
 *    consumer thread (@c tThread_entry) asserts via zassert_equal() that
 *    the received pointer equals the node that was passed as the list head.
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-20-11`
 * @endverbatim
 *
 * @see k_queue_init(), k_queue_append_list()
 * @draft
 */
ZTEST(queue_api, test_queue_append_list_error)
{
	static qdata_t data_l[2];
	static struct k_queue queue;
	static qdata_t *head = NULL, *tail = &data_l[1];

	/** @par Arrange
	 * -# Initialise @p queue via k_queue_init().
	 * -# Zero-initialise @p data_l[].
	 * -# Set head = NULL, tail = &data_l[1].
	 */
	k_queue_init(&queue);
	memset(data_l, 0, sizeof(data_l));

	/** @par Act -- NULL head
	 * -# Call k_queue_append_list() with head == NULL and a valid tail.
	 */
	/** @par Assert
	 * -# Return value equals -EINVAL.
	 */
	/* Check if the list of head is equal to null */
	zassert_true(k_queue_append_list(&queue, (uint32_t *)head,
			(uint32_t *)tail) == -EINVAL,
			"failed to CHECKIF head == NULL");

	/** @par Act -- NULL tail
	 * -# Set head = &data_l[0], tail = NULL.
	 * -# Call k_queue_append_list() with a valid head and tail == NULL.
	 */
	/** @par Assert
	 * -# Return value equals -EINVAL.
	 */
	/* Check if the list of tail is equal to null */
	head = &data_l[0];
	tail = NULL;
	zassert_true(k_queue_append_list(&queue, (uint32_t *)head,
			(uint32_t *)tail) == -EINVAL,
			"failed to CHECKIF tail == NULL");

	/** @par Act -- single-node list with waiting consumer thread
	 * -# Re-initialise @p queue via k_queue_init().
	 * -# Build a single-node list: head = &data_l[0], head->snode.next = NULL.
	 * -# Create a consumer thread (@c tThread_entry) at preemptive priority 0
	 *    that will block on k_queue_get() with K_FOREVER.
	 * -# Wait for the consumer thread to signal @c sem (confirming it has
	 *    started and is about to block).
	 * -# Call k_queue_append_list() with head == tail == &data_l[0].
	 */
	/** @par Assert
	 * -# @c tThread_entry receives the item and asserts via zassert_equal()
	 *    that the received pointer equals @p head, confirming delivery of the
	 *    single-node list to the waiting thread.
	 */
	/* Initializing the queue for re-using below */
	k_queue_init(&queue);

	/* Append unnormal list(just one node)into the queue for sub-thread */
	head = &data_l[0];
	head->snode.next = NULL;

	k_thread_create(&tdata, tstack, STACK_SIZE, tThread_entry, &queue,
			head, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	/* Delay for thread initializing */
	k_sem_take(&sem, K_FOREVER);

	k_queue_append_list(&queue, (uint32_t *)head, (uint32_t *)head);

	k_thread_join(&tdata, K_FOREVER);
}

/**
 * @brief k_queue_merge_slist() returns -EINVAL for an empty slist or a slist with NULL tail.
 *
 * @details
 * Verifies two invalid-input conditions for k_queue_merge_slist():
 * -# An empty @c sys_slist_t causes the call to return -EINVAL.
 * -# A slist that has nodes appended but whose @c tail pointer has been
 *    set to NULL (a corrupted/incomplete list) causes the call to return
 *    a non-zero error code.
 *
 * @verbatim embed:rst
 * - :external+req:ref:`zep-srs-20-12`
 * @endverbatim
 *
 * @see k_queue_init(), k_queue_merge_slist()
 * @draft
 */
ZTEST(queue_api, test_queue_merge_list_error)
{
	qdata_t data_sl[2];
	static struct k_queue queue;
	sys_slist_t slist;

	/** @par Arrange
	 * -# Initialise @p queue via k_queue_init().
	 * -# Initialise an empty @p slist via sys_slist_init().
	 * -# Zero-initialise @p data_sl[].
	 */
	k_queue_init(&queue);
	sys_slist_init(&slist);
	memset(data_sl, 0, sizeof(data_sl));

	/** @par Act -- empty slist
	 * -# Call k_queue_merge_slist() with an empty @p slist.
	 */
	/** @par Assert
	 * -# Return value equals -EINVAL.
	 */
	/* Check if the slist is empty */
	zassert_true(k_queue_merge_slist(&queue, &slist) == -EINVAL,
			"Failed to CHECKIF slist is empty");

	/** @par Act -- slist with NULL tail
	 * -# Append @p data_sl[0] and @p data_sl[1] to @p slist.
	 * -# Corrupt the list by setting slist.tail = NULL.
	 * -# Call k_queue_merge_slist() with the corrupted @p slist.
	 */
	/** @par Assert
	 * -# Return value is non-zero, confirming the corrupted tail is rejected.
	 */
	/* Check if the tail of the slist is null */
	sys_slist_append(&slist, (sys_snode_t *)&(data_sl[0].snode));
	sys_slist_append(&slist, (sys_snode_t *)&(data_sl[1].snode));
	slist.tail = NULL;
	zassert_true(k_queue_merge_slist(&queue, &slist) != 0,
			"Failed to CHECKIF the tail of slist == null");
}

#ifdef CONFIG_USERSPACE
/**
 * @brief Calling k_queue_init() with a NULL pointer from user mode triggers a kernel fault.
 *
 * @details
 * Marks the expected kernel fault valid via ztest_set_fault_valid(), then
 * calls k_queue_init(NULL) from user mode.  The kernel must generate an oops
 * that ztest intercepts as the valid expected fault.
 *
 * @see k_queue_init()
 * @draft
 */
ZTEST_USER(queue_api, test_queue_init_null)
{
	/** @par Act
	 * -# Mark the expected fault valid via ztest_set_fault_valid().
	 * -# Call k_queue_init(NULL) from user mode.
	 */
	/** @par Assert
	 * -# A kernel fault is triggered and intercepted by ztest as the valid
	 *    expected fault, confirming NULL-pointer protection.
	 */
	ztest_set_fault_valid(true);
	k_queue_init(NULL);
}

/**
 * @brief Calling k_queue_alloc_append() with a NULL queue pointer from user mode triggers a kernel fault.
 *
 * @details
 * Marks the expected kernel fault valid via ztest_set_fault_valid(), then
 * calls k_queue_alloc_append(NULL, &data) from user mode.  The kernel must
 * generate an oops that ztest intercepts as the valid expected fault.
 *
 * @see k_queue_alloc_append()
 * @draft
 */
ZTEST_USER(queue_api, test_queue_alloc_append_null)
{
	qdata_t data;

	/** @par Arrange
	 * -# Zero-initialise a local @p data item.
	 */
	memset(&data, 0, sizeof(data));

	/** @par Act
	 * -# Mark the expected fault valid via ztest_set_fault_valid().
	 * -# Call k_queue_alloc_append(NULL, &data) from user mode.
	 */
	/** @par Assert
	 * -# A kernel fault is triggered and intercepted by ztest as the valid
	 *    expected fault, confirming NULL-pointer protection.
	 */
	ztest_set_fault_valid(true);
	k_queue_alloc_append(NULL, &data);
}

/**
 * @brief Calling k_queue_alloc_prepend() with a NULL queue pointer from user mode triggers a kernel fault.
 *
 * @details
 * Marks the expected kernel fault valid via ztest_set_fault_valid(), then
 * calls k_queue_alloc_prepend(NULL, &data) from user mode.  The kernel must
 * generate an oops that ztest intercepts as the valid expected fault.
 *
 * @see k_queue_alloc_prepend()
 * @draft
 */
ZTEST_USER(queue_api, test_queue_alloc_prepend_null)
{
	qdata_t data;

	/** @par Arrange
	 * -# Zero-initialise a local @p data item.
	 */
	memset(&data, 0, sizeof(data));

	/** @par Act
	 * -# Mark the expected fault valid via ztest_set_fault_valid().
	 * -# Call k_queue_alloc_prepend(NULL, &data) from user mode.
	 */
	/** @par Assert
	 * -# A kernel fault is triggered and intercepted by ztest as the valid
	 *    expected fault, confirming NULL-pointer protection.
	 */
	ztest_set_fault_valid(true);
	k_queue_alloc_prepend(NULL, &data);
}

/**
 * @brief Calling k_queue_get() with a NULL queue pointer from user mode triggers a kernel fault.
 *
 * @details
 * Marks the expected kernel fault valid via ztest_set_fault_valid(), then
 * calls k_queue_get(NULL, K_FOREVER) from user mode.  The kernel must
 * generate an oops that ztest intercepts as the valid expected fault.
 *
 * @see k_queue_get()
 * @draft
 */
ZTEST_USER(queue_api, test_queue_get_null)
{
	/** @par Act
	 * -# Mark the expected fault valid via ztest_set_fault_valid().
	 * -# Call k_queue_get(NULL, K_FOREVER) from user mode.
	 */
	/** @par Assert
	 * -# A kernel fault is triggered and intercepted by ztest as the valid
	 *    expected fault, confirming NULL-pointer protection.
	 */
	ztest_set_fault_valid(true);
	k_queue_get(NULL, K_FOREVER);
}

/**
 * @brief Calling k_queue_is_empty() with a NULL queue pointer from user mode triggers a kernel fault.
 *
 * @details
 * Marks the expected kernel fault valid via ztest_set_fault_valid(), then
 * calls k_queue_is_empty(NULL) from user mode.  The kernel must generate
 * an oops that ztest intercepts as the valid expected fault.
 *
 * @see k_queue_is_empty()
 * @draft
 */
ZTEST_USER(queue_api, test_queue_is_empty_null)
{
	/** @par Act
	 * -# Mark the expected fault valid via ztest_set_fault_valid().
	 * -# Call k_queue_is_empty(NULL) from user mode.
	 */
	/** @par Assert
	 * -# A kernel fault is triggered and intercepted by ztest as the valid
	 *    expected fault, confirming NULL-pointer protection.
	 */
	ztest_set_fault_valid(true);
	k_queue_is_empty(NULL);
}

/**
 * @brief Calling k_queue_peek_head() with a NULL queue pointer from user mode triggers a kernel fault.
 *
 * @details
 * Marks the expected kernel fault valid via ztest_set_fault_valid(), then
 * calls k_queue_peek_head(NULL) from user mode.  The kernel must generate
 * an oops that ztest intercepts as the valid expected fault.
 *
 * @see k_queue_peek_head()
 * @draft
 */
ZTEST_USER(queue_api, test_queue_peek_head_null)
{
	/** @par Act
	 * -# Mark the expected fault valid via ztest_set_fault_valid().
	 * -# Call k_queue_peek_head(NULL) from user mode.
	 */
	/** @par Assert
	 * -# A kernel fault is triggered and intercepted by ztest as the valid
	 *    expected fault, confirming NULL-pointer protection.
	 */
	ztest_set_fault_valid(true);
	k_queue_peek_head(NULL);
}

/**
 * @brief Calling k_queue_peek_tail() with a NULL queue pointer from user mode triggers a kernel fault.
 *
 * @details
 * Marks the expected kernel fault valid via ztest_set_fault_valid(), then
 * calls k_queue_peek_tail(NULL) from user mode.  The kernel must generate
 * an oops that ztest intercepts as the valid expected fault.
 *
 * @see k_queue_peek_tail()
 * @draft
 */
ZTEST_USER(queue_api, test_queue_peek_tail_null)
{
	/** @par Act
	 * -# Mark the expected fault valid via ztest_set_fault_valid().
	 * -# Call k_queue_peek_tail(NULL) from user mode.
	 */
	/** @par Assert
	 * -# A kernel fault is triggered and intercepted by ztest as the valid
	 *    expected fault, confirming NULL-pointer protection.
	 */
	ztest_set_fault_valid(true);
	k_queue_peek_tail(NULL);
}

/**
 * @brief k_queue_cancel_wait() is a no-op on a queue with no waiting thread;
 * calling it with NULL from user mode triggers a kernel fault.
 *
 * @details
 * Verifies two conditions for k_queue_cancel_wait():
 * -# Calling it on a valid, initialised queue that has no waiting thread
 *    completes without error.
 * -# Calling it with a NULL pointer from user mode causes a kernel oops
 *    that ztest intercepts as the valid expected fault.
 *
 * @see k_queue_init(), k_queue_cancel_wait()
 * @draft
 */
ZTEST_USER(queue_api, test_queue_cancel_wait_error)
{
	struct k_queue *q;

	/** @par Arrange
	 * -# Allocate a kernel queue object via k_object_alloc() and assert
	 *    the result is non-null.
	 * -# Initialise the allocated queue via k_queue_init().
	 */
	q = k_object_alloc(K_OBJ_QUEUE);
	zassert_not_null(q, "no memory for allocated queue object");
	k_queue_init(q);

	/** @par Act -- cancel on queue with no waiter
	 * -# Call k_queue_cancel_wait() on the valid queue that has no
	 *    waiting thread.
	 */
	/** @par Assert
	 * -# The call completes without error or assertion failure.
	 */
	/* Check if cancel a qeueu that no thread to wait */
	k_queue_cancel_wait(q);

	/** @par Act -- NULL pointer
	 * -# Mark the expected fault valid via ztest_set_fault_valid().
	 * -# Call k_queue_cancel_wait(NULL) from user mode.
	 */
	/** @par Assert
	 * -# A kernel fault is triggered and intercepted by ztest as the valid
	 *    expected fault, confirming NULL-pointer protection.
	 */
	/* Check if cancel a null pointer */
	ztest_set_fault_valid(true);
	k_queue_cancel_wait(NULL);
}

#endif /* CONFIG_USERSPACE */
