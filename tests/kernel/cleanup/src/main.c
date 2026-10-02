/**
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/cleanup/kernel.h>

extern struct k_heap _system_heap;
static size_t free_bytes;

static void *cleanup_setup(void)
{
	struct sys_memory_stats stats;

	zassert_ok(sys_heap_runtime_stats_get(&_system_heap.heap, &stats));

	/* Store the amount of heap bytes usable in tests */
	free_bytes = stats.free_bytes;

	return NULL;
}

static void cleanup_after(void *fixture)
{
	struct sys_memory_stats stats;

	ARG_UNUSED(fixture);

	zassert_ok(sys_heap_runtime_stats_get(&_system_heap.heap, &stats));
	zassert_equal(free_bytes, stats.free_bytes, "Memory leaked in a test");
}

/**
 * @brief Verify that scope_guard(k_mutex) locks a mutex and unlocks it at the
 * end of the scope.
 *
 * @details
 * The guard locks the mutex where the code declares it. The guard unlocks the
 * mutex when the code leaves the scope.
 *
 * Test steps:
 * - Initialize a mutex with k_mutex_init().
 * - In an inner scope, declare scope_guard(k_mutex) on the mutex. Then read the
 *   lock count.
 * - Leave the inner scope. Then read the lock count.
 *
 * Expected result:
 * - k_mutex_init() returns 0.
 * - In the scope, the lock count is 1.
 * - After the scope, the lock count is 0.
 *
 * @see scope_guard(), k_mutex_lock(), k_mutex_unlock()
 */
ZTEST(cleanup_api, test_guard_k_mutex)
{
	struct k_mutex lock;
	int ret;

	ret = k_mutex_init(&lock);
	zassert_ok(ret);

	{
		scope_guard(k_mutex)(&lock);
		zexpect_equal(lock.lock_count, 1);
	}

	zexpect_equal(lock.lock_count, 0);
}

/**
 * @brief Verify that scope_defer(k_mutex_unlock) unlocks a mutex at the end of
 * the scope.
 *
 * @details
 * The deferred call runs k_mutex_unlock() when the code leaves the scope.
 *
 * Test steps:
 * - Initialize a mutex with k_mutex_init().
 * - In an inner scope, lock the mutex with k_mutex_lock() and K_NO_WAIT.
 * - Declare scope_defer(k_mutex_unlock) on the mutex. Then read the lock count.
 * - Leave the inner scope. Then read the lock count.
 *
 * Expected result:
 * - k_mutex_init() and k_mutex_lock() return 0.
 * - In the scope, the lock count is 1.
 * - After the scope, the lock count is 0.
 *
 * @see scope_defer(), k_mutex_unlock()
 */
ZTEST(cleanup_api, test_defer_k_mutex_unlock)
{
	struct k_mutex lock;
	int ret;

	ret = k_mutex_init(&lock);
	zassert_ok(ret);

	{
		ret = k_mutex_lock(&lock, K_NO_WAIT);
		zassert_ok(ret);
		scope_defer(k_mutex_unlock)(&lock);

		zexpect_equal(lock.lock_count, 1);
	}

	zexpect_equal(lock.lock_count, 0);
}

/**
 * @brief Verify that scope_guard(k_sem) takes a semaphore and gives it at the
 * end of the scope.
 *
 * @details
 * The guard takes the semaphore where the code declares it. The guard gives the
 * semaphore when the code leaves the scope.
 *
 * Test steps:
 * - Initialize a semaphore with k_sem_init(), the count 1 and the limit 1.
 * - In an inner scope, declare scope_guard(k_sem) on the semaphore. Then read
 *   the count.
 * - Leave the inner scope. Then read the count.
 *
 * Expected result:
 * - k_sem_init() returns 0.
 * - In the scope, the count is 0.
 * - After the scope, the count is 1.
 *
 * @see scope_guard(), k_sem_take(), k_sem_give()
 */
ZTEST(cleanup_api, test_guard_k_sem)
{
	struct k_sem lock;
	int ret;

	ret = k_sem_init(&lock, 1, 1);
	zassert_ok(ret);

	{
		scope_guard(k_sem)(&lock);
		zexpect_equal(lock.count, 0);
	}

	zexpect_equal(lock.count, 1);
}

/**
 * @brief Verify that scope_defer(k_sem_give) gives a semaphore at the end of
 * the scope.
 *
 * @details
 * The deferred call runs k_sem_give() when the code leaves the scope.
 *
 * Test steps:
 * - Initialize a semaphore with k_sem_init(), the count 1 and the limit 1.
 * - In an inner scope, take the semaphore with k_sem_take() and K_NO_WAIT.
 * - Declare scope_defer(k_sem_give) on the semaphore. Then read the count.
 * - Leave the inner scope. Then read the count.
 *
 * Expected result:
 * - k_sem_init() and k_sem_take() return 0.
 * - In the scope, the count is 0.
 * - After the scope, the count is 1.
 *
 * @see scope_defer(), k_sem_give()
 */
ZTEST(cleanup_api, test_defer_k_sem_give)
{
	struct k_sem lock;
	int ret;

	ret = k_sem_init(&lock, 1, 1);
	zassert_ok(ret);

	{
		ret = k_sem_take(&lock, K_NO_WAIT);
		zassert_ok(ret);
		scope_defer(k_sem_give)(&lock);

		zexpect_equal(lock.count, 0);
	}

	zexpect_equal(lock.count, 1);
}

/**
 * @brief Verify that a scoped_guard(k_mutex) block runs one time with the mutex
 * locked, and then unlocks it.
 *
 * @details
 * scoped_guard() holds the guard only for the block that follows it.
 *
 * Test steps:
 * - Initialize a mutex with k_mutex_init().
 * - In a scoped_guard(k_mutex) block, increment a run counter and read the lock
 *   count.
 * - After the block, read the run counter and the lock count.
 *
 * Expected result:
 * - In the block, the lock count is 1.
 * - The block runs one time.
 * - After the block, the lock count is 0.
 *
 * @see scoped_guard(), k_mutex_lock(), k_mutex_unlock()
 */
ZTEST(cleanup_api, test_scoped_guard_k_mutex)
{
	struct k_mutex lock;
	int runs = 0;

	zassert_ok(k_mutex_init(&lock));

	scoped_guard(k_mutex, &lock) {
		runs++;
		zexpect_equal(lock.lock_count, 1);
	}

	zexpect_equal(runs, 1);
	zexpect_equal(lock.lock_count, 0);
}

/**
 * @brief Verify that a scoped_guard(k_sem) block runs one time with the
 * semaphore taken, and then gives it.
 *
 * @details
 * scoped_guard() holds the guard only for the block that follows it.
 *
 * Test steps:
 * - Initialize a semaphore with k_sem_init(), the count 1 and the limit 1.
 * - In a scoped_guard(k_sem) block, increment a run counter and read the count.
 * - After the block, read the run counter and the count.
 *
 * Expected result:
 * - In the block, the count is 0.
 * - The block runs one time.
 * - After the block, the count is 1.
 *
 * @see scoped_guard(), k_sem_take(), k_sem_give()
 */
ZTEST(cleanup_api, test_scoped_guard_k_sem)
{
	struct k_sem lock;
	int runs = 0;

	zassert_ok(k_sem_init(&lock, 1, 1));

	scoped_guard(k_sem, &lock) {
		runs++;
		zexpect_equal(lock.count, 0);
	}

	zexpect_equal(runs, 1);
	zexpect_equal(lock.count, 1);
}

/**
 * @brief Verify that a break statement in a scoped_guard(k_mutex) block leaves
 * the block and unlocks the mutex.
 *
 * @details
 * A break statement in the block must leave the block. The block must not run
 * again, and the guard must release the mutex.
 *
 * Test steps:
 * - Initialize a mutex with k_mutex_init().
 * - In a scoped_guard(k_mutex) block, increment a run counter and read the lock
 *   count.
 * - Leave the block with a break statement.
 * - After the block, read the run counter and the lock count.
 *
 * Expected result:
 * - In the block, the lock count is 1.
 * - The block runs one time.
 * - After the block, the lock count is 0.
 *
 * @see scoped_guard()
 */
ZTEST(cleanup_api, test_scoped_guard_break)
{
	struct k_mutex lock;
	int runs = 0;

	zassert_ok(k_mutex_init(&lock));

	scoped_guard(k_mutex, &lock) {
		runs++;
		zexpect_equal(lock.lock_count, 1);
		break;
	}

	zexpect_equal(runs, 1);             /* ran once, break did not re-enter */
	zexpect_equal(lock.lock_count, 0);  /* break released the guard */
}

/**
 * @brief Verify that a scoped_cond_guard(k_sem_try) block runs with the
 * semaphore taken when the semaphore is available.
 *
 * @details
 * The k_sem_try guard takes the semaphore with K_NO_WAIT. If the take fails,
 * scoped_cond_guard() runs the fail statement and not the block. In this test,
 * the fail statement is zassert_unreachable().
 *
 * Test steps:
 * - Initialize a semaphore with k_sem_init(), the count 1 and the limit 1.
 * - In a scoped_cond_guard(k_sem_try) block, increment a run counter and read
 *   the count.
 * - After the block, read the run counter and the count.
 *
 * Expected result:
 * - The fail statement does not run.
 * - In the block, the count is 0.
 * - The block runs one time.
 * - After the block, the count is 1.
 *
 * @see scoped_cond_guard(), k_sem_take(), k_sem_give()
 */
ZTEST(cleanup_api, test_scoped_cond_guard_acquired)
{
	struct k_sem lock;
	int runs = 0;

	zassert_ok(k_sem_init(&lock, 1, 1));

	scoped_cond_guard(k_sem_try, zassert_unreachable(), &lock) {
		runs++;
		zexpect_equal(lock.count, 0);
	}

	zexpect_equal(runs, 1);
	zexpect_equal(lock.count, 1);
}

/**
 * @brief Verify that scoped_cond_guard(k_sem_try) skips its block and runs the
 * fail statement when the semaphore is not available.
 *
 * @details
 * The k_sem_try guard takes the semaphore with K_NO_WAIT. The semaphore has no
 * tokens, so the take fails.
 *
 * Test steps:
 * - Initialize a semaphore with k_sem_init(), the count 0 and the limit 1.
 * - Use scoped_cond_guard(k_sem_try) with a fail statement that sets a flag.
 * - In the block, increment a run counter.
 * - After the block, read the run counter, the flag and the count.
 *
 * Expected result:
 * - The block does not run.
 * - The fail statement runs and sets the flag.
 * - The count stays 0.
 *
 * @see scoped_cond_guard(), k_sem_take()
 */
ZTEST(cleanup_api, test_scoped_cond_guard_busy)
{
	struct k_sem lock;
	int runs = 0;
	bool failed = false;

	/* No tokens available, so the take with K_NO_WAIT must fail */
	zassert_ok(k_sem_init(&lock, 0, 1));

	scoped_cond_guard(k_sem_try, failed = true, &lock) {
		runs++;
	}

	/* The body must be skipped and the fail statement must run */
	zexpect_equal(runs, 0);
	zexpect_true(failed);
	zexpect_equal(lock.count, 0);
}

/**
 * @brief Verify that scoped_guard() with the conditional k_sem_try guard skips
 * its block when the semaphore is not available.
 *
 * @details
 * The plain scoped_guard() can use a conditional guard. If the guard cannot
 * take the lock, the block must not run without the lock.
 *
 * Test steps:
 * - Initialize a semaphore with k_sem_init(), the count 0 and the limit 1.
 * - In a scoped_guard(k_sem_try) block, increment a run counter.
 * - After the block, read the run counter and the count.
 *
 * Expected result:
 * - The block does not run.
 * - The count stays 0.
 *
 * @see scoped_guard(), k_sem_take()
 */
ZTEST(cleanup_api, test_scoped_guard_cond_busy_skips)
{
	struct k_sem lock;
	int runs = 0;

	/* No tokens available, so the take with K_NO_WAIT must fail */
	zassert_ok(k_sem_init(&lock, 0, 1));

	/* Using a conditional guard with the plain scoped_guard must skip the body
	 * when the lock cannot be acquired, never run it without the lock held.
	 */
	scoped_guard(k_sem_try, &lock) {
		runs++;
	}

	zexpect_equal(runs, 0);
	zexpect_equal(lock.count, 0);
}

/**
 * @brief Verify that scope_defer(k_free) frees a k_malloc() block when the test
 * function returns.
 *
 * @details
 * The suite after function cleanup_after() compares the free bytes of the
 * system heap with the value that cleanup_setup() stored. A difference fails
 * the test as a memory leak.
 *
 * Test steps:
 * - Allocate 10 bytes with k_malloc().
 * - Declare scope_defer(k_free) on the pointer.
 * - Return from the test function.
 *
 * Expected result:
 * - k_malloc() returns a pointer that is not NULL.
 * - After the test, the system heap has the same number of free bytes as before
 *   the suite.
 *
 * @see scope_defer(), k_malloc(), k_free()
 */
ZTEST(cleanup_api, test_defer_k_free)
{
	void *my_ptr = k_malloc(10);

	scope_defer(k_free)(my_ptr);

	zassert_not_null(my_ptr);

	/* Rely on cleanup_after to check that the ptr is freed */
}

/**
 * @brief Verify that scope_defer(k_heap_free) frees a k_heap_alloc() block when
 * the test function returns.
 *
 * @details
 * The suite after function cleanup_after() compares the free bytes of the
 * system heap with the value that cleanup_setup() stored. A difference fails
 * the test as a memory leak.
 *
 * Test steps:
 * - Allocate 42 bytes from the system heap with k_heap_alloc() and K_FOREVER.
 * - Declare scope_defer(k_heap_free) on the heap and the pointer.
 * - Return from the test function.
 *
 * Expected result:
 * - k_heap_alloc() returns a pointer that is not NULL.
 * - After the test, the system heap has the same number of free bytes as before
 *   the suite.
 *
 * @see scope_defer(), k_heap_alloc(), k_heap_free()
 */
ZTEST(cleanup_api, test_defer_k_heap_free)
{
	void *my_ptr = k_heap_alloc(&_system_heap, 42, K_FOREVER);

	scope_defer(k_heap_free)(&_system_heap, my_ptr);

	zassert_not_null(my_ptr);

	/* Rely on cleanup_after to check that the ptr is freed */
}

K_MEM_SLAB_DEFINE_STATIC(test_slabs, 4, 1, 1);
/**
 * @brief Verify that scope_defer(k_mem_slab_free) frees a memory slab block at
 * the end of the scope.
 *
 * @details
 * The test uses a static memory slab with one block. k_mem_slab_num_used_get()
 * gives the number of blocks in use.
 *
 * Test steps:
 * - Read the number of used blocks.
 * - In an inner scope, allocate a block with k_mem_slab_alloc() and K_NO_WAIT.
 * - Declare scope_defer(k_mem_slab_free) on the block. Then read the number of
 *   used blocks.
 * - Leave the inner scope. Then read the number of used blocks.
 *
 * Expected result:
 * - Before the scope, 0 blocks are in use.
 * - k_mem_slab_alloc() returns 0.
 * - In the scope, 1 block is in use.
 * - After the scope, 0 blocks are in use.
 *
 * @see scope_defer(), k_mem_slab_alloc(), k_mem_slab_free()
 */
ZTEST(cleanup_api, test_defer_k_mem_slab_free)
{
	void *ptr;
	int ret;

	zexpect_equal(k_mem_slab_num_used_get(&test_slabs), 0);

	{
		ret = k_mem_slab_alloc(&test_slabs, &ptr, K_NO_WAIT);
		zassert_ok(ret);
		scope_defer(k_mem_slab_free)(&test_slabs, ptr);

		zexpect_equal(k_mem_slab_num_used_get(&test_slabs), 1);
	}

	zexpect_equal(k_mem_slab_num_used_get(&test_slabs), 0);
}

static bool void_function_called;
static void void_function(void)
{
	void_function_called = true;
}
SCOPE_DEFER_DEFINE(void_function);

/**
 * @brief Verify that scope_defer() calls a function without arguments at the
 * end of the scope.
 *
 * @details
 * SCOPE_DEFER_DEFINE() defines a deferred call for void_function(), which has
 * no arguments. void_function() sets a flag.
 *
 * Test steps:
 * - In an inner scope, declare scope_defer(void_function). Then read the flag.
 * - Leave the inner scope. Then read the flag.
 *
 * Expected result:
 * - In the scope, the flag is false.
 * - After the scope, the flag is true.
 *
 * @see scope_defer(), SCOPE_DEFER_DEFINE()
 */
ZTEST(cleanup_api, test_defer_void_function)
{
	{
		scope_defer(void_function)();

		zexpect_false(void_function_called);
	}

	zexpect_true(void_function_called);
}

struct foo {
	uint8_t *const buf;
	const size_t buf_len;
};

static inline struct foo foo_constructor(size_t len)
{
	return (struct foo){
		.buf = k_malloc(len),
		.buf_len = len,
	};
}

static inline void foo_destructor(struct foo f)
{
	k_free(f.buf);
}

SCOPE_VAR_DEFINE(foo, struct foo, foo_destructor(_T), foo_constructor(len), size_t len);

/**
 * @brief Verify that scope_var() with a custom SCOPE_VAR_DEFINE() helper calls
 * its constructor and its destructor.
 *
 * @details
 * The foo helper allocates a buffer with k_malloc() in its constructor. Its
 * destructor frees the buffer with k_free(). The suite after function
 * cleanup_after() compares the free bytes of the system heap with the value
 * that cleanup_setup() stored. A difference fails the test as a memory leak.
 *
 * Test steps:
 * - Declare a foo variable with scope_var() and the length 42.
 * - Read the buffer pointer and the length of the variable.
 * - Return from the test function.
 *
 * Expected result:
 * - The buffer pointer is not NULL.
 * - The length is 42.
 * - After the test, the system heap has the same number of free bytes as before
 *   the suite.
 *
 * @see scope_var(), SCOPE_VAR_DEFINE()
 */
ZTEST(cleanup_api, test_custom_cleanup_helper)
{
	scope_var(foo, f)(42);

	zexpect_not_null(f.buf);
	zexpect_equal(f.buf_len, 42);

	/* Rely on cleanup_after to check that f is destructed */
}

ZTEST_SUITE(cleanup_api, NULL, cleanup_setup, NULL, cleanup_after, NULL);
