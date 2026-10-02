/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/irq_multilevel.h>
#include <zephyr/ztest.h>

#define TEST_L1_IRQ 3U
#define TEST_L2_IRQ 5U

#define TEST_ENCODED_L2_IRQ (TEST_L1_IRQ | IRQ_TO_L2(TEST_L2_IRQ))

/**
 * @brief Verify that irq_increment() increments a level 1 IRQ number.
 *
 * @details
 * A level 1 IRQ number has no bits in the fields of the higher levels. The test
 * uses the level 1 IRQ number 3.
 *
 * Test steps:
 * - Call irq_increment() with the IRQ number 3 and the value 1.
 *
 * Expected result:
 * - irq_increment() returns 4.
 *
 * @see irq_increment()
 */
ZTEST(irq_multilevel, test_level_1_increment)
{
	const unsigned int irq = TEST_L1_IRQ;
	const unsigned int expected = TEST_L1_IRQ + 1U;
	const unsigned int actual = irq_increment(irq, 1U);

	zassert_equal(actual, expected, "irq_increment(%u, 1) returned 0x%x, expected 0x%x", irq,
		      actual, expected);
}

/**
 * @brief Verify that irq_to_level_2() gives the same value as the IRQ_TO_L2()
 * macro.
 *
 * @details
 * Both irq_to_level_2() and IRQ_TO_L2() put a level 2 IRQ number into the level
 * 2 bit field. The function and the macro must give the same encoding.
 *
 * Test steps:
 * - Call irq_to_level_2() with the level 2 IRQ number 5.
 * - Compare the result with the value of IRQ_TO_L2() for the same number.
 *
 * Expected result:
 * - The two values are equal.
 *
 * @see irq_to_level_2(), IRQ_TO_L2()
 */
ZTEST(irq_multilevel, test_level_2_encoding_matches_macro)
{
	const unsigned int expected = IRQ_TO_L2(TEST_L2_IRQ);
	const unsigned int actual = irq_to_level_2(TEST_L2_IRQ);

	zassert_equal(actual, expected,
		      "irq_to_level_2(%u) returned 0x%x, "
		      "but IRQ_TO_L2(%u) returned 0x%x",
		      TEST_L2_IRQ, actual, TEST_L2_IRQ, expected);
}

/**
 * @brief Verify that irq_get_level() returns 2 for an encoded level 2 IRQ
 * number.
 *
 * @details
 * The encoded IRQ number contains the level 1 IRQ number 3 and the level 2 IRQ
 * number 5.
 *
 * Test steps:
 * - Call irq_get_level() with the encoded level 2 IRQ number.
 *
 * Expected result:
 * - irq_get_level() returns 2.
 *
 * @see irq_get_level()
 */
ZTEST(irq_multilevel, test_level_2_get_level)
{
	const unsigned int irq = TEST_ENCODED_L2_IRQ;
	const unsigned int actual = irq_get_level(irq);

	zassert_equal(actual, 2U, "irq_get_level(0x%x) returned %u", irq, actual);
}

/**
 * @brief Verify that irq_from_level_2() gets the level 2 IRQ number from an
 * encoded IRQ number.
 *
 * @details
 * The encoded IRQ number contains the level 1 IRQ number 3 and the level 2 IRQ
 * number 5.
 *
 * Test steps:
 * - Call irq_from_level_2() with the encoded level 2 IRQ number.
 *
 * Expected result:
 * - irq_from_level_2() returns 5.
 *
 * @see irq_from_level_2()
 */
ZTEST(irq_multilevel, test_level_2_decode)
{
	const unsigned int irq = TEST_ENCODED_L2_IRQ;
	const unsigned int actual = irq_from_level_2(irq);

	zassert_equal(actual, TEST_L2_IRQ, "irq_from_level_2(0x%x) returned %u", irq, actual);
}

/**
 * @brief Verify that irq_parent_level_2() gets the level 1 parent IRQ number
 * from an encoded level 2 IRQ number.
 *
 * @details
 * The encoded IRQ number contains the level 1 IRQ number 3 and the level 2 IRQ
 * number 5.
 *
 * Test steps:
 * - Call irq_parent_level_2() with the encoded level 2 IRQ number.
 *
 * Expected result:
 * - irq_parent_level_2() returns 3.
 *
 * @see irq_parent_level_2()
 */
ZTEST(irq_multilevel, test_level_2_parent)
{
	const unsigned int irq = TEST_ENCODED_L2_IRQ;
	const unsigned int actual = irq_parent_level_2(irq);

	zassert_equal(actual, TEST_L1_IRQ, "irq_parent_level_2(0x%x) returned %u", irq, actual);
}

/**
 * @brief Verify that irq_get_intc_irq() returns the IRQ number of the parent
 * interrupt controller for a level 2 IRQ number.
 *
 * @details
 * The encoded IRQ number contains the level 1 IRQ number 3 and the level 2 IRQ
 * number 5. The parent interrupt controller of a level 2 IRQ uses the level 1
 * IRQ number.
 *
 * Test steps:
 * - Call irq_get_intc_irq() with the encoded level 2 IRQ number.
 *
 * Expected result:
 * - irq_get_intc_irq() returns 3.
 *
 * @see irq_get_intc_irq()
 */
ZTEST(irq_multilevel, test_level_2_intc_irq)
{
	const unsigned int irq = TEST_ENCODED_L2_IRQ;
	const unsigned int actual = irq_get_intc_irq(irq);

	zassert_equal(actual, TEST_L1_IRQ, "irq_get_intc_irq(0x%x) returned 0x%x", irq, actual);
}

/**
 * @brief Verify that irq_increment() increments the level 2 part of an encoded
 * IRQ number.
 *
 * @details
 * The encoded IRQ number contains the level 1 IRQ number 3 and the level 2 IRQ
 * number 5. irq_increment() must change only the IRQ number of the highest
 * level.
 *
 * Test steps:
 * - Call irq_increment() with the encoded level 2 IRQ number and the value 1.
 *
 * Expected result:
 * - The result contains the level 1 IRQ number 3 and the level 2 IRQ number 6.
 *
 * @see irq_increment()
 */
ZTEST(irq_multilevel, test_level_2_increment)
{
	const unsigned int irq = TEST_ENCODED_L2_IRQ;
	const unsigned int expected = TEST_L1_IRQ | IRQ_TO_L2(TEST_L2_IRQ + 1U);
	const unsigned int actual = irq_increment(irq, 1U);

	zassert_equal(actual, expected, "irq_increment(0x%x, 1) returned 0x%x, expected 0x%x", irq,
		      actual, expected);
}

#if defined(CONFIG_3RD_LEVEL_INTERRUPTS)
#define TEST_L3_IRQ 7U

#define TEST_ENCODED_L3_IRQ (TEST_L1_IRQ | IRQ_TO_L2(TEST_L2_IRQ) | IRQ_TO_L3(TEST_L3_IRQ))

#define TEST_ENCODED_L3_PARENT (TEST_L1_IRQ | IRQ_TO_L2(TEST_L2_IRQ))

/**
 * @brief Verify that irq_to_level_3() gives the same value as the IRQ_TO_L3()
 * macro.
 *
 * @details
 * Both irq_to_level_3() and IRQ_TO_L3() put a level 3 IRQ number into the level
 * 3 bit field. The build contains this test only if CONFIG_3RD_LEVEL_INTERRUPTS
 * is enabled.
 *
 * Test steps:
 * - Call irq_to_level_3() with the level 3 IRQ number 7.
 * - Compare the result with the value of IRQ_TO_L3() for the same number.
 *
 * Expected result:
 * - The two values are equal.
 *
 * @see irq_to_level_3(), IRQ_TO_L3()
 */
ZTEST(irq_multilevel, test_level_3_encoding_matches_macro)
{
	const unsigned int expected = IRQ_TO_L3(TEST_L3_IRQ);
	const unsigned int actual = irq_to_level_3(TEST_L3_IRQ);

	zassert_equal(actual, expected,
		      "irq_to_level_3(%u) returned 0x%x, "
		      "but IRQ_TO_L3(%u) returned 0x%x",
		      TEST_L3_IRQ, actual, TEST_L3_IRQ, expected);
}

/**
 * @brief Verify that irq_get_level() returns 3 for an encoded level 3 IRQ
 * number.
 *
 * @details
 * The encoded IRQ number contains the level 1 IRQ number 3, the level 2 IRQ
 * number 5 and the level 3 IRQ number 7. The build contains this test only if
 * CONFIG_3RD_LEVEL_INTERRUPTS is enabled.
 *
 * Test steps:
 * - Call irq_get_level() with the encoded level 3 IRQ number.
 *
 * Expected result:
 * - irq_get_level() returns 3.
 *
 * @see irq_get_level()
 */
ZTEST(irq_multilevel, test_level_3_get_level)
{
	const unsigned int irq = TEST_ENCODED_L3_IRQ;
	const unsigned int actual = irq_get_level(irq);

	zassert_equal(actual, 3U, "irq_get_level(0x%x) returned %u", irq, actual);
}

/**
 * @brief Verify that irq_from_level_3() gets the level 3 IRQ number from an
 * encoded IRQ number.
 *
 * @details
 * The encoded IRQ number contains the level 1 IRQ number 3, the level 2 IRQ
 * number 5 and the level 3 IRQ number 7. The build contains this test only if
 * CONFIG_3RD_LEVEL_INTERRUPTS is enabled.
 *
 * Test steps:
 * - Call irq_from_level_3() with the encoded level 3 IRQ number.
 *
 * Expected result:
 * - irq_from_level_3() returns 7.
 *
 * @see irq_from_level_3()
 */
ZTEST(irq_multilevel, test_level_3_decode)
{
	const unsigned int irq = TEST_ENCODED_L3_IRQ;
	const unsigned int actual = irq_from_level_3(irq);

	zassert_equal(actual, TEST_L3_IRQ, "irq_from_level_3(0x%x) returned %u", irq, actual);
}

/**
 * @brief Verify that irq_parent_level_3() gets the level 2 parent IRQ number
 * from an encoded level 3 IRQ number.
 *
 * @details
 * The encoded IRQ number contains the level 1 IRQ number 3, the level 2 IRQ
 * number 5 and the level 3 IRQ number 7. The build contains this test only if
 * CONFIG_3RD_LEVEL_INTERRUPTS is enabled.
 *
 * Test steps:
 * - Call irq_parent_level_3() with the encoded level 3 IRQ number.
 *
 * Expected result:
 * - irq_parent_level_3() returns 5.
 *
 * @see irq_parent_level_3()
 */
ZTEST(irq_multilevel, test_level_3_parent)
{
	const unsigned int irq = TEST_ENCODED_L3_IRQ;
	const unsigned int actual = irq_parent_level_3(irq);

	zassert_equal(actual, TEST_L2_IRQ, "irq_parent_level_3(0x%x) returned %u", irq, actual);
}

/**
 * @brief Verify that irq_get_intc_irq() returns the encoded level 2 parent IRQ
 * number for a level 3 IRQ number.
 *
 * @details
 * The encoded IRQ number contains the level 1 IRQ number 3, the level 2 IRQ
 * number 5 and the level 3 IRQ number 7. The build contains this test only if
 * CONFIG_3RD_LEVEL_INTERRUPTS is enabled. The parent interrupt controller of a
 * level 3 IRQ uses the encoded level 2 IRQ number.
 *
 * Test steps:
 * - Call irq_get_intc_irq() with the encoded level 3 IRQ number.
 *
 * Expected result:
 * - irq_get_intc_irq() returns the encoded IRQ number with the level 1 IRQ
 *   number 3 and the level 2 IRQ number 5.
 *
 * @see irq_get_intc_irq()
 */
ZTEST(irq_multilevel, test_level_3_intc_irq)
{
	const unsigned int irq = TEST_ENCODED_L3_IRQ;
	const unsigned int actual = irq_get_intc_irq(irq);

	zassert_equal(actual, TEST_ENCODED_L3_PARENT,
		      "irq_get_intc_irq(0x%x) returned 0x%x, expected 0x%x", irq, actual,
		      TEST_ENCODED_L3_PARENT);
}

/**
 * @brief Verify that irq_increment() increments the level 3 part of an encoded
 * IRQ number.
 *
 * @details
 * The encoded IRQ number contains the level 1 IRQ number 3, the level 2 IRQ
 * number 5 and the level 3 IRQ number 7. The build contains this test only if
 * CONFIG_3RD_LEVEL_INTERRUPTS is enabled.
 *
 * Test steps:
 * - Call irq_increment() with the encoded level 3 IRQ number and the value 1.
 *
 * Expected result:
 * - The result contains the level 1 IRQ number 3, the level 2 IRQ number 5 and
 *   the level 3 IRQ number 8.
 *
 * @see irq_increment()
 */
ZTEST(irq_multilevel, test_level_3_increment)
{
	const unsigned int irq = TEST_ENCODED_L3_IRQ;
	const unsigned int expected =
		TEST_L1_IRQ | IRQ_TO_L2(TEST_L2_IRQ) | IRQ_TO_L3(TEST_L3_IRQ + 1U);
	const unsigned int actual = irq_increment(irq, 1U);

	zassert_equal(actual, expected, "irq_increment(0x%x, 1) returned 0x%x, expected 0x%x", irq,
		      actual, expected);
}

#endif /* CONFIG_3RD_LEVEL_INTERRUPTS */

ZTEST_SUITE(irq_multilevel, NULL, NULL, NULL, NULL, NULL);
