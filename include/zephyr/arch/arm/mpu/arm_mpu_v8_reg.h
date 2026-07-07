/*
 * Copyright (c) 2018 Linaro Limited.
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 * Copyright 2025 Arm Limited and/or its affiliates <open-source-office@arm.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_MPU_ARM_MPU_V8_REG_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_MPU_ARM_MPU_V8_REG_H_

/*
 * ARMv8-M (PMSAv8, Cortex-M profile) MPU RBAR/RLAR register-field positions and
 * masks.
 *
 * These are compile-time constants describing the ARMv8-M PMSAv8 MPU register
 * layout, as specified in the "ARMv8-M Architecture Reference Manual"
 * (ARM DDI 0553), for the MPU_RBAR and MPU_RLAR registers.
 *
 * The values are kept textually identical to the corresponding CMSIS
 * definitions (core_cm23.h / core_cm33.h / core_cm55.h / core_cm85.h) on
 * purpose: this header lets arm_mpu_v8.h build its access-permission and
 * share-ability convenience macros WITHOUT pulling the vendor CMSIS core
 * headers into the (arch-portable) kernel include graph, while a translation
 * unit that legitimately includes both this header and <cmsis_core.h> (e.g.
 * arch/SoC MPU driver code) sees only benign, identical macro redefinitions.
 *
 * Note: these are the Cortex-M PMSAv8 encodings; the ARMv8-R PMSAv8-64 layout
 * differs and is defined separately in arm_mpu_v8.h under CONFIG_AARCH32_ARMV8_R.
 *
 * Do not "clean up" the spelling (suffixes, hex casing, the octal 01UL, the
 * self-referential shifts): it must match CMSIS token-for-token to stay a
 * benign redefinition.
 */

/* MPU Region Base Address Register (RBAR) field positions/masks. */
#define MPU_RBAR_BASE_Pos                   5U
#define MPU_RBAR_BASE_Msk                  (0x7FFFFFFUL << MPU_RBAR_BASE_Pos)

#define MPU_RBAR_SH_Pos                     3U
#define MPU_RBAR_SH_Msk                    (0x3UL << MPU_RBAR_SH_Pos)

#define MPU_RBAR_AP_Pos                     1U
#define MPU_RBAR_AP_Msk                    (0x3UL << MPU_RBAR_AP_Pos)

#define MPU_RBAR_XN_Pos                     0U
#define MPU_RBAR_XN_Msk                    (01UL /*<< MPU_RBAR_XN_Pos*/)

/* MPU Region Limit Address Register (RLAR) field positions/masks. */
#define MPU_RLAR_LIMIT_Pos                  5U
#define MPU_RLAR_LIMIT_Msk                 (0x7FFFFFFUL << MPU_RLAR_LIMIT_Pos)

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_MPU_ARM_MPU_V8_REG_H_ */
