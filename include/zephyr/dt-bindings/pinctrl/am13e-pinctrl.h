/*
 * SPDX-FileCopyrightText: 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AM13E_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AM13E_PINCTRL_H_

#define AM13E_PIN_FUNCTION_ANALOG (0x00000000)
#define AM13E_PIN_FUNCTION_GPIO   (0x00000001)
#define AM13E_PIN_FUNCTION_2      (0x00000002)
#define AM13E_PIN_FUNCTION_3      (0x00000003)
#define AM13E_PIN_FUNCTION_4      (0x00000004)
#define AM13E_PIN_FUNCTION_5      (0x00000005)
#define AM13E_PIN_FUNCTION_6      (0x00000006)
#define AM13E_PIN_FUNCTION_7      (0x00000007)
#define AM13E_PIN_FUNCTION_8      (0x00000008)
#define AM13E_PIN_FUNCTION_9      (0x00000009)
#define AM13E_PIN_FUNCTION_10     (0x0000000A)
#define AM13E_PIN_FUNCTION_11     (0x0000000B)
#define AM13E_PIN_FUNCTION_12     (0x0000000C)
#define AM13E_PIN_FUNCTION_13     (0x0000000D)
#define AM13E_PIN_FUNCTION_14     (0x0000000E)
#define AM13E_PIN_FUNCTION_15     (0x0000000F)
#define AM13E_PIN_FUNCTION_16     (0x00000010)

/* Creates a concatenation of the correct pin function based on the pin control
 * management register offset and the function suffix. AM13E's PINCMx array
 * is 0-based (no leading reserved word), so unlike MSPM0's MSP_PINMUX() this
 * does not subtract 1 from pincm.
 */
#define AM13E_PINMUX(pincm, function) ((pincm << 0x10) | function)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AM13E_PINCTRL_H_ */
