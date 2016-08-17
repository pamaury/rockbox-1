/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * This file was automatically generated by headergen, DO NOT EDIT it.
 * headergen version: 3.0.0
 *
 * Copyright (C) 2015 by the authors
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#ifndef __HEADERGEN_MACRO_H__
#define __HEADERGEN_MACRO_H__

#define __VAR_OR1(prefix, suffix) \
    (prefix##suffix)
#define __VAR_OR2(pre, s1, s2) \
    (__VAR_OR1(pre, s1) | __VAR_OR1(pre, s2))
#define __VAR_OR3(pre, s1, s2, s3) \
    (__VAR_OR1(pre, s1) | __VAR_OR2(pre, s2, s3))
#define __VAR_OR4(pre, s1, s2, s3, s4) \
    (__VAR_OR2(pre, s1, s2) | __VAR_OR2(pre, s3, s4))
#define __VAR_OR5(pre, s1, s2, s3, s4, s5) \
    (__VAR_OR2(pre, s1, s2) | __VAR_OR3(pre, s3, s4, s5))
#define __VAR_OR6(pre, s1, s2, s3, s4, s5, s6) \
    (__VAR_OR3(pre, s1, s2, s3) | __VAR_OR3(pre, s4, s5, s6))
#define __VAR_OR7(pre, s1, s2, s3, s4, s5, s6, s7) \
    (__VAR_OR3(pre, s1, s2, s3) | __VAR_OR4(pre, s4, s5, s6, s7))
#define __VAR_OR8(pre, s1, s2, s3, s4, s5, s6, s7, s8) \
    (__VAR_OR4(pre, s1, s2, s3, s4) | __VAR_OR4(pre, s5, s6, s7, s8))
#define __VAR_OR9(pre, s1, s2, s3, s4, s5, s6, s7, s8, s9) \
    (__VAR_OR4(pre, s1, s2, s3, s4) | __VAR_OR5(pre, s5, s6, s7, s8, s9))
#define __VAR_OR10(pre, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10) \
    (__VAR_OR5(pre, s1, s2, s3, s4, s5) | __VAR_OR5(pre, s6, s7, s8, s9, s10))
#define __VAR_OR11(pre, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11) \
    (__VAR_OR5(pre, s1, s2, s3, s4, s5) | __VAR_OR6(pre, s6, s7, s8, s9, s10, s11))
#define __VAR_OR12(pre, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12) \
    (__VAR_OR6(pre, s1, s2, s3, s4, s5, s6) | __VAR_OR6(pre, s7, s8, s9, s10, s11, s12))
#define __VAR_OR13(pre, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13) \
    (__VAR_OR6(pre, s1, s2, s3, s4, s5, s6) | __VAR_OR7(pre, s7, s8, s9, s10, s11, s12, s13))

#define __VAR_NARGS(...) __VAR_NARGS_(__VA_ARGS__, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1)
#define __VAR_NARGS_(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, N, ...) N

#define __VAR_EXPAND(macro, prefix, ...) __VAR_EXPAND_(macro, __VAR_NARGS(__VA_ARGS__), prefix, __VA_ARGS__)
#define __VAR_EXPAND_(macro, cnt, prefix, ...) __VAR_EXPAND__(macro, cnt, prefix, __VA_ARGS__)
#define __VAR_EXPAND__(macro, cnt, prefix, ...) __VAR_EXPAND___(macro##cnt, prefix, __VA_ARGS__)
#define __VAR_EXPAND___(macro, prefix, ...) macro(prefix, __VA_ARGS__)

#define JIO_8_RO(op, name, ...)         JIO_8_RO_##op(name, __VA_ARGS__)
#define JIO_8_RO_RD(name, ...)          (*(const volatile uint8_t *)(JA_##name))
#define JIO_8_RO_WR(name, val)          _Static_assert(0, #name " is read-only")
#define JIO_8_RO_RMW(name, vand, vor)   _Static_assert(0, #name " is read-only")
#define JIO_8_RO_VAR(name, ...)         (*(const volatile uint8_t *)(JA_##name))

#define JIO_16_RO(op, name, ...)        JIO_16_RO_##op(name, __VA_ARGS__)
#define JIO_16_RO_RD(name, ...)         (*(const volatile uint16_t *)(JA_##name))
#define JIO_16_RO_WR(name, val)         _Static_assert(0, #name " is read-only")
#define JIO_16_RO_RMW(name, vand, vor)  _Static_assert(0, #name " is read-only")
#define JIO_16_RO_VAR(name, ...)        (*(const volatile uint16_t *)(JA_##name))

#define JIO_32_RO(op, name, ...)        JIO_32_RO_##op(name, __VA_ARGS__)
#define JIO_32_RO_RD(name, ...)         (*(const volatile uint32_t *)(JA_##name))
#define JIO_32_RO_WR(name, val)         _Static_assert(0, #name " is read-only")
#define JIO_32_RO_RMW(name, vand, vor)  _Static_assert(0, #name " is read-only")
#define JIO_32_RO_VAR(name, ...)        (*(const volatile uint32_t *)(JA_##name))

#define JIO_8_RW(op, name, ...)         JIO_8_RW_##op(name, __VA_ARGS__)
#define JIO_8_RW_RD(name, ...)          (*(volatile uint8_t *)(JA_##name))
#define JIO_8_RW_WR(name, val)          (*(volatile uint8_t *)(JA_##name)) = (val)
#define JIO_8_RW_RMW(name, vand, vor)   JIO_8_RW_WR(name, (JIO_8_RW_RD(name) & (vand)) | (vor))
#define JIO_8_RW_VAR(name, ...)         (*(volatile uint8_t *)(JA_##name))

#define JIO_16_RW(op, name, ...)        JIO_16_RW_##op(name, __VA_ARGS__)
#define JIO_16_RW_RD(name, ...)         (*(volatile uint16_t *)(JA_##name))
#define JIO_16_RW_WR(name, val)         (*(volatile uint16_t *)(JA_##name)) = (val)
#define JIO_16_RW_RMW(name, vand, vor)  JIO_16_RW_WR(name, (JIO_16_RW_RD(name) & (vand)) | (vor))
#define JIO_16_RW_VAR(name, ...)        (*(volatile uint16_t *)(JA_##name))

#define JIO_32_RW(op, name, ...)        JIO_32_RW_##op(name, __VA_ARGS__)
#define JIO_32_RW_RD(name, ...)         (*(volatile uint32_t *)(JA_##name))
#define JIO_32_RW_WR(name, val)         (*(volatile uint32_t *)(JA_##name)) = (val)
#define JIO_32_RW_RMW(name, vand, vor)  JIO_32_RW_WR(name, (JIO_32_RW_RD(name) & (vand)) | (vor))
#define JIO_32_RW_VAR(name, ...)        (*(volatile uint32_t *)(JA_##name))

#define JIO_8_WO(op, name, ...)         JIO_8_WO_##op(name, __VA_ARGS__)
#define JIO_8_WO_RD(name, ...)          ({_Static_assert(0, #name " is write-only"); 0;})
#define JIO_8_WO_WR(name, val)          (*(volatile uint8_t *)(JA_##name)) = (val)
#define JIO_8_WO_RMW(name, vand, vor)   JIO_8_WO_WR(name, vor)
#define JIO_8_WO_VAR(name, ...)         (*(volatile uint8_t *)(JA_##name))

#define JIO_16_WO(op, name, ...)        JIO_16_WO_##op(name, __VA_ARGS__)
#define JIO_16_WO_RD(name, ...)         ({_Static_assert(0, #name " is write-only"); 0;})
#define JIO_16_WO_WR(name, val)         (*(volatile uint16_t *)(JA_##name)) = (val)
#define JIO_16_WO_RMW(name, vand, vor)  JIO_16_WO_WR(name, vor)
#define JIO_16_WO_VAR(name, ...)        (*(volatile uint16_t *)(JA_##name))

#define JIO_32_WO(op, name, ...)        JIO_32_WO_##op(name, __VA_ARGS__)
#define JIO_32_WO_RD(name, ...)         ({_Static_assert(0, #name " is write-only"); 0;})
#define JIO_32_WO_WR(name, val)         (*(volatile uint32_t *)(JA_##name)) = (val)
#define JIO_32_WO_RMW(name, vand, vor)  JIO_32_WO_WR(name, vor)
#define JIO_32_WO_VAR(name, ...)        (*(volatile uint32_t *)(JA_##name))


/** __jz_variant
 *
 * usage: __jz_variant(register, variant_prefix, variant_postfix)
 *
 * effect: expands to register variant given as argument
 * note: internal usage
 * note: register must be fully qualified if indexed
 *
 * example: __jz_variant(ICOLL_CTRL, , _SET)
 * example: __jz_variant(ICOLL_ENABLE(3), , _CLR)
 */
#define __jz_variant(name, varp, vars) __jz_variant_(JN_##name, JI_##name, varp, vars)
#define __jz_variant_(...) __jz_variant__(__VA_ARGS__)
#define __jz_variant__(name, index, varp, vars) varp##name##vars index

/** jz_orf
 *
 * usage: jz_orf(register, f1(v1), f2(v2), ...)
 *
 * effect: expands to the register value where each field fi has value vi.
 *         Informally: reg_f1(v1) | reg_f2(v2) | ...
 * note: enumerated values for fields can be obtained by using the syntax:
 *          f1_V(name)
 *
 * example: jz_orf(ICOLL_CTRL, SFTRST(1), CLKGATE(0), TZ_LOCK_V(UNLOCKED))
 */
#define jz_orf(reg, ...) __VAR_EXPAND(__VAR_OR, BF_##reg##_, __VA_ARGS__)

/** __jz_orfm
 *
 * usage: __jz_orfm(register, f1(v1), f2(v2), ...)
 *
 * effect: expands to the register value where each field fi has maximum value (vi is ignored).
 * note: internal usage
 *
 * example: __jz_orfm(ICOLL_CTRL, SFTRST(1), CLKGATE(0), TZ_LOCK_V(UNLOCKED))
 */
#define __jz_orfm(reg, ...) __VAR_EXPAND(__VAR_OR, BFM_##reg##_, __VA_ARGS__)

/** jz_orm
 *
 * usage: jz_orm(register, f1, f2, ...)
 *
 * effect: expands to the register value where each field fi is set to its maximum value.
 *         Informally: reg_f1_mask | reg_f2_mask | ...
 *
 * example: jz_orm(ICOLL_CTRL, SFTRST, CLKGATE)
 */
#define jz_orm(reg, ...) __VAR_EXPAND(__VAR_OR, BM_##reg##_, __VA_ARGS__)


/** jz_read
 *
 * usage: jz_read(register)
 *
 * effect: read a register and return its value
 * note: register must be fully qualified if indexed
 *
 * example: jz_read(ICOLL_STATUS)
 *          jz_read(ICOLL_ENABLE(42))
 */
#define jz_read(name) JT_##name(RD, name)

/** jz_vreadf
 *
 * usage: jz_vreadf(value, register, field)
 *
 * effect: given a register value, return the value of a particular field
 * note: this macro does NOT read any register
 *
 * example: jz_vreadf(0xc0000000, ICOLL_CTRL, SFTRST)
 *          jz_vreadf(0x46ff, ICOLL_ENABLE, CPU0_PRIO)
 */
#define jz_vreadf(val, name, field) (((val) & BM_##name##_##field) >> BP_##name##_##field)

/** jz_readf
 *
 * usage: jz_readf(register, field)
 *
 * effect: read a register and return the value of a particular field
 * note: register must be fully qualified if indexed
 *
 * example: jz_readf(ICOLL_CTRL, SFTRST)
 *          jz_readf(ICOLL_ENABLE(3), CPU0_PRIO)
 */
#define jz_readf(name, field) jz_readf_(jz_read(name), JN_##name, field)
#define jz_readf_(...) jz_vreadf(__VA_ARGS__)

/** jz_write
 *
 * usage: jz_write(register, value)
 *
 * effect: write a register
 * note: register must be fully qualified if indexed
 *
 * example: jz_write(ICOLL_CTRL, 0x42)
 *          jz_write(ICOLL_ENABLE_SET(3), 0x37)
 */
#define jz_write(name, val) JT_##name(WR, name, val)

/** jz_writef
 *
 * usage: jz_writef(register, f1(v1), f2(v2), ...)
 *
 * effect: change the register value so that field fi has value vi
 * note: register must be fully qualified if indexed
 * note: this macro may perform a read-modify-write
 *
 * example: jz_writef(ICOLL_CTRL, SFTRST(1), CLKGATE(0), TZ_LOCK_V(UNLOCKED))
 *          jz_writef(ICOLL_ENABLE(3), CPU0_PRIO(1), CPU0_TYPE_V(FIQ))
 */
#define jz_writef(name, ...) jz_writef_(name, JN_##name, __VA_ARGS__)
#define jz_writef_(name, name2, ...) JT_##name(RMW, name, ~__jz_orfm(name2, __VA_ARGS__), jz_orf(name2, __VA_ARGS__))

/** jz_overwritef
 *
 * usage: jz_overwritef(register, f1(v1), f2(v2), ...)
 *
 * effect: change the register value so that field fi has value vi and other fields have value zero
 *         thus this macro is equivalent to:
 *         jz_write(register, jz_orf(register, f1(v1), ...))
 * note: register must be fully qualified if indexed
 * note: this macro will overwrite the register (it is NOT a read-modify-write)
 *
 * example: jz_overwritef(ICOLL_CTRL, SFTRST(1), CLKGATE(0), TZ_LOCK_V(UNLOCKED))
 *          jz_overwritef(ICOLL_ENABLE(3), CPU0_PRIO(1), CPU0_TYPE_V(FIQ))
 */
#define jz_overwritef(name, ...) jz_overwritef_(name, JN_##name, __VA_ARGS__)
#define jz_overwritef_(name, name2, ...) JT_##name(WR, name, jz_orf(name2, __VA_ARGS__))

/** jz_vwritef
 *
 * usage: jz_vwritef(var, register, f1(v1), f2(v2), ...)
 *
 * effect: change the variable value so that field fi has value vi
 * note: this macro will perform a read-modify-write
 *
 * example: jz_vwritef(var, ICOLL_CTRL, SFTRST(1), CLKGATE(0), TZ_LOCK_V(UNLOCKED))
 *          jz_vwritef(var, ICOLL_ENABLE, CPU0_PRIO(1), CPU0_TYPE_V(FIQ))
 */
#define jz_vwritef(var, name, ...) (var) = jz_orf(name, __VA_ARGS__) | (~__jz_orfm(name, __VA_ARGS__) & (var))

/** jz_setf
 *
 * usage: jz_setf(register, f1, f2, ...)
 *
 * effect: change the register value so that field fi has maximum value
 * IMPORTANT: this macro performs a write to the set variant of the register
 * note: register must be fully qualified if indexed
 *
 * example: jz_setf(ICOLL_CTRL, SFTRST, CLKGATE)
 *          jz_setf(ICOLL_ENABLE(3), CPU0_PRIO, CPU0_TYPE)
 */
#define jz_setf(name, ...) jz_setf_(__jz_variant(name, , _SET), JN_##name, __VA_ARGS__)
#define jz_setf_(name, name2, ...) jz_write(name, jz_orm(name2, __VA_ARGS__))

/** jz_clrf
 *
 * usage: jz_clrf(register, f1, f2, ...)
 *
 * effect: change the register value so that field fi has value zero
 * IMPORTANT: this macro performs a write to the clr variant of the register
 * note: register must be fully qualified if indexed
 *
 * example: jz_clrf(ICOLL_CTRL, SFTRST, CLKGATE)
 *          jz_clrf(ICOLL_ENABLE(3), CPU0_PRIO, CPU0_TYPE)
 */
#define jz_clrf(name, ...) jz_clrf_(__jz_variant(name, , _CLR), JN_##name, __VA_ARGS__)
#define jz_clrf_(name, name2, ...) jz_write(name, jz_orm(name2, __VA_ARGS__))

/** jz_set
 *
 * usage: jz_set(register, set_value)
 *
 * effect: set some bits using set variant
 * note: register must be fully qualified if indexed
 *
 * example: jz_set(ICOLL_CTRL, 0x42)
 *          jz_set(ICOLL_ENABLE(3), 0x37)
 */
#define jz_set(name, sval) jz_set_(__jz_variant(name, , _SET), sval)
#define jz_set_(sname, sval) jz_write(sname, sval)

/** jz_clr
 *
 * usage: jz_clr(register, clr_value)
 *
 * effect: clear some bits using clr variant
 * note: register must be fully qualified if indexed
 *
 * example: jz_clr(ICOLL_CTRL, 0x42)
 *          jz_clr(ICOLL_ENABLE(3), 0x37)
 */
#define jz_clr(name, cval) jz_clr_(__jz_variant(name, , _CLR), cval)
#define jz_clr_(cname, cval) jz_write(cname, cval)

/** jz_cs
 *
 * usage: jz_cs(register, clear_value, set_value)
 *
 * effect: clear some bits using clr variant and then set some using set variant
 * note: register must be fully qualified if indexed
 *
 * example: jz_cs(ICOLL_CTRL, 0xff, 0x42)
 *          jz_cs(ICOLL_ENABLE(3), 0xff, 0x37)
 */
#define jz_cs(name, cval, sval) jz_cs_(__jz_variant(name, , _CLR), __jz_variant(name, , _SET), cval, sval)
#define jz_cs_(cname, sname, cval, sval) do { jz_write(cname, cval); jz_write(sname, sval); } while(0)

/** jz_csf
 *
 * usage: jz_csf(register, f1(v1), f2(v2), ...)
 *
 * effect: change the register value so that field fi has value vi using clr and set variants
 * note: register must be fully qualified if indexed
 * note: this macro will NOT perform a read-modify-write and is thus safer
 * IMPORTANT: this macro will set some fields to 0 temporarily, make sure this is acceptable
 *
 * example: jz_csf(ICOLL_CTRL, SFTRST(1), CLKGATE(0), TZ_LOCK_V(UNLOCKED))
 *          jz_csf(ICOLL_ENABLE(3), CPU0_PRIO(1), CPU0_TYPE_V(FIQ))
 */
#define jz_csf(name, ...) jz_csf_(name, JN_##name, __VA_ARGS__)
#define jz_csf_(name, name2, ...) jz_cs(name, __jz_orfm(name2, __VA_ARGS__), jz_orf(name2, __VA_ARGS__))

/** jz_reg
 *
 * usage: jz_reg(register)
 *
 * effect: return a variable-like expression that can be read/written
 * note: register must be fully qualified if indexed
 * note: read-only registers will yield a constant expression
 *
 * example: unsigned x = jz_reg(ICOLL_STATUS)
 *          unsigned x = jz_reg(ICOLL_ENABLE(42))
 *          jz_reg(ICOLL_ENABLE(42)) = 64
 */
#define jz_reg(name) JT_##name(VAR, name)


#endif /* __HEADERGEN_MACRO_H__*/
