--
-- Sansa View
--
SANSAVIEW = {}

function SANSAVIEW.set_backlight(val)
    -- backlight is PD0
    PP.gpio.pin("D",0).enable(true)
    PP.gpio.pin("D",0).output_enable(true)
    PP.gpio.pin("D",0).write(val)
end

function SANSAVIEW.lcd_reset()
    PP.gpio.pin("B", 2).write(true)
    PP.gpio.pin("B", 2).write(false)
    PP.gpio.pin("B", 2).write(true)
end

function SANSAVIEW.lcd_send_msg(count, val)
    local clock = PP.gpio.pin("H", 6)
    local data = PP.gpio.pin("H", 4)
    local cs = PP.gpio.pin("H", 7)
    clock.write(true)
    cs.write(false)
    for i = count-1, 0, -1 do
        data.write(bit32.extract(val, i) == 1)
        clock.write(false)
        clock.write(true)
    end
    cs.write(true)
end

function SANSAVIEW.lcd_write_cmd(cmd)
    SANSAVIEW.lcd_send_msg(24, bit32.bor(0x700000, cmd))
end

function SANSAVIEW.lcd_write_data(data)
    SANSAVIEW.lcd_send_msg(24, bit32.bor(0x720000, data))
end

function SANSAVIEW.lcd_write_reg(cmd, data)
    SANSAVIEW.lcd_write_cmd(cmd)
    SANSAVIEW.lcd_write_data(data)
end

function SANSAVIEW.lcd_init()
    -- lcd reset
    PP.gpio.pin("B", 2).enable(true)
    PP.gpio.pin("B", 2).write(true)
    PP.gpio.pin("B", 2).output_enable(true)
    -- lcd type
    PP.gpio.pin("G", 3).enable(true)
    PP.gpio.pin("G", 3).output_enable(false)
    -- spi data
    PP.gpio.pin("H", 4).enable(true)
    PP.gpio.pin("H", 4).write(true)
    PP.gpio.pin("H", 4).output_enable(true)
    -- spi clock
    PP.gpio.pin("H", 6).enable(true)
    PP.gpio.pin("H", 6).write(true)
    PP.gpio.pin("H", 6).output_enable(true)
    -- spi cs
    PP.gpio.pin("H", 7).enable(true)
    PP.gpio.pin("H", 7).write(true)
    PP.gpio.pin("H", 7).output_enable(true)
    -- lcd unk
    PP.gpio.pin("J", 1).enable(false)
    PP.gpio.pin("J", 1).write(false)
    PP.gpio.pin("J", 1).output_enable(false)

    HW.SYS.DEV1.write(bit32.bor(HW.SYS.DEV1.read(),0xfc000000))
    HW.SYS.DEV3.write(bit32.bor(HW.SYS.DEV3.read(),0xc300000))
    HW.SYS.DEV2.write(0x40000000)

    SANSAVIEW.lcd_reset()
    SANSAVIEW.lcd_type = PP.gpio.pin("G", 3).read()
    print(string.format("sansaview: lcd type is %s", SANSAVIEW.lcd_type))

    SANSAVIEW.lcd_write_reg(0xE5, 0x8000)
    SANSAVIEW.lcd_write_reg(0x0, 0x1)
    SANSAVIEW.lcd_write_reg(0x1, 0x100)
    SANSAVIEW.lcd_write_reg(0x2, 0x700)
    SANSAVIEW.lcd_write_reg(0x3, 0x1230)
    SANSAVIEW.lcd_write_reg(0x4, 0x0)
    SANSAVIEW.lcd_write_reg(0x8, 0x408)
    SANSAVIEW.lcd_write_reg(0x9, 0x0)
    SANSAVIEW.lcd_write_reg(0xa, 0x0)
    SANSAVIEW.lcd_write_reg(0xd, 0x0)
    SANSAVIEW.lcd_write_reg(0xf, 0x2)
    SANSAVIEW.lcd_write_reg(0x10, 0x0)
    SANSAVIEW.lcd_write_reg(0x11, 0x0)
    SANSAVIEW.lcd_write_reg(0x12, 0x0)
    SANSAVIEW.lcd_write_reg(0x13, 0x0)
    SANSAVIEW.lcd_write_reg(0x10, 0x17B0)
    SANSAVIEW.lcd_write_reg(0x11, 0x7)
    SANSAVIEW.lcd_write_reg(0x12, 0x13c)

    if SANSAVIEW.lcd_type == 0 then
        SANSAVIEW.lcd_write_reg(0x13, 0x1700)
        SANSAVIEW.lcd_write_reg(0x29, 0x10)
        SANSAVIEW.lcd_write_reg(0x20, 0x0)
        SANSAVIEW.lcd_write_reg(0x21, 0x0)

        SANSAVIEW.lcd_write_reg(0x30, 0x7)
        SANSAVIEW.lcd_write_reg(0x31, 0x403)
        SANSAVIEW.lcd_write_reg(0x32, 0x400)
        SANSAVIEW.lcd_write_reg(0x35, 0x3)
        SANSAVIEW.lcd_write_reg(0x36, 0xF07)
        SANSAVIEW.lcd_write_reg(0x37, 0x606)
        SANSAVIEW.lcd_write_reg(0x38, 0x106)
        SANSAVIEW.lcd_write_reg(0x39, 0x7)
    else
        SANSAVIEW.lcd_write_reg(0x13, 0x1800)
        SANSAVIEW.lcd_write_reg(0x29, 0x13)
        SANSAVIEW.lcd_write_reg(0x20, 0x0)
        SANSAVIEW.lcd_write_reg(0x21, 0x0)

        SANSAVIEW.lcd_write_reg(0x30, 0x2)
        SANSAVIEW.lcd_write_reg(0x31, 0x606)
        SANSAVIEW.lcd_write_reg(0x32, 0x501)
        SANSAVIEW.lcd_write_reg(0x35, 0x206)
        SANSAVIEW.lcd_write_reg(0x36, 0x504)
        SANSAVIEW.lcd_write_reg(0x37, 0x707)
        SANSAVIEW.lcd_write_reg(0x38, 0x306)
        SANSAVIEW.lcd_write_reg(0x39, 0x7)
    end

    SANSAVIEW.lcd_write_reg(0x3c, 0x700)
    SANSAVIEW.lcd_write_reg(0x3d, 0x700)

    SANSAVIEW.lcd_write_reg(0x50, 0x0)
    SANSAVIEW.lcd_write_reg(0x51, 0xef)  -- 239 - LCD_WIDTH
    SANSAVIEW.lcd_write_reg(0x52, 0x0)
    SANSAVIEW.lcd_write_reg(0x53, 0x13f) -- 319 - LCD_HEIGHT

    SANSAVIEW.lcd_write_reg(0x60, 0x2700)
    SANSAVIEW.lcd_write_reg(0x61, 0x1)
    SANSAVIEW.lcd_write_reg(0x6a, 0x0)

    SANSAVIEW.lcd_write_reg(0x80, 0x0)
    SANSAVIEW.lcd_write_reg(0x81, 0x0)
    SANSAVIEW.lcd_write_reg(0x82, 0x0)
    SANSAVIEW.lcd_write_reg(0x83, 0x0)
    SANSAVIEW.lcd_write_reg(0x84, 0x0)
    SANSAVIEW.lcd_write_reg(0x85, 0x0)

    SANSAVIEW.lcd_write_reg(0x90, 0x10)
    SANSAVIEW.lcd_write_reg(0x92, 0x0)
    SANSAVIEW.lcd_write_reg(0x93, 0x3)
    SANSAVIEW.lcd_write_reg(0x95, 0x110)
    SANSAVIEW.lcd_write_reg(0x97, 0x0)
    SANSAVIEW.lcd_write_reg(0x98, 0x0)

    SANSAVIEW.lcd_write_reg(0xc, 0x110)
    SANSAVIEW.lcd_write_reg(0x7, 0x173)
end

function SANSAVIEW.sd_init()
    HW.GPIO.OUTPUT_VALn[7].write(0) -- GPIOH_OUTPUT_VAL = 0
    HW.GPIO.ENABLEn[8].write(0) -- GPIOI_ENABLE = 0
    HW.GPIO.OUTPUT_ENn[8].write(0) -- GPIOI_OUTPUT_EN = 0
    HW.GPIO.OUTPUT_VALn[8].write(0) -- GPIOI_OUTPUT_VAL = 0
    HW.DEVICE.INIT1.write(bit32.band(HW.DEVICE.INIT1.read(),0xffcccfff)) -- DEV_INIT1 &= 0xffcccfff
    HW.DEVICE.INIT1.write(bit32.bor(HW.DEVICE.INIT1.read(),0x333000)) -- DEV_INIT1 |= ~0xffcccfff
    HW.DEVICE.INIT1.write(bit32.band(HW.DEVICE.INIT1.read(),0xffcccfff)) -- DEV_INIT1 &= 0xffcccfff
    HW.DEVICE.INIT2.write(bit32.band(HW.DEVICE.INIT2.read(),0xfffffeff)) -- DEV_INIT2 &= ~0x100
    PP.gpio.pin("O", 5).enable(true) -- GPIOO_ENABLE |= 0x20
    PP.gpio.pin("O", 5).output_enable(true) -- GPIOO_OUTPUT_EN |= 0x20
    PP.gpio.pin("O", 5).write(true) -- GPIOO_OUTPUT_VAL |= 0x20
end

function SANSAVIEW.sd_init2()
    HW.DEVICE.INIT2.clr(0x100)
    HW.DEVICE.INIT2.clr(0x800)
    HW.DEVICE.INIT2.clr(0x4000)
    HW.DEVICE.INIT1.clr(0x333000)
    HW.DEVICE.INIT1.set(0x333000)
    HW.DEVICE.INIT3.clr(0xf0000000)
    PP.gpio.pin("O", 5).enable(true)
    PP.gpio.pin("O", 5).write(true)
    PP.gpio.pin("O", 5).output_enable(true)
    PP.gpio.pin("O", 6).enable(true)
    PP.gpio.pin("O", 6).write(false)
    PP.gpio.pin("O", 6).output_enable(true)
end

function SANSAVIEW.sd_select(which)
    if which ~= 0 then
        HW.DEVICE.INIT1.clr(0x333000)
        PP.gpio.pin("I", 6).write(true)
        PP.gpio.pin("I", 6).output_enable(true)
        PP.gpio.pin("I", 6).enable(true)
        for i = 0, 3 do
            PP.gpio.pin("H", i).write(true)
            PP.gpio.pin("H", i).output_enable(true)
            PP.gpio.pin("H", i).enable(true)
        end
        HW.DEVICE.INIT3.clr(0xf0000000)
        HW.DEVICE.INIT3.set(0xf0000000)
        PP.gpio.pin("D", 5).enable(false)
        PP.gpio.pin("C", 6).enable(false)
        for i = 0, 3 do
            PP.gpio.pin("L", i).enable(false)
        end
        PP.gpio.pin("O", 6).write(true)
    else
        PP.gpio.pin("D", 5).write(true)
        PP.gpio.pin("D", 5).output_enable(true)
        PP.gpio.pin("D", 5).enable(true)
        for i = 0, 3 do
            PP.gpio.pin("L", i).write(true)
            PP.gpio.pin("L", i).output_enable(true)
            PP.gpio.pin("L", i).enable(true)
        end
        HW.DEVICE.INIT1.clr(0x333000)
        HW.DEVICE.INIT1.set(0x333000)
        PP.gpio.pin("I", 6).enable(false)
        PP.gpio.pin("I", 5).enable(false)
        for i = 0, 3 do
            PP.gpio.pin("H", i).enable(false)
        end
        PP.gpio.pin("O", 5).write(true)
    end
end

function SANSAVIEW.init()
    --SANSAVIEW.set_backlight(true)
    --SANSAVIEW.lcd_init()
    --SANSAVIEW.sd_init()
end
