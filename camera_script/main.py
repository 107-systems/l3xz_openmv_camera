'''
This software is distributed under the terms of the MIT License.
Copyright (c) 2022 107-Systems
Author: Jonas Wühr
'''
import image, network, omv, rpc, sensor, struct, pyb

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
omv.disable_fb(True) # Disable IDE framedump

interface = rpc.rpc_usb_vcp_slave() # Open slave

def jpeg_image_snapshot(data): # Snapshot rpc callback
    pixformat, framesize = bytes(data).decode().split(",")
    sensor.set_pixformat(eval(pixformat))
    sensor.set_framesize(eval(framesize))
    img = sensor.snapshot().compress(quality=90)
    return struct.pack("<I", img.size())

def jpeg_image_read_cb(): # Transmit buffer callback
    interface.put_bytes(sensor.get_fb().bytearray(), 5000) # timeout

def jpeg_image_read(data): # Read buffer callback (initiates transmit buffer
    interface.schedule_callback(jpeg_image_read_cb)
    return bytes()

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)
ir_led = pyb.LED(4)
red_led.off()
green_led.off()
blue_led.off()
ir_led.off()

def rgb(data): # RGB LED callback
    r, g, b = struct.unpack("<bbb", data)
    if r:
        red_led.on()
    else:
        red_led.off()
    if g:
        green_led.on()
    else:
        green_led.off()
    if b:
        blue_led.on()
    else:
        blue_led.off()

    return bytes()

def ir(data): # IR LED callback
    state = struct.unpack("<b", data) 
    if state:
        ir_led.on()
    else:
        ir_led.off()

    return bytes()

gpios = []
def gpio_config(data): # GPIO configuration callback
    pin, output, opendrain, pullup, pulldown = struct.unpack("<bbbbb", data) 
    pin_nr = "P" + str(pin)
    pin_config = None
    if output:
        if opendrain:
            pin_config = pyb.Pin(pin_nr, pyb.Pin.OUT_OD)
        elif pullup:
            pin_config = pyb.Pin(pin_nr, pyb.Pin.OUT_PP, pyb.Pin.PULL_UP)
        elif puldown:
            pin_config = pyb.Pin(pin_nr, pyb.Pin.OUT_PP, pyb.Pin.PULL_DOWN)
        else:
            pin_config = pyb.Pin(pin_nr, pyb.Pin.OUT_PP, pyb.Pin.PULL_NONE)
    else:
        if pullup:
            pin_config = pyb.Pin(pin_nr, pyb.Pin.IN, pyb.Pin.PULL_UP)
        elif puldown:
            pin_config = pyb.Pin(pin_nr, pyb.Pin.IN, pyb.Pin.PULL_DOWN)
        else:
            pin_config = pyb.Pin(pin_nr, pyb.Pin.IN, pyb.Pin.PULL_NONE)

    found = False
    for g in gpios:
        if pin == g[0]:
            g[1] = pin_config
            found = True
            break
    if not found:
        gpios.append([pin, pin_config])
    return bytes()

def gpio_set(data): # Set GPIO
    pin, value = struct.unpack("<bb", data)
    for g in gpios:
        if pin == g[0]:
            g[1].value(value)
            break
    return bytes()

def gpio_poll(data): # Poll GPIO
    pin = int(bytes(data).decode()) 
    val = -1
    for g in gpios:
        if pin == g[0]:
            val = g[1].value()
            break
    return struct.pack("<b", val)

# Register callbacks and loop
interface.register_callback(jpeg_image_snapshot)
interface.register_callback(jpeg_image_read)
interface.register_callback(rgb)
interface.register_callback(ir)
interface.register_callback(gpio_config)
interface.register_callback(gpio_set)
interface.register_callback(gpio_poll)

interface.loop()
