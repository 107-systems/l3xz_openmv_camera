import image, network, omv, rpc, sensor, struct, pyb

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

omv.disable_fb(True)
interface = rpc.rpc_usb_vcp_slave()

def jpeg_image_snapshot(data):
    pixformat, framesize = bytes(data).decode().split(",")
    sensor.set_pixformat(eval(pixformat))
    sensor.set_framesize(eval(framesize))
    img = sensor.snapshot().compress(quality=90)
    return struct.pack("<I", img.size())

def jpeg_image_read_cb():
    interface.put_bytes(sensor.get_fb().bytearray(), 5000) # timeout

def jpeg_image_read(data):
    if not len(data):
        interface.schedule_callback(jpeg_image_read_cb)
        return bytes()
    else:
        offset, size = struct.unpack("<II", data)
        return memoryview(sensor.get_fb().bytearray())[offset:offset+size]

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)
red_led.off()
green_led.off()
blue_led.off()

def rgb(data):
    r, g, b = bytes(data).decode().split(",")
    if "1" == r:
        red_led.on()
    else:
        red_led.off()
    if "1" == g:
        green_led.on()
    else:
        green_led.off()
    if "1" == b:
        blue_led.on()
    else:
        blue_led.off()

    return bytes()

interface.register_callback(jpeg_image_snapshot)
interface.register_callback(jpeg_image_read)
interface.register_callback(rgb)

interface.loop()
