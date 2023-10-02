
import image, sensor, ustruct, pyb, lcd, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # use QVGA 320*240
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
lcd.init(triple_buffer=True) # Initialize the lcd screen.  Make Non-blocking but 3X RAM

uart = pyb.UART(3)
uart.init(115200, bits=8, parity=None)
threshold = (0, 100, -128, 127, -128, 127) # change to a color threshold range
# Packets to Send
blob_packet = '<fff'

# Setup RED LED for easier debugging
red_led   = pyb.LED(1)
green_led = pyb.LED(2)
blue_led  = pyb.LED(3)
clock = time.clock()                # Create a clock object to track the FPS.


while True:
    clock.tick()                    # Update the FPS clock.
    green_led.on()
    img = sensor.snapshot()

    blobs = img.find_blobs([threshold], roi=(0,80,320,240), pixels_threshold=5, area_threshold=20)


    if blobs:
        blob_sort = sorted(blobs, key = lambda b: b.pixels(), reverse=True)
        blob_largest = blob_sort[:3]
        blobs_found = len(blob_largest)

        msg = "**".encode()
        uart.write(msg)
        for i in range(3):
            if i < blobs_found:
                b = blob_largest[i]
                a = float(b.area())
                x_cnt = float(b.cx())
                y_cnt = float(b.cy())
                img.draw_rectangle(b[0:4]) # rect on x,y,w,h
                img.draw_cross(b.cx(), b.cy())
            else:
                a = 0.0
                x_cnt = 0.0
                y_cnt = 0.0

            # Send the blob area and centroids over UART
            b = ustruct.pack(blob_packet, a, x_cnt, y_cnt)
            uart.write(b)

    lcd.display(img, roi=(96,80,224,240)) # display the image to lcd only middle 128 cols by 160 rows.
    print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
    green_led.off()
