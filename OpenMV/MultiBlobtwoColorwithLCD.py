
import image, sensor, ustruct, pyb, lcd, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # use QVGA 320*240
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
lcd.init(triple_buffer=True) # Initialize the lcd screen.  Make Non-blocking but 3X RAM

uart = pyb.UART(3)
uart.init(115200, bits=8, parity=None)
threshold1 = (0, 100, -128, 127, -128, 127) # change to a color threshold range
threshold2 = (0, 100, -128, 127, -128, 127) # change to a color threshold range
# Packets to Send
blob_packet = '<fff'

# Setup RED LED for easier debugging
red_led   = pyb.LED(1)
green_led = pyb.LED(2)
blue_led  = pyb.LED(3)
clock = time.clock()                # Create a clock object to track the FPS.
framecount = 0
toggle = 0
offcount = 0

while True:
    clock.tick()                    # Update the FPS clock.
    if framecount % 2 == 0:
        if toggle == 0:
            blue_led.on()
            toggle = 1
            offcount = 0
        else:
            blue_led.off()
            offcount = offcount + 1
            if offcount == 20:
                toggle = 0
                offcount = 0

    img = sensor.snapshot()

    blobs1 = img.find_blobs([threshold1], roi=(0,80,320,240), pixels_threshold=5, area_threshold=20)
    blobs2 = img.find_blobs([threshold2], roi=(0,80,320,240), pixels_threshold=5, area_threshold=20)

    if blobs1:
        blob1_sort = sorted(blobs1, key = lambda b: b.pixels(), reverse=True)
        blob1_largest = blob1_sort[:3]
        blobs1_found = len(blob1_largest)

        msg = "**".encode()
        uart.write(msg)
        for i in range(3):
            if i < blobs1_found:
                b = blob1_largest[i]
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
    else:  # nothing found
        msg = "**".encode()
        uart.write(msg)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)

    if blobs2:
        blob2_sort = sorted(blobs2, key = lambda b: b.pixels(), reverse=True)
        blob2_largest = blob2_sort[:3]
        blobs2_found = len(blob2_largest)

        msg = "*!".encode()
        uart.write(msg)
        for i in range(3):
            if i < blobs2_found:
                b = blob2_largest[i]
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
    else:  # nothing found
        msg = "*!".encode()
        uart.write(msg)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)

    lcd.display(img, roi=(96,80,224,240)) # display the image to lcd only middle 128 cols by 160 rows.
    print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
