#!/usr/bin/env python3
"""APA102 (DotStar) bit-bang test over Jetson GPIO. No SPI / no reboot needed.

Wiring (40-pin J30):
  APA102 VCC -> Pin 2  (5V)
  APA102 GND -> Pin 6  (GND)
  APA102 DI  -> Pin 19 (SPI1_MOSI)   <- DATA below
  APA102 CI  -> Pin 23 (SPI1_SCK)    <- CLK  below
Same wires work later with hardware spidev once SPI is enabled.

Usage: python3 blink_apa102.py [num_leds]
       python3 blink_apa102.py off [num_leds]   # turn all LEDs off
"""
import sys
import time
import Jetson.GPIO as GPIO

args = sys.argv[1:]
OFF = bool(args) and args[0] == "off"
LOOP = bool(args) and args[0] == "loop"
if OFF or LOOP:
    args = args[1:]

DATA = 19          # BOARD pin 19 -> APA102 DI
CLK = 23           # BOARD pin 23 -> APA102 CI
NUM = int(args[0]) if args else 1
BRIGHTNESS = 8     # 0..31 (start low to avoid blinding / inrush)

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(DATA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(CLK, GPIO.OUT, initial=GPIO.LOW)


def write_byte(b):
    for i in range(8):
        GPIO.output(DATA, (b >> (7 - i)) & 1)
        GPIO.output(CLK, GPIO.HIGH)   # APA102 latches on rising edge
        GPIO.output(CLK, GPIO.LOW)


def show(pixels):
    for _ in range(4):                # start frame: 32 zero bits
        write_byte(0x00)
    for r, g, b in pixels:            # per-LED frame: header, B, G, R
        write_byte(0xE0 | (BRIGHTNESS & 0x1F))
        write_byte(b)
        write_byte(g)
        write_byte(r)
    for _ in range(4):                # end frame: clock data through
        write_byte(0xFF)


try:
    if OFF:
        show([(0, 0, 0)] * NUM)
        print("off")
        sys.exit(0)
    seq = [("RED", (255, 0, 0)), ("GREEN", (0, 255, 0)),
           ("BLUE", (0, 0, 255)), ("WHITE", (255, 255, 255))]
    while True:
        for name, c in seq:
            print(name)
            show([c] * NUM)
            time.sleep(1)
        if not LOOP:
            break
    if not LOOP:
        show([(40, 40, 40)] * NUM)    # leave it dim-on so you can see it held
        print("done (held dim white)")
finally:
    GPIO.cleanup()                    # APA102 latches last state, stays lit
