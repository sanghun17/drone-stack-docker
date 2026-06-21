#!/usr/bin/env python3
"""flight_safety status LED: /flight_safety/state .control_lane -> APA102 color.

Launched (backgrounded) by this module's run.sh as a SEPARATE process from the
KILL-authority response node, so a GPIO hiccup can never stall the 50 Hz safety
loop. Runs inside the privileged container (-> /dev/gpiochip0; FlightState on path).
LED goes OFF when no fresh state arrives, so a down/absent flight_safety reads dark.

Wiring (40-pin J30): APA102 DI->pin19, CI->pin23, VCC->pin2 (5V), GND->pin6.
Colors below are plain edits — no image rebuild needed to change them.
"""
import rospy
import Jetson.GPIO as GPIO
from flight_safety.msg import FlightState

DATA, CLK = 19, 23      # BOARD pins -> APA102 DI / CI
BRIGHT = 8              # 0..31
TIMEOUT = 1.0           # s without a message -> LED off
RENDER_HZ = 20.0
BLINK_HZ = 4.0          # amber LAND blink rate (full on/off cycles per second)

# control_lane -> (r, g, b, blink)
COLORS = {
    "NORMAL": (0, 255, 0, False),    # green  solid  = offboard, healthy
    "MANUAL": (0, 0, 255, False),    # blue   solid  = pilot owns the vehicle
    "LAND":   (255, 80, 0, True),    # amber  blink  = emergency descend in progress
    "KILL":   (255, 0, 0, False),    # red    solid  = force-disarm, terminal
}
OFF = (0, 0, 0, False)


def _byte(b):
    for i in range(8):
        GPIO.output(DATA, (b >> (7 - i)) & 1)
        GPIO.output(CLK, GPIO.HIGH)
        GPIO.output(CLK, GPIO.LOW)


def _show(r, g, b):
    for _ in range(4):
        _byte(0x00)
    _byte(0xE0 | (BRIGHT & 0x1F))
    _byte(b)
    _byte(g)
    _byte(r)
    for _ in range(4):
        _byte(0xFF)


class LedNode(object):
    def __init__(self):
        self.lane = None
        self.last = None
        rospy.Subscriber("/flight_safety/state", FlightState, self._cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / RENDER_HZ), self._render)

    def _cb(self, m):
        self.lane = m.control_lane
        self.last = rospy.Time.now()

    def _render(self, _evt):
        now = rospy.Time.now()
        fresh = self.last is not None and (now - self.last).to_sec() < TIMEOUT
        r, g, b, blink = COLORS.get(self.lane, OFF) if fresh else OFF
        if blink and int(now.to_sec() * BLINK_HZ * 2) % 2:
            r = g = b = 0
        _show(r, g, b)


if __name__ == "__main__":
    rospy.init_node("flight_safety_led")
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(DATA, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(CLK, GPIO.OUT, initial=GPIO.LOW)
    LedNode()
    try:
        rospy.spin()
    finally:
        _show(0, 0, 0)
        GPIO.cleanup()
