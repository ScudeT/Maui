#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import gpiod
from std_msgs.msg import Bool
import time

class ButtonReader(Node):
    def __init__(self):
        super().__init__('button_reader')
        
        # Declare parameters with default values.
        # freq is the timer frequency in Hz.
        # gpio_line is the line number for gpiod.Chip.get_line.
        # break_time is the period (in seconds) to ignore events after a release.
        # release decides if falling edge (button release) events are published.
        self.declare_parameter('freq', 100.0)
        self.declare_parameter('gpio_line', 4)
        self.declare_parameter('break_time', 1.0)
        self.declare_parameter('release', True)
        
        # Retrieve parameter values.
        freq = self.get_parameter('freq').value
        gpio_line = self.get_parameter('gpio_line').value
        break_time = self.get_parameter('break_time').value
        release = self.get_parameter('release').value
        
        self.freq = freq
        self.gpio_line = gpio_line
        self.break_time = break_time
        self.release = release
        
        self.get_logger().info(
            f"Parameters set: freq={self.freq} Hz, gpio_line={self.gpio_line}, "
            f"break_time={self.break_time}s, release publishing={self.release}"
        )
        
        # Calculate timer period from frequency (period = 1/freq).
        timer_period = 1.0 / self.freq if self.freq > 0 else 0.01
        
        # Publisher for button state (True for pressed, False for released).
        self.publisher_ = self.create_publisher(Bool, 'button_state', 10)
        
        # Open GPIO chip and request the specified line.
        self.chip = gpiod.Chip('/dev/gpiochip0')
        self.line = self.chip.get_line(self.gpio_line)
        
        # Request both rising and falling edge events with pull-down bias.
        self.line.request(
            consumer="button_reader",
            type=gpiod.LINE_REQ_EV_BOTH_EDGES,
            flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN
        )
        
        # Create a timer to poll for events.
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Timestamp until which events should be ignored.
        self.ignore_until = 0.0

    def timer_callback(self):
        now = time.monotonic()
        # If still in ignore period, flush (discard) any queued events.
        if now < self.ignore_until:
            while self.line.event_wait(0):
                self.line.event_read()  # Discard stale events.
            return
        
        # Check for a new event (non-blocking).
        if self.line.event_wait(0):
            event = self.line.event_read()
            if event.type == gpiod.LineEvent.RISING_EDGE:
                self.get_logger().info("Button pressed (rising edge detected)")
                msg = Bool(data=True)
                self.publisher_.publish(msg)
            elif event.type == gpiod.LineEvent.FALLING_EDGE:
                if self.release:
                    self.get_logger().info("Button released (falling edge detected)")
                    msg = Bool(data=False)
                    self.publisher_.publish(msg)
                # Start ignore period after a falling edge.
                self.ignore_until = now + self.break_time

def main(args=None):
    rclpy.init(args=args)
    node = ButtonReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
