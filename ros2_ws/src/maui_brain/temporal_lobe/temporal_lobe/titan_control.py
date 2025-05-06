import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import math as m

class WingControl():
    def __init__(self, range, freq_M, var_speed):
        
        self.r = range                      # range of control   
        self.A_M = (range[1] - range[0])/2  # maximum amplitude of the sinusoid  
        self.m = (range[1] + range[0])/2
        self.freq_M = freq_M                # maximum frequency 
        
        self.f = 0
        self.v = var_speed
        self.phase = 0  # Initialize phase
        self.prev_time = 0  # Initialize previous time
    
    def control(self, input, t):

        [x,y] = input

        self.update(input)
        w = self.f
        
        Al = self.A_M+max(-1, min(x, 0))        # Sinusoid amplitude
        Ar = self.A_M-max(0, min(x, 1))
        
        
        if y<0: [Al,Ar] = [-Al,-Ar]
        
        delta_t = t - self.prev_time  # Calculate elapsed time
        self.phase += 2 * m.pi * w * delta_t  # Update phase based on frequency and elapsed time
        self.prev_time = t  # Update previous time

        l = Al * m.sin(self.phase) + self.m
        r = Ar * m.sin(self.phase) + self.m
        
        
        least_decimal = 3
        return [round(l, least_decimal), round(-r, least_decimal)]
    
    def update(self, input):
        
        [x,y] = input
        
        new_f = m.sqrt(x*x+y*y)*self.freq_M
        
        if new_f < 1e-3:    
            self.f = 0
            self.phase = 0 
        else:               
            self.f = self.f + (new_f-self.f)*self.v
            
        
        max(0, min(self.f, self.freq_M))
        
        
class TailControl():
    
    def __init__(self, range):
        self.A_M = (range[1] - range[0])/2    # maximum amplitude of the sinusoid
        self.r = range
        
    def control(self, input):
        
        [x,y] = input
        
        A = self.A_M*max(-1, min(y, 1))
        dA = x*self.A_M/2
        
        [l, r] = [max(self.r[0], min(A+dA, self.r[1])), max(self.r[0], min(A-dA, self.r[1]))]
        
        least_decimal = 3
        return [round(l, least_decimal), round(-r, least_decimal)]

class JoyManager(Node):

    def __init__(self):
        super().__init__('joycon_manager') # node name 
        self.subscription = self.create_subscription( Joy, 'joy', self.joy_callback, 10) # subscribe to controller topic
        self.publisher = self.create_publisher(Float32MultiArray, 'servo_angles', 10) # publish data 
        
        self.declare_parameter('publish_rate', 0.001)                # declare publish rate parameter if set -> impose 0.1Hz value otherwise
        rate = self.get_parameter('publish_rate').value             # set rate as parameter value
         
        self.axes = [0, 0, 0, 0]    # save controller data on update
        self.drift = 0.1            # discard values with abs value less than drift
        
        # Wing control setup
        standard_range = [-m.pi/3, m.pi/3]                          # standard value of wing range
        self.declare_parameter('wing_range', standard_range)        # declare range parameter if set -> impose standard value otherwise
        range = self.get_parameter('wing_range').value              # set range as parameter value
        
        standard_max_freq = 1                                       # standard value of wing maximum flap frequency
        self.declare_parameter('wing_max_freq', standard_max_freq)  # declare freq parameter if set -> impose standard value otherwise
        max_freq = self.get_parameter('wing_max_freq').value        # set max_freq as parameter value
        
        standard_freq_v = 0.1                                       # standard value of wing maximum flap frequency
        self.declare_parameter('wing_freq_v', standard_freq_v)      # declare freq parameter if set -> impose standard value otherwise
        freq_v = self.get_parameter('wing_freq_v').value            # set max_freq as parameter value
        
        self.wings = WingControl(range, max_freq, freq_v)           # setup wing control with the given parameters
        
        # Flap control setup 
        standard_range = [-m.pi/2, m.pi/2]                          # standard value of tail flap range
        self.declare_parameter('tail_range', standard_range)        # declare range parameter if set -> impose standard value otherwise
        range = self.get_parameter('tail_range').value              # set range as parameter value
        
        self.tail = TailControl(range)                              # setup tail control with the given parameters
        
        self.start = self.get_clock().now().seconds_nanoseconds()[0]# starting seconds from clock
        self.timer = self.create_timer(rate, self.timer_callback)   # set function to execute every {rate} Hz

    def joy_callback(self, msg): # save joycon pads 
        
        axes = msg.axes
        corr_axes = []
        
        for val in axes:
            if abs(val) < self.drift:
                corr_axes.append(0)
            else:
                corr_axes.append(val)
                
        self.axes = corr_axes
        # self.get_logger().info(f'Joystick Axes: {self.axes}')
        
    def output_msg(self, wl, wr, tl, tr): # create msg to publish on rostopic
        msg = Float32MultiArray()
        msg.data = [wl, wr, tl, tr]  # Replace with your actual data
        return msg
    
    def timer_callback(self):
        current_time = self.get_clock().now().seconds_nanoseconds()
        time_in_seconds = current_time[0]- self.start + current_time[1] * 1e-9 
        t = round(time_in_seconds, 3)
        
        wing_input = self.axes[0:2]
        
        tail_input = self.axes[2:4]
        
        [wl, wr] = self.wings.control(wing_input, t)
        [tl, tr] = self.tail.control(tail_input)
        
        msg = self.output_msg(wl, wr, tl, tr)
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    joy_listener = JoyManager()

    rclpy.spin(joy_listener)

    joy_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()