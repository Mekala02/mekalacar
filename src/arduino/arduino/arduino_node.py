#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32
from messages.msg import Rc
# from common_functions import PID, Limiter, pwm2float, float2pwm
import time

class PID():
    def __init__(self, Kp=0, Ki=0, Kd=0, I_max=1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.I_max = I_max
        self.current_time = 0
        self.previous_time = 0
        self.current_error = 0
        self.previous_error = 0
        self.P = 0
        self.I = 0
        self.D = 0

    def __call__(self, current_value, requested_value):
        self.current_time = time.time()
        delta_time = self.current_time - self.previous_time
        self.current_error = requested_value - current_value
        delta_error = self.current_error - self.previous_error

        self.P = self.Kp * self.current_error
        self.I += self.Ki * self.current_error * delta_time
        self.D = self.Kd * delta_error / delta_time

        # Integrator limit
        if self.I > self.I_max: self.I = self.I_max
        if self.I < -self.I_max: self.I = -self.I_max

        # Integrator Reset
        # if - 0.1 < self.current_error < 0.1: self.I = 0

        self.previous_time = self.current_time
        self.previous_error = self.current_error

        return self.P + self.I + self.D

class Limiter():
    def __init__(self, min_=0, max_=1):
        self.min = min_
        self.max = max_

    def __call__(self, value):
        if value > self.max:
            return self.max
        elif value < self.min:
            return self.min
        else:
            return value

def sgn(x):
    return (x > 0) - (x < 0)

def pwm2float(value):
    return (value - 1500) / 500

def float2pwm(value):
    return int(value * 500 + 1500)

# import rospy
# from std_msgs.msg import String, Float32
# from arduino.msg import rc
# from common.common_functions import PID, Limiter, pwm2float, float2pwm

import serial
import time
import re

class Arduino(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('arduino')
        qos_profile = QoSProfile(depth=10)
        node = rclpy.create_node('arduino')
        node.get_logger().info('Created node')

        self.outputs = {"Steering": 0, "Throttle": 0, "Speed": 0, "Mode1": 0, "Mode2": 0, "Target_Speed": None}
        # Also It can output Act_Value, Record and drive mode acoording to mode button
        # self.act_value_type = memory.cfg["ACT_VALUE_TYPE"]
        
        self.act_value_type = node.declare_parameter("ACT_VALUE_TYPE", rclpy.Parameter.Type.STRING).value
        self.Steering_A = 1500
        self.Act_Value_A = 1500
        self.Speed_A = 0
        self.mode1 = 0
        self.mode2 = 0
        self.steering = 0.0
        self.Act_Value = 0
        self.speed = 0.0
        self.throttle = 0.0
        self.ticks_per_unit = node.declare_parameter("ENCODER_TICKS_PER_UNIT", rclpy.Parameter.Type.INTEGER).value
        self.stick_multiplier = node.declare_parameter("TRANSMITTER_STICK_SPEED_MULTIPLIER", rclpy.Parameter.Type.DOUBLE).value
        self.steering_limiter = Limiter(
            min_=pwm2float(node.declare_parameter("STEERING_MIN_PWM", rclpy.Parameter.Type.INTEGER).value),
            max_=pwm2float(node.declare_parameter("STEERING_MAX_PWM", rclpy.Parameter.Type.INTEGER).value)
        )
        self.throttle_limiter = Limiter(
            min_=pwm2float(node.declare_parameter("THROTTLE_MIN_PWM", rclpy.Parameter.Type.INTEGER).value),
            max_=pwm2float(node.declare_parameter("THROTTLE_MAX_PWM", rclpy.Parameter.Type.INTEGER).value)
        )
        if self.act_value_type == "Speed":
            self.Target_Speed = 0
            self.pid = PID(
                Kp=node.declare_parameter("K_PID", rclpy.Parameter.Type.STRING).value["Kp"],
                Ki=node.declare_parameter("K_PID", rclpy.Parameter.Type.STRING).value["Ki"],
                Kd=node.declare_parameter("K_PID", rclpy.Parameter.Type.STRING).value["Kd"],
                I_max=node.declare_parameter("K_PID", rclpy.Parameter.Type.STRING).value["I_max"]
            )

        self.arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.006, write_timeout=0.006)
        time.sleep(0.04)

        # self.rc_pub = rospy.Publisher('chatter', String)
        self.rc_pub = node.create_publisher(Rc, "rc", qos_profile)
        self.encoder_pub = node.create_publisher(Float32, "encoder", qos_profile)
        self.rate = self.create_rate(30)

        # rospy.init_node("arduino", anonymous=True)
        # self.rc_pub = rospy.Publisher("rc", rc, queue_size=10)
        # self.encoder_pub = rospy.Publisher("encoder", Float32, queue_size=10)
        # self.rate = rospy.Rate(120)
        # rospy.loginfo("Arduino Successfully Added")

    def grab_data(self):
        buffer = False
        try: buffer = self.arduino.read(self.arduino.in_waiting).decode('utf-8') 
        except Exception as e: pass
        # Looking if data is right format
        if buffer and buffer[0] == "s":
            # re.search(r"t\d+.\d+s\d+.\d+v\d+.\d+e", data) // todo
            data_array = re.split(r's|t|m|m|v|e', buffer)
            self.Steering_A = int(data_array[1])
            self.Act_Value_A = int(data_array[2])
            # if 0 --> radio turned off, elif 1 --> mode 1, elif 2 --> mode 2
            self.Mode1 = int(data_array[3])
            self.Mode2 = int(data_array[4])
            self.Speed_A = float(data_array[5])

    def start_thread(self):
        # rospy.loginfo("Arduino Node Starting")
        while rclpy.ok():
            self.grab_data()
            self.update()
            self.rate.sleep()

    def update(self):
        '''
        We want to send the data as fast as we can so we are only sending 2 int as string, those
        strings encode motor power and drive mode parameters to throttle and steering values
        If char is between 1000, 2000 arduino will use that value to drive motors
        If char is = 0 arduino won't use that value for controlling the actuator
        '''
        # # Speed_A is ticks/sec we converting it to unit/sec
        self.Speed = self.Speed_A / self.ticks_per_unit
        # pilot_mode_string = "Manuel"
        # # pilot_mode_string = self.memory.memory["Pilot_Mode"]
        # self.Act_Value = pwm2float(self.Act_Value_A)
        # if pilot_mode_string == "Full_Auto":
        #     Steering_Signal = float2pwm(-self.memory.memory["Steering"])
        #     Throttle_Signal = float2pwm(self.memory.memory["Throttle"] * self.memory.memory["Motor_Power"])
        # elif pilot_mode_string == "Manuel" or pilot_mode_string == "Angle":
        #     if self.act_value_type == "Throttle":
        #         self.Throttle = self.Act_Value
        #         Throttle_Signal = 0
        #     elif self.act_value_type == "Speed":
        #         self.Target_Speed = self.stick_multiplier * self.Act_Value
        #         self.Throttle = self.throttle_limiter(self.pid(self.Speed, self.Target_Speed))
        #         Throttle_Signal = float2pwm(self.Throttle)
        #         self.memory.memory["Target_Speed"] = self.Target_Speed
        #     self.memory.memory["Throttle"] = self.Throttle
        #     if pilot_mode_string == "Manuel":
        #         self.Steering = -pwm2float(self.Steering_A)
        #         self.memory.memory["Steering"] = self.Steering
        #         Steering_Signal = 0
        #     if pilot_mode_string == "Angle":
        #         Steering_Signal = float2pwm(-self.memory.memory["Steering"])

        Steering_Signal = 0
        Throttle_Signal = 0
        self.Steering = -pwm2float(self.Steering_A)
        self.Throttle = pwm2float(self.Act_Value_A)

        msg = Rc()
        msg.steering = self.steering
        msg.throttle = self.throttle
        msg.button1 = self.mode1
        msg.button2 = self.mode2
        self.rc_pub.publish(msg)
        
        msg = Float32()
        msg.data=self.speed
        self.encoder_pub.publish(msg)

        # s is for stating start of steering value t is for throttle and e is for end, \r for read ending
        formatted_data = "s" + str(Steering_Signal) + "t" + str(Throttle_Signal) + 'e' + '\r'
        try: self.arduino.write(formatted_data.encode())
        except Exception as e: pass # rospy.logwarn(e)
        else: pass # rospy.loginfo("Succes !!!")

    def shut_down(self):
        # If we reading from arduino and close the port that
        # will cause error so we waiting to finish reading then close
        time.sleep(0.05)
        self.arduino.close()
        # rospy.loginfo("Arduino Node Stopped")

def main():
    arduino = Arduino()      
    try:
        arduino.start_thread()
    except KeyboardInterrupt:
        arduino.shut_down()
    # except rospy.ROSInitException:
    #     arduino.shut_down()

if __name__ == "__main__":
    main()