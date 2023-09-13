#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
from messages.msg import Rc

from arduino.common_functions import PID, Limiter, pwm2float, float2pwm

import serial
import time
import sys
import re

class Arduino(Node):
    def __init__(self):
        super().__init__('arduino')

        node = rclpy.create_node('arduino')

        self.act_value_type = node.declare_parameter("ACT_VALUE_TYPE", rclpy.Parameter.Type.STRING).value
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

        self.steering_a = 1500
        self.act_value_a = 1500
        self.speed_a = 0.0
        self.act_value = 0.0
        self.steering = 0.0
        self.throttle = 0.0
        self.mode1 = 0
        self.mode2 = 0
        self.speed = 0.0

        self.rc_msg = Rc()
        self.speed_msg = Float32()
        qos_profile = QoSProfile(depth=10)
        self.rc_pub = node.create_publisher(Rc, "rc", qos_profile)
        self.encoder_pub = node.create_publisher(Float32, "encoder", qos_profile)

        self.arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.006, write_timeout=0.006)
        time.sleep(0.04)
        node.get_logger().info("Node Created")
        self.timer = self.create_timer(1.0/120.0, self.timer_callback)


    def timer_callback(self):
        self.grab_data()
        self.update()

    def grab_data(self):
        buffer = False
        try: buffer = self.arduino.read(self.arduino.in_waiting).decode('utf-8') 
        except Exception as e: pass
        # Looking if data is right format
        if buffer and buffer[0] == "s":
            # re.search(r"t\d+.\d+s\d+.\d+v\d+.\d+e", data) // todo
            data_array = re.split(r's|t|m|m|v|e', buffer)
            self.steering_a = int(data_array[1])
            self.act_value_a = int(data_array[2])
            # if 0 --> radio turned off, elif 1 --> mode 1, elif 2 --> mode 2
            self.mode1 = int(data_array[3])
            self.mode2 = int(data_array[4])
            self.speed_a = float(data_array[5])

    def update(self):
        '''
        We want to send the data as fast as we can so we are only sending 2 int as string, those
        strings encode motor power and drive mode parameters to throttle and steering values
        If char is between 1000, 2000 arduino will use that value to drive motors
        If char is = 0 arduino won't use that value for controlling the actuator
        '''
        # # speed_a is ticks/sec we converting it to unit/sec
        self.speed = self.speed_a / self.ticks_per_unit
        # pilot_mode_string = "Manuel"
        # # pilot_mode_string = self.memory.memory["Pilot_Mode"]
        # self.act_value = pwm2float(self.act_value_a)
        # if pilot_mode_string == "Full_Auto":
        #     Steering_Signal = float2pwm(-self.memory.memory["Steering"])
        #     Throttle_Signal = float2pwm(self.memory.memory["Throttle"] * self.memory.memory["Motor_Power"])
        # elif pilot_mode_string == "Manuel" or pilot_mode_string == "Angle":
        #     if self.act_value_type == "Throttle":
        #         self.Throttle = self.act_value
        #         Throttle_Signal = 0
        #     elif self.act_value_type == "Speed":
        #         self.Target_Speed = self.stick_multiplier * self.act_value
        #         self.Throttle = self.throttle_limiter(self.pid(self.Speed, self.Target_Speed))
        #         Throttle_Signal = float2pwm(self.Throttle)
        #         self.memory.memory["Target_Speed"] = self.Target_Speed
        #     self.memory.memory["Throttle"] = self.Throttle
        #     if pilot_mode_string == "Manuel":
        #         self.Steering = -pwm2float(self.steering_a)
        #         self.memory.memory["Steering"] = self.Steering
        #         Steering_Signal = 0
        #     if pilot_mode_string == "Angle":
        #         Steering_Signal = float2pwm(-self.memory.memory["Steering"])

        Steering_Signal = 0
        Throttle_Signal = 0
        self.steering = -pwm2float(self.steering_a)
        self.throttle = pwm2float(self.act_value_a)

        self.rc_msg.steering = self.steering
        self.rc_msg.throttle = self.throttle
        self.rc_msg.button1 = self.mode1
        self.rc_msg.button2 = self.mode2
        self.rc_pub.publish(self.rc_msg)

        self.speed_msg.data=self.speed
        self.encoder_pub.publish(self.speed_msg)

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


def main(args=None):
    rclpy.init(args=args)
    try:
        arduino = Arduino()      
        rclpy.spin(arduino)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        arduino.shut_down()
        rclpy.try_shutdown()
        arduino.destroy_node()

if __name__ == "__main__":
    main()