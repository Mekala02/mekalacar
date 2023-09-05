#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from rclpy.executors import ExternalShutdownException
from messages.msg import Rc

import sys

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
    
        self.j_steering_f_r = 0.0
        self.j_tire_f_r = 0.0
        self.j_tire_r_r = 0.0
        self.j_steering_f_l = 0.0
        self.j_tire_f_l = 0.0
        self.j_tire_r_l = 0.0

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.sub = self.create_subscription(Rc, "rc", self.sub_callback, 10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.get_logger().info("{0} started".format(self.nodeName))

        self.joint_state_msg = JointState()

    def timer_callback(self):
    # update joint_state
        now = self.get_clock().now()
        self.joint_state_msg .header.stamp = now.to_msg()
        self.joint_state_msg .name = ["j_steering_f_r", "j_tire_f_r", "j_tire_r_r", "j_steering_f_l", "j_tire_f_l", "j_tire_r_l"]
        self.joint_state_msg .position = [
            self.j_steering_f_r,
            self.j_tire_f_r,
            self.j_tire_r_r,
            self.j_steering_f_l,
            self.j_tire_f_l,
            self.j_tire_r_l
        ]
        # send the joint state and transform
        self.joint_pub.publish(self.joint_state_msg )

    def sub_callback(self, data):
        self.j_steering_f_r = data.steering
        self.j_tire_f_r = 0.0
        self.j_tire_r_r = 0.0
        self.j_steering_f_l = data.steering
        self.j_tire_f_l = 0.0
        self.j_tire_r_l = 0.0

def main(args=None):
    rclpy.init(args=args)
    try:
        state_publisher = StatePublisher()
        rclpy.spin(state_publisher)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        state_publisher.destroy_node()

if __name__ == '__main__':
    main()