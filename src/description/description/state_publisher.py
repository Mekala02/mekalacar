#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.executors import ExternalShutdownException

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
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.get_logger().info("{0} started".format(self.nodeName))

        # message declarations
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


    #     rospy.init_node("joint_states", anonymous=True)
    #     self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    #     self.rate = rospy.Rate(10)

    #     rospy.Subscriber("rc", rc, self.grab_data)

    #     rospy.loginfo("Joint State Publisher Successfully Added")

    # def grab_data(self, data):
    #     self.j_steering_f_r = data.Steering
    #     self.j_tire_f_r = 0
    #     self.j_tire_r_r = 0
    #     self.j_steering_f_l = data.Steering
    #     self.j_tire_f_l = 0
    #     self.j_tire_r_l = 0

    # def start_thread(self):
    #     rospy.loginfo("Joint State Publisher Starting")
    #     while not rospy.is_shutdown():
    #         # self.grab_data()
    #         self.update()
    #         self.rate.sleep()
    #     jsp.shut_down()

    # def update(self):
    #     msg = JointState()
    #     msg.header.stamp = rospy.Time.now()

    #     msg.name = ["j_steering_f_r", "j_tire_f_r", "j_tire_r_r", "j_steering_f_l", "j_tire_f_l", "j_tire_r_l"]
    #     msg.position = [
    #         self.j_steering_f_r,
    #         self.j_tire_f_r,
    #         self.j_tire_r_r,
    #         self.j_steering_f_l,
    #         self.j_tire_f_l,
    #         self.j_tire_r_l
    #     ]
    #     msg.velocity = []
    #     msg.effort = []

    #     self.joint_state_pub.publish(msg)

    # def shut_down(self):
    #     # If we reading from arduino and close the port that
    #     # will cause error so we waiting to finish reading then close
    #     time.sleep(0.05)
    #     rospy.loginfo("Joint State Publisher Node Stopped")

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
        state_publisher.shut_down()
        rclpy.try_shutdown()
        state_publisher.destroy_node()

if __name__ == '__main__':
    main()