#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from vesc_msgs.msg import VescStateStamped

wheelbase = 0.40
wheel_radian = 0.1651/2


class VescCmdVel:
    def __init__(self) -> None:
        self.vesc_cmd_vel_sub = rospy.Subscriber('/vesc_cmd_vel', Twist, self.vesc_cmd_vel_callback, queue_size=10)
        self.left_wheel_speed_pub = rospy.Publisher('/left_wheel/commands/motor/speed',Float64, queue_size=10)
        self.right_wheel_speed_pub = rospy.Publisher('/right_wheel/commands/motor/speed',Float64, queue_size=10)
        
    def vesc_cmd_vel_callback(self, data):
        twist_cmd = Twist()
        twist_cmd.linear.x = data.linear.x
        twist_cmd.angular.z = data.angular.z
        
        left_wheel_speed = Float64()
        right_wheel_speed = Float64()
        left_wheel_speed.data = (twist_cmd.linear.x - twist_cmd.angular.z * (wheelbase/2))/wheel_radian
        right_wheel_speed.data = -1*(twist_cmd.linear.x + twist_cmd.angular.z * (wheelbase/2))/wheel_radian
        
        self.left_wheel_speed_pub.publish(left_wheel_speed)
        self.right_wheel_speed_pub.publish(right_wheel_speed)
        
        
if __name__ == '__main__':
    rospy.loginfo("Vesc commande velocity Node")
    rospy.init_node('vesc_cmd_vel_node')
    traker = VescCmdVel()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
        
        
        