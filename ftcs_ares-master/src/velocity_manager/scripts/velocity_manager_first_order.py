#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist, Accel
from nav_msgs.msg import Odometry
from math import atan2, sqrt

#全局变量声明
flowfield_velocity_latest = Twist() #记录最后一个到来的流场数据
control_msg_latest = Twist() #记录最后一个到来的控制输入
expect_velocity = Twist() #记录当前的静场运动速度
ground_truth_latest = Odometry() #记录当前的实时状态
flowfield_direction_publisher  = rospy.Publisher('flowfield_direction', Twist, queue_size=10)

def sign(x):
    if x>0.0:
        return 1.0
    elif x==0.0:
        return 0.0
    else:
        return -1.0
    
def sat(x):
    if abs(x) <= 5:
        return x
    else:
        return 5*sign(x)

def OnFlowfieldMsg(flowfield_velocity):
    #全局变量声明
    global flowfield_velocity_latest
    global flowfield_direction_publisher
    
    flowfield_velocity_latest = flowfield_velocity
    flowfield_direction = Twist()
    flowfield_speed_square = (flowfield_velocity.linear.x * flowfield_velocity.linear.x + flowfield_velocity.linear.y * flowfield_velocity.linear.y)
    flowfield_direction.linear.x=flowfield_velocity.linear.x / flowfield_speed_square
    flowfield_direction.linear.y=flowfield_velocity.linear.y / flowfield_speed_square
    flowfield_direction_publisher.publish(flowfield_direction)
    
def OnControlMsg(control_msg):
    #全局变量声明
    global control_msg_latest
    control_msg_latest = control_msg
    
def OnGroundTruth(ground_truth):
    global ground_truth_latest
    ground_truth_latest = ground_truth

def velocity_manager():
    #全局变量声明
    global flowfield_velocity_latest
    global control_msg_latest
    global expect_velocity
    global flowfield_direction_publisher
    global ground_truth_latest
    
    # ROS节点初始化
    rospy.init_node('velocity_manager', anonymous=False)
    rospy.loginfo("velocity_manager node initialized.") 
    
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    flowfield_direction_publisher = rospy.Publisher('flowfield_direction', Twist, queue_size=10)
    
    rospy.Subscriber("flowfield_velocity", Twist, OnFlowfieldMsg)
    rospy.Subscriber("control_msg", Twist, OnControlMsg)
    rospy.Subscriber("base_pose_ground_truth", Odometry, OnGroundTruth)
    prev_e = 0
    #设置循环的频率
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():  
        cmd_vel = Twist()
        
        #四元数转换为欧拉角
        siny_cosp = 2.0 * (ground_truth_latest.pose.pose.orientation.w * ground_truth_latest.pose.pose.orientation.z + ground_truth_latest.pose.pose.orientation.x * ground_truth_latest.pose.pose.orientation.y);
        cosy_cosp = 1.0 - 2.0 * (ground_truth_latest.pose.pose.orientation.y * ground_truth_latest.pose.pose.orientation.y + ground_truth_latest.pose.pose.orientation.z * ground_truth_latest.pose.pose.orientation.z);
        yaw = atan2(siny_cosp, cosy_cosp)
        
        #使用PD修正小车朝向
        e = yaw
        kp = -12
        kd = 1
        cmd_vel.angular.z = kp * e + kd * (e - prev_e)
        prev_e = e

        #求出实际运动速度
        cmd_vel.linear.x=flowfield_velocity_latest.linear.x + control_msg_latest.linear.x
        cmd_vel.linear.y=flowfield_velocity_latest.linear.y + control_msg_latest.linear.y
        #rospy.loginfo("Publish cmd_vel[linear.x=%f, linear.y=%f]",
        #              cmd_vel.linear.x, cmd_vel.linear.y)
        velocity_publisher.publish(cmd_vel)
        #按照循环频率延时
        rate.sleep()
     

if __name__ == '__main__':
     try:
         velocity_manager()
     except rospy.ROSInterruptException as r:
         rospy.loginfo("velocity_manager node terminated.") 
         rospy.loginfo(r) 
 
