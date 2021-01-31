#!/usr/bin/env python

# this is used for publish some need formation
# pub message 50 hz
'''dd ROS Node'''
import rospy
from std_msgs.msg import String
from target.msg import robotState
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import tf
import math
import numpy as np

curRobotPos = robotState()
last_angle_global = 0
elapse_angle_global = 0
last_time = 0
last_pos = Point()

rho = 0
pub = rospy
def getName():
    global name
    global pub
    global rho
    name = rospy.get_param("~number","robot_1")
    pub = rospy.Publisher("/"+name+"/robotStates",robotState,queue_size=1)
    rho = rospy.get_param("~rho",1)

#different with matlab
def normalize_angle(curAngle,lastAngle,x):
    if curAngle<=0 : 
        return -(curAngle - lastAngle)
    elif curAngle>0 and lastAngle<=0 :
        if x > 0:
            return lastAngle - curAngle
        elif x < 0 :
            return 2 * math.pi - curAngle + lastAngle
    elif curAngle>0 and lastAngle>0:
        return -(curAngle - lastAngle)
#same in matlab
def normalize_angle1(curAngle,lastAngle,x):
    if (curAngle >= 0 and lastAngle >= 0) or (curAngle<0 and lastAngle<0): 
        return - (curAngle - lastAngle)
    elif curAngle*lastAngle<0 and x > 0:
        return - (curAngle - lastAngle)
    elif curAngle*lastAngle<0 and x < 0 and curAngle >= 0:
        return 2 * math.pi - (curAngle - lastAngle)
    elif curAngle*lastAngle<0 and x < 0 and curAngle < 0:
        return (lastAngle - curAngle) - 2 * math.pi 

def callRobotOdom(data):
    global curRobotPos
    global pub
    global last_angle_global
    global elapse_angle_global
    global last_time
    global last_pos
    global rho
    global name
    rate = rospy.Rate(50)
    #four to oula: robot
    current_time = rospy.get_time()
    (rx,ry,rz)=tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    #global pos and xi
    curRobotPos.x=data.pose.pose.position.x
    curRobotPos.y=data.pose.pose.position.y
    #nomalize angle and get speed
    # print "curTime:%s" %current_time
    gloAng = math.atan2(curRobotPos.y,curRobotPos.x)
    deltaTime = current_time - last_time
    last_time = current_time
    curRobotPos.vx = data.twist.twist.linear.x * math.cos(rz) + data.twist.twist.linear.y * math.sin(rz)
    curRobotPos.vy = -data.twist.twist.linear.x * math.sin(rz) + data.twist.twist.linear.y * math.cos(rz)

    pos = np.mat([[curRobotPos.x],[curRobotPos.y]])
    v = np.mat([[curRobotPos.vx],[curRobotPos.vy]])
    N = pos / (rho * np.linalg.norm(pos))
    T = np.mat([[0,1],[-1,0]]) * N
    yita = (( 1 / (rho * np.linalg.norm(T)) * (T.T * v) ).tolist())[0][0]
    curRobotPos.yita = yita
    
    if last_angle_global == 0:
        last_angle_global = -gloAng
        elapse_angle_global = last_angle_global
    else:
        #in normal
        #deltaAngle = normalize_angle(gloAng , last_angle_global,curRobotPos.x)
        #same in matlab
        deltaAngle = normalize_angle1(gloAng , last_angle_global,curRobotPos.x)
        # print "deltaAngle:%s gloAng:%s last:%s" %(deltaAngle,gloAng,last_angle_global)        
        elapse_angle_global += deltaAngle
        curRobotPos.xi = elapse_angle_global
        curRobotPos.angSp = deltaAngle / deltaTime   # radians per seconds 
        curRobotPos.dirAngle = rz
    last_angle_global = gloAng
    # print "-----name:%s rho:%s yita:%s angSp:%s" %(name,rho,yita,curRobotPos.angSp)
    # pub message
    pub.publish(curRobotPos)
    # print "last_angle_global:%s" %last_angle_global
    # rospy.loginfo(name+":(%3f,%3f),xi: %3f,w: %3s, robotAngle: %3s, vx:%3s, vy:%3s",curRobotPos.x,curRobotPos.y,curRobotPos.xi,curRobotPos.angSp, curRobotPos.dirAngle, curRobotPos.vx, curRobotPos.vy)
    rate.sleep()

def listener():
    '''dd Subscriber'''
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('odomToStates', anonymous=True)
    getName()
    rospy.Subscriber("/"+name+"/odom",Odometry,callRobotOdom,queue_size=1)
    global last_time 
    last_time = rospy.get_time()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    return 0

if __name__ == '__main__':
    listener()
