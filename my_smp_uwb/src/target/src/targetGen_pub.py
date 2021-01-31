#!/usr/bin/env python
'''targetGen ROS Node'''
# license removed for brevity
# this python generate target postion message 

import rospy
from geometry_msgs.msg import Twist,Point
from target.msg import robotState
import math

def LineFunction(curArc,lamb=0,a=1,b=1):
    tPos = Point()
    tPos.x = (1-lamb)*a*math.cos(-curArc)
    tPos.y = (1-lamb)*b*math.sin(-curArc)
    return tPos

def talker():
    '''targetGen Publisher'''
    rospy.init_node('targetGen', anonymous=True)
    pub = rospy.Publisher('leader', robotState, queue_size=1)
    rate = rospy.Rate(50) # 50hz
    tArcSpeed = rospy.get_param("~AngleSpeed",0.1)
    initArc = rospy.get_param("~initArc",0)
    
    last_point = Point()
    last_angle = 0
    last_time = rospy.get_time()

    last_point.x = LineFunction(initArc).x
    last_point.y = LineFunction(initArc).y

    start = rospy.get_time()
    while start == 0:
        start = rospy.get_time()
    # print "start: %s" %start

    while not rospy.is_shutdown():

        leader = robotState()

        duration = rospy.get_time() - start
        if duration == 0:
            continue
        # print "duration: %s" %duration
        angle = initArc+tArcSpeed*duration #count from x+
            
        leader.xi = angle
        leader.x = LineFunction(angle).x
        leader.y = LineFunction(angle).y
        leader.angSp = tArcSpeed  # angler speed: given in there
        leader.dirAngle = math.atan2(leader.y - last_point.y, leader.x - last_point.x)

        last_point.x = leader.x
        last_point.y = leader.y
        curTime = rospy.get_time()
        if last_time == curTime:
            continue
            
        # print "leaderTime:%s; (%s, %s); xi: %s; anSp: %s; dir: %s" % (duration, leader.x, leader.y, leader.xi, leader.angSp, leader.dirAngle)
        RobotAnSpee = (leader.dirAngle - last_angle) / (curTime - last_time)
        last_time = curTime
        last_angle =leader.dirAngle

        #pub leader messages
        pub.publish(leader)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
