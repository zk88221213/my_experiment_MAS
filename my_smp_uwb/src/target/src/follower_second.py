#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
from target.msg import robotState
import tf

import numpy as np

ns = '/robot_1'

leaderInfo = robotState()
CurRobot = robotState()

rho = 0.5

def getLeader(data):
    global leaderInfo
    leaderInfo = data


def getOwnMes(data):
    global CurRobot
    CurRobot = data


class followTest():
    def __init__(self):
        global ns
        # Give the node a name
        rospy.init_node('followerTest', anonymous=False)
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        # How fast will we check the odometry values?
        self.rate = 20
        r = rospy.Rate(self.rate)
        # Set the goal
        self.start_test = True

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher(ns + '/cmd_vel', Twist, queue_size=2)

        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        # it's needs to be correct in multirobot system
        self.base_frame = ns + '/base_footprint'
        # The odom frame is usually just /odom
        self.odom_frame = ns + '/odom'

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        # Make sure we see the odom and base frames, just wait for messages
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))

        rospy.loginfo("It maybe start! Caution!")

        self.position = Point()
        # Get the starting position from the tf transform between the odom and base frames
        self.position = self.get_position()

        x_start = self.position.x
        y_start = self.position.y

        rospy.Subscriber("leader",robotState,getLeader,queue_size=1)
        rospy.Subscriber("/robot_1/robotStates",robotState,getOwnMes,queue_size=1)
        while leaderInfo.x == 0 or CurRobot.x == 0:
            r.sleep()
        scaleNum = 0.5
        # test point
        # CurRobot.x = 0
        # CurRobot.y = 1
        # CurRobot.xi = 0.1
        # leaderInfo.x = 1
        # leaderInfo.y = 1
        # leaderInfo.xi = 0.5

        move_cmd = Twist()

        global leaderInfo
        global lamb
        global rho

        a0 = 1
        yita0 = 0.1
        k1 = 1
        k2 = 10
        k3 = 5
        k4 = 1
        ro = 1/6
        while not rospy.is_shutdown():
            # Stop the robot by default
            move_cmd = Twist()
            # drop zero point
            rospy.loginfo("CurRobot: (%s,%s) xi: %s", CurRobot.x, CurRobot.y, CurRobot.xi)

            if self.start_test:
                pos = np.mat([CurRobot.x],[CurRobot.y])
                v = np.mat([CurRobot.vx],[CurRobot.vy])
                p = self.getDeltLambda(CurRobot.x, CurRobot.y, rho)
                N = pos / (rho * self.norm(CurRobot.x,CurRobot.y))
                T = np.mat([0, 1],[-1, 0]) * N
                normT = self.norm((T.tolist)[0][0],(T.tolist)[0][1])
                deltN = np.mat([-CurRobot.y**2, CurRobot.x * CurRobot.y],[CurRobot.x * CurRobot.y, - CurRobot.x**2 ]) / (rho * (CurRobot.x**2+CurRobot.y**2)**1.5)
                ps = 0
                pksi = 1 / rho
                yita = CurRobot.xi
                yita1 = yita
                sita = CurRobot.angSp
                sita1 = sita
                eyita = yita - a0 * yita0
                gc = ((  (deltN * v).T * v  ).tolist())[0][0]
                gksi = pksi * ps
                ga =(( pksi * (N.T * (deltN * v)) / (normT) ** 3 * (- T.T * v) + pksi * 1 / normT * ( np.mat([0, -1],[1, 0]) * deltN * v).T * v ).tolist())[0][0]
                gb = 0
                dp = (( - (N.T * v) ).tolist())[0][0]

                f = self.getflam(p)
                ut = (normT * pksi ** (-1))*(-ga-k2*( a1*sita - a1*sita1 )-k3*( a1*yita - a1*yita1 )-k4*a0*(yita-yita0)-gb*dp-k1*f)
                un = p + dp + gc + k2 * eyita * ro * gksi + f
                
                ux = (( np.mat([1, 0]) * np.linalg.inv(np.mat(N.T, T.T)) * np.mat([[un], [ut]]) ).tolist())[0][0]
                uy = (( np.mat([1, 0]) * np.linalg.inv(np.mat(N.T, T.T)) * np.mat([[un], [ut]]) ).tolist())[0][0]

                vi = self.norm(v.tolist[0][0],v.tolist[0][1])
                if vi == 0 :
                    #give defalut value in case of zero break
                    vi = 0.05
                r = math.cos(CurRobot.dirAngle) * ux + math.sin(CurRobot.dirAngle) * uy
                w = -math.sin(CurRobot.dirAngle)/vi * ux + math.cos(CurRobot.dirAngle)/vi * uy
                # calculate and publish
                move_cmd.linear.x = 0
                move_cmd.angular.z = 0
            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
                # Stop the robot
            self.cmd_vel.publish(move_cmd)
            r.sleep()

    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            # rospy.loginfo(trans)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception:%s,%s", self.odom_frame, self.base_frame)
            return

        return Point(*trans)

    def LineFunction(self, curArc, lamb=0, a=1, b=1):
        tPos = Point()
        tPos.x = (1 - lamb) * a * math.cos(curArc)
        tPos.y = (1 - lamb) * b * math.sin(curArc)
        return tPos

    def getDeltLambda(self, x, y, rho):
        p = 1 - 1/ rho * (x**2 + y**2)**0.5
        return p


    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def norm(self, x, y):
        return math.sqrt(x ** 2 + y ** 2)

    def getflam(self,lamb):
        if lamb < -1 :
            return -20
        elif lamb > 1:
            return 20
        else:
            return math.log10(1 + lamb) - math.log10(1 - lamb) + lamb 
        


if __name__ == '__main__':
    try:
        followTest()
        rospy.spin()
    except:
        rospy.loginfo("Calibration terminated.")