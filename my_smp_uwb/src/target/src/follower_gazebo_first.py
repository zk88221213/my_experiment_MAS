#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist, Point
import math
from target.msg import robotState
import tf
import numpy as np
from std_msgs.msg import Float64

leaderInfo = robotState()
CurRobot = robotState()

Robot1 = robotState()
Robot2 = robotState()
Robot3 = robotState()
Robot4 = robotState()

def getLeader(data):
    global leaderInfo
    leaderInfo = data

def getOwnMes0(data):
    global CurRobot
    CurRobot = data

def getOwnMes1(data):
    global Robot1
    Robot1 = data

def getOwnMes2(data):
    global Robot2
    Robot2 = data

def getOwnMes3(data):
    global Robot3
    Robot3 = data

def getOwnMes4(data):
    global Robot4
    Robot4 = data

class followTest():
    def __init__(self):
        global ns
        global CurRobot
        global leaderInfo

        rospy.init_node('followerTest', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        self.name = rospy.get_param("~name","robot_1")
        self.start_test = True
        self.cmd_vel = rospy.Publisher("/"+self.name + '/cmd_vel', Twist, queue_size=1)
        self.ppub = rospy.Publisher("/"+self.name + '/p', Float64, queue_size=1)
        self.vnpub = rospy.Publisher("/"+self.name + '/vn', Float64, queue_size=1)
        self.vtpub = rospy.Publisher("/"+self.name + '/vt', Float64, queue_size=1)

        self.a = rospy.get_param("~a",0.5)
        self.a1 = rospy.get_param("~a1",1)
        self.a2 = rospy.get_param("~a2",0)
        self.a3 = rospy.get_param("~a3",0)
        self.a4 = rospy.get_param("~a4",0)
        self.rho = rospy.get_param("~rho",0.5)
        
        self.k1 = rospy.get_param("~k1",1)
        self.k2 = rospy.get_param("~k2",1)
        rospy.loginfo("%s param: a=%s,a1=%s,a2=%s,a3=%s,a4=%s,rho=%s,k1=%s,k2=%s",\
                        self.name,self.a,self.a1,self.a2,self.a3,self.a4,self.rho,self.k1,self.k2)
        #get other robot info
        if self.a1 != 0 :
            rospy.Subscriber("/robot_1/robotStates",robotState,getOwnMes1,queue_size=1)
        else:
            Robot1.angSp = 0
            Robot1.xi = 0
        if self.a2 != 0 :
            rospy.Subscriber("/robot_2/robotStates",robotState,getOwnMes2,queue_size=1)
        else:
            Robot2.angSp = 0
            Robot2.xi = 0
        if self.a3 != 0 :
            rospy.Subscriber("/robot_3/robotStates",robotState,getOwnMes3,queue_size=1)
        else:
            Robot3.angSp = 0
            Robot3.xi = 0
        if self.a4 != 0 :
            rospy.Subscriber("/robot_4/robotStates",robotState,getOwnMes4,queue_size=1)
        else:
            Robot4.angSp = 0
            Robot4.xi = 0
            
        rospy.Subscriber("/leader",robotState,getLeader,queue_size=1)
        #get self info 
        rospy.Subscriber("/"+self.name + "/robotStates",robotState,getOwnMes0,queue_size=1)

        rate = rospy.Rate(50)
        while CurRobot.x == 0:
            print "wait message"
            rate.sleep()

        rospy.loginfo("It maybe start! Caution!")
        
        a = self.a
        a1 = self.a1
        a2 = self.a2
        a3 = self.a3
        a4 = self.a4
        rho = self.rho

        k1 = self.k1
        k2 = self.k2

        move_cmd = Twist()
        startTime = rospy.get_time()
        while not rospy.is_shutdown():
            # Stop the robot by default
            move_cmd = Twist()

            pos = np.mat([[CurRobot.x],[CurRobot.y]])

            if self.start_test:
                delLambda = self.getDeltLambda(CurRobot.x, CurRobot.y, rho)
                N = pos / (rho * np.linalg.norm(pos))
                T = np.mat([[0,1],[-1,0]]) * N
                ps = 0
                pksi = 1 / rho
                deltTime = rospy.get_time() - startTime
                # print "startTime: %s curTime:%s delt:%s" %(startTime,rospy.get_time(),deltTime)
                targetXi = leaderInfo.xi
                deltaXi = ( CurRobot.xi - targetXi ) 

                # f00 = math.log(1 + delLambda) - math.log(1 - delLambda) + 5 * delLambda
                sita1 = Robot1.xi
                sita2 = Robot2.xi
                sita3 = Robot3.xi
                sita4 = Robot4.xi

                wn = a1*(deltaXi - sita1 + targetXi) + a2*(deltaXi - sita2 + targetXi) + a3*(deltaXi - sita3 + targetXi) + a4*(deltaXi - sita4 + targetXi)

                self.ppub.publish(delLambda)
                f=self.getf(delLambda)
                
                vn = self.sat(k1 * f + 5 * math.tanh(f))
                vt = self.sat(np.linalg.norm(T) * (pksi ** (-1)) * ( a - k2 *wn - 3 * math.tanh(deltaXi) ))
                # print "curXi:%s sita1:%s sita2:%s sita3:%s sita4:%s leader:%s" %(CurRobot.xi, Robot1.xi, Robot2.xi, Robot3.xi, Robot4.xi, leaderInfo.xi)
                self.vnpub.publish(vn)
                self.vtpub.publish(vt)
                b = self.getInterValue(deltTime, 1, 0.8)
                
                vx =b * (( np.mat([1, 0]) * np.linalg.pinv( np.vstack((N.T,T.T)) ) * np.vstack((vn,vt)) ).tolist())[0][0]
                vy =b * (( np.mat([0, 1]) * np.linalg.pinv( np.vstack((N.T,T.T)) ) * np.vstack((vn,vt)) ).tolist())[0][0]

                # it work for nonholometic model
                # l = 0.1
                # v = math.cos(CurRobot.dirAngle) * vx + math.sin(CurRobot.dirAngle) * vy
                # w = - 1 / l * math.sin(CurRobot.dirAngle) * vx + 1 / l * math.cos(CurRobot.dirAngle) * vy
                #
                vrx = vx * math.cos(CurRobot.dirAngle) - vy * math.sin(CurRobot.dirAngle)
                vry = vx * math.sin(CurRobot.dirAngle) + vy * math.cos(CurRobot.dirAngle)

                # print "vn:%3f vt:%3f vx: %3f vy: %3f vrx:%3f vry:%3f \ndeltLambda:%3f deltTime:%3f deltaXi:%3f" % (vn,vt,vx,vy,vrx,vry,delLambda,deltTime,deltaXi)
                # calculate and publish
                move_cmd.linear.x = self.setMax(vrx, 0)
                move_cmd.linear.y = self.setMax(vry, 0)
            else:
                self.position = CurRobot
                x_start = self.position.x
                y_start = self.position.y
                # Stop the robot
            self.cmd_vel.publish(move_cmd)
            rate.sleep()

    def getInterValue(self, time, period, controlTime):
        if  (time % period) <= controlTime:
            return 1
        else:
            return 0
    
    def setMax(self, originVal, maxVal):
        if originVal < maxVal or maxVal == 0:
            return originVal
        else:
            return maxVal

    def sat(self,data):
        d = abs(data)
        if d <= 0.2:
            return  data
        else:
            return 0.2 * np.sign(data)

    def LineFunction(self, curArc, lamb=0, a=1, b=1):
        tPos = Point()
        tPos.x = (1 - lamb) * a * math.cos(curArc)
        tPos.y = (1 - lamb) * b * math.sin(curArc)
        return tPos

    def getDeltLambda(self, x, y, rho, a=1, b=1):
        p = 1 - 1/ rho * math.sqrt(x**2 + y**2)
        return p

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def normPos(self, x, y):
        return math.sqrt(x ** 2 + y ** 2)

    def getf(self,p):
        if p>-1 and p<1:
            return (math.log(1 + p) - math.log(1 - p) + 3*p)
        elif p<=-1:
            return -100
        elif p>=1:
            return 100

if __name__ == '__main__':
    try:
        followTest()
        rospy.spin()
    except:
        rospy.loginfo("Calibration terminated.")