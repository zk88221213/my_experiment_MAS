#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
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
        self.dpub = rospy.Publisher("/"+self.name + '/dp', Float64, queue_size=1)
        self.unpub = rospy.Publisher("/"+self.name + '/un', Float64, queue_size=1)
        self.utpub = rospy.Publisher("/"+self.name + '/ut', Float64, queue_size=1)
        self.p1gjpub = rospy.Publisher("/"+self.name + '/p1gj', Float64, queue_size=1)
        self.p2gjpub = rospy.Publisher("/"+self.name + '/p2gj', Float64, queue_size=1)
        
        self.a1 = rospy.get_param("~a1",1)
        self.a2 = rospy.get_param("~a2",0)
        self.a3 = rospy.get_param("~a3",0)
        self.a4 = rospy.get_param("~a4",0)
        self.rho = rospy.get_param("~rho",0.5)
        
        self.k1 = rospy.get_param("~k1",1)
        self.k2 = rospy.get_param("~k2",1)
        self.k3 = rospy.get_param("~k3",1)
        self.k4 = rospy.get_param("~k4",1)

        self.kc1 = rospy.get_param("~kc1",1)
        self.kc2 = rospy.get_param("~kc2",1)

        self.kg2 = rospy.get_param("~kg2",1)
        self.kg3 = rospy.get_param("~kg3",1)
        self.kg4 = rospy.get_param("~kg4",1)

        self.quan1 = rospy.get_param("~quan1",1)
        self.tao1 = rospy.get_param("~tao1",1)
        self.quan2 = rospy.get_param("~quan2",1)
        self.tao2 = rospy.get_param("~tao2",1)

        self.a0 = rospy.get_param("~a0",1)
        self.yita0 = rospy.get_param("~yita0",0.2)
        #ro is the topology feature result, could caculate by a11-a44
        self.ro = rospy.get_param("~ro",0.3333)
        self.isSat = rospy.get_param("~isSat",0)

        print "%s: a1:%s, a2:%s, a3:%s, a4:%s, rho:%s, k1:%s, k2:%s, k3:%s, k4:%s, a0:%s, yita0:%s, ro:%s, isSat:%s" \
            %(self.name, self.a1,self.a2,self.a3,self.a4,self.rho,self.k1,self.k2,self.k3,self.k4,self.a0,self.yita0,self.ro,self.isSat) 

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
            #"wait message"
            rate.sleep()

        # rospy.loginfo("It maybe start! Caution!")
        
        a1 = self.a1
        a2 = self.a2
        a3 = self.a3
        a4 = self.a4
        rho = self.rho
        
        k1 = self.k1
        k2 = self.k2
        k3 = self.k3
        k4 = self.k4

        kc1 = self.kc1
        kc2 = self.kc2

        kg2 = self.kg2
        kg3 = self.kg3
        kg4 = self.kg4

        quan1 = self.quan1
        tao1 = self.tao1
        quan2 = self.quan2
        tao2 = self.tao2

        vi12 = 0.001
        vb12 = 5*(1-math.tanh(1))

        print "kg2:%s kg3:%s kg4:%s "%(kg2,kg3,kg4)

        a0 = self.a0
        yita0 = self.yita0
        ro = self.ro
        
        move_cmd = Twist()

        lastp = self.getDeltLambda(CurRobot.x, CurRobot.y, rho)
        lastTime = rospy.get_time()
        p1gj = -1.5
        p2gj = -1.5

        while not rospy.is_shutdown():
            # Stop the robot by default
            move_cmd = Twist()
            #last message
            currentTime = rospy.get_time()
            deltTime = currentTime - lastTime
            #rule out 0
            if deltTime==0:
                continue

            pos = np.mat([[CurRobot.x],[CurRobot.y]])
            v = np.mat([[CurRobot.vx],[CurRobot.vy]])
            if self.start_test:
                # cal p and pub p
                curp = self.getDeltLambda(CurRobot.x, CurRobot.y, rho)
                self.ppub.publish(curp)

                N = pos / (rho * np.linalg.norm(pos))
                T = np.mat([[0,1],[-1,0]]) * N
                normT = np.linalg.norm(T)
                deltN = np.mat([[-CurRobot.y**2, CurRobot.x*CurRobot.y],[CurRobot.x*CurRobot.y, -CurRobot.x**2]]) / ( rho * (CurRobot.x ** 2 + CurRobot.y ** 2)**1.5 )
                ps = 0
                pksi = 1 / rho
                esita = CurRobot.xi - leaderInfo.xi
                eYita = ( CurRobot.angSp - a0 * yita0 ) 
                gc = (( (deltN * v).T * v ).tolist())[0][0]
                gksi = pksi * ps
                ga = (( pksi * (N.T * (deltN * v)) / (np.linalg.norm(T))**3 * (- T.T * v) + pksi * 1 / np.linalg.norm(T) * (np.mat([[0, -1],[1, 0]]) * deltN * v).T * v ).tolist())[0][0]
                gb = 0
                dp = (curp - lastp)/deltTime
                self.dpub.publish(dp)
                sita1 = Robot1.xi 
                sita2 = Robot2.xi
                sita3 = Robot3.xi
                sita4 = Robot4.xi
                
                yita1 = Robot1.yita
                yita2 = Robot2.yita
                yita3 = Robot3.yita
                yita4 = Robot4.yita
                
                f = self.getf(curp)
                wn = k2 *( (a1+a2+a3+a4)*CurRobot.xi - a1*sita1 - a2*sita2 - a3*sita3 - a4*sita4 ) \
                    +k3 *( (a1+a2+a3+a4)*CurRobot.yita - a1*yita1 - a2*yita2 - a3*yita3 - a4*yita4 ) \
                    +k4 * a0 * (CurRobot.yita - yita0) 
                   
                dp1gj = tao1*( abs(((kg2+kg3)*f+2*kg4*dp)*(-kc1*f-kc2*dp+gc)) + abs((kg2+kg3)*f+2*kg4*dp)*vb12) - quan1*p1gj
                dp2gj = tao2*( abs((wn+ga+gb*dp)*(k2*ro*esita+k3*ro*eYita)) + abs(pksi*1/normT*(k2*ro*esita+k3*ro*eYita))*vb12 ) - quan2*p2gj
             
                p1gj += dp1gj*deltTime
                p2gj += dp2gj*deltTime 

                b = self.getInterValue(currentTime,1,0.8)

                un = b*self.sat( (-kc1*f-kc2*dp+gc)*p1gj*math.tanh((kg2+kg3)*f+2*kg4*dp)*(-kc1*f-kc2*dp+gc)*p1gj/vi12 \
                                +vb12*p1gj*math.tanh(((kg2+kg3)*f+2*kg4*dp)*vb12*p1gj/vi12)   ,5)
                ut = b*self.sat( -(pksi*1/normT)**(-1)*(wn+ga+gb*dp)*p2gj*math.tanh((wn+ga+gb*dp)*(k2*ro*esita+k3*ro*eYita)*p2gj/vi12) \
                                -vb12*p2gj*math.tanh(pksi*1/normT*(k2*ro*esita+k3*ro*eYita)*p2gj*vb12/vi12)    ,5)

                ux = (( np.mat([1, 0]) * np.linalg.pinv( np.vstack((N.T,T.T)) ) * np.vstack((un,ut)) ).tolist())[0][0]
                uy = (( np.mat([0, 1]) * np.linalg.pinv( np.vstack((N.T,T.T)) ) * np.vstack((un,ut)) ).tolist())[0][0]
                # cal acc
                urx = b * ( ux * math.cos(CurRobot.dirAngle) - uy * math.sin(CurRobot.dirAngle) ) *deltTime 
                ury = b * ( ux * math.sin(CurRobot.dirAngle) + uy * math.cos(CurRobot.dirAngle) ) *deltTime
                # calculate and update
                lastTime = currentTime
                lastp = curp
                #robot model
                #!----!
                move_cmd.linear.x = CurRobot.vx + urx
                move_cmd.linear.y = CurRobot.vy + ury

                self.unpub.publish(un)
                self.utpub.publish(ut)
                self.p1gjpub.publish(p1gj)
                self.p2gjpub.publish(p2gj)

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

    def getf(self,p):
        if p>-1 and p<1 :
            return (math.log(1 + p) - math.log(1 - p) + 3*p)
        elif p<=-1:
            return -100
        elif p>=1:
            return 100
            
    
    def setMax(self, originVal, maxVal):
        if originVal < maxVal or maxVal == 0:
            return originVal
        else:
            return maxVal

    def sat(self,data,up):
        d = abs(data)
        if d <= up:
            return  data
        else:
            return up * np.sign(data)

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
        rospy.loginfo("Stopping the %s",self.name)
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def normPos(self, x, y):
        return math.sqrt(x ** 2 + y ** 2)


if __name__ == '__main__':
    try:
        followTest()
        rospy.spin()
    except:
        rospy.loginfo("Calibration terminated.")