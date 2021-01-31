#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h> 
#include <cstring>
#include <cstdlib>

#include "uwbpub/uwbPos.h"

#include "trilateration.cpp"


serial::Serial ser; //声明串口对象
//回调函数
// void write_callback(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO_STREAM("Writing to serial port" <<msg->data);
//     ser.write(msg->data);   //发送串口数据
// }

long hex2int(const std::string& hexStr){
    char *offset;
    if(hexStr.length() > 2){
        if(hexStr[0] == '0' && hexStr[1] == 'x'){
            return strtol(hexStr.c_str(), &offset, 0);
        }
    }
    return strtol(hexStr.c_str(), &offset, 16);
}

int disCheck(int& originDis){
    float k1 = 0;
    float k2 = 0.9889;
    float k3 = -474.15;
    return (int)(originDis-k1)*k2+k3;
}

int main (int argc, char** argv){
    //初始化节点
    ros::init(argc, argv, "uwb_serial");
    //声明节点句柄
    ros::NodeHandle nh("robot2");
    // 订阅主题，并配置回调函数
    // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    // 发布主题
    ros::Publisher read_pub = nh.advertise<uwbpub::uwbPos>("uwbPos_pub", 20);
    // ros::Publisher originData = nh.advertise<std_msgs::Int32>("oriDa", 20);
    try{
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    //指定循环的频率
    ros::Rate loop_rate(20);

    std_msgs::String portRead;
    //获取坐标信息
    int result = 0;
    vec3d anchorArray[4];
    vec3d report;
    int Range_deca[4];
    
    anchorArray[0].x = 0.800; //anchor0.x uint:m
    anchorArray[0].y = 0.000; //anchor0.y uint:m
    anchorArray[0].z = 1.000; //anchor0.z uint:m

    anchorArray[1].x = 0.000; //anchor1.x uint:m
    anchorArray[1].y = 1.000; //anchor1.y uint:m
    anchorArray[1].z = 1.000; //anchor1.z uint:m

    anchorArray[2].x = 0.800; //anchor2.x uint:m
    anchorArray[2].y = 2.000; //anchor2.y uint:m
    anchorArray[2].z = 1.000; //anchor2.z uint:m

    anchorArray[3].x = 0.000; //anchor3.x uint:m
    anchorArray[3].y = 0.000; //anchor3.y uint:m
    anchorArray[3].z = 0.000; //anchor3.z uint:m

    int count = 0;
    int avDis = 0;
    while(ros::ok()){
        if(ser.available()){
            portRead.data = ser.read(ser.available());
            //匹配出字符串
            std::string s = portRead.data;
            int first = s.find("mc");//mc 代表标签-基站距离（优化修正过的数据，用于定位标签）
            //数据预处理
            if(s.length() < 64 || s.length() > 64*3) continue;
            //mc 05 00000621 00000000 00000414 00000000 2ef4 80 0019c2af t6:0
            std::string curData = s.substr(first,63);
            std::string a0 = s.substr(first+6,8);
            int a00 = hex2int(a0);
            std::string a1 = s.substr(first+15,8);
            int a10 = hex2int(a1);
            std::string a2 = s.substr(first+24,8);
            int a20 = hex2int(a2);
            count++;
            avDis = (avDis*(count-1) + a00)/count;
            ROS_INFO("count:%d; avrageDis:%d; origin:%d; check:%d",count,avDis,a00,disCheck(a00));

            Range_deca[0] = disCheck(a00); //tag to A0 distance
            Range_deca[1] = disCheck(a10); //tag to A1 distance
            Range_deca[2] = disCheck(a20); //tag to A2 distance
            Range_deca[3] = 2000; //tag to A3 distance
            
            result = GetLocation(&report, 0, &anchorArray[0], &Range_deca[0]);

            uwbpub::uwbPos curPos;
            curPos.result = result;
            curPos.x = report.x;
            curPos.y = report.y;
            curPos.z = report.z;
            
			read_pub.publish(curPos);
        }
        //处理ROS的信息，比如订阅消息,并调用回调函数
        //ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
