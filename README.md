# my_experiment
controller and experiment in my paper

多智能体编队（MAS）gazebo平台实验
使用环境：ROS kinetic 以及ubuntu 16.04
使用方法：

1.下载并安装功能包

2.启动gazebo仿真脚本robotworld.launch初始化相关信息；
  该启动器中可以添加机器人，或者更改机器人初始位置等信息；
  
3.待gazebo正常加载完毕之后，分别启动first_order.launch等控制脚本，用于批量启动相应的控制器；
  该启动脚本中可以更改控制脚本follower_gazebo_first.py中的相应参数；
  需关闭当前脚本之后再启动下一次的控制脚本

功能包说明：
  ftcs_ares-master 为仿真相应的模型文件，启动器配置等；
  my_smp_uwb 为launch文件中所用到的相应脚本文件
    my_smp_uwb\src\target 为在该实验中所用到的所有脚本文件，如leader信息发布，一阶二阶控制器
    my_smp_uwb\src\uwbpub 为uwb室内定位设备的移植程序，在仿真实验中未使用，可用于实物实验中 
