# ROS编程

本部分内容主要参考《机器人移动开发技术（激光slam版）》

代码：[SGL/mrobot_book (gitee.com)](https://gitee.com/mrobotit/mrobot_book)

## 1.ROS通信机制

通常来说，一个机器人项目是需要多进程协同工作的，所以大部分进程间需要数据交互，**进程间通信是构建复杂机器人的基础**

ROS通信的主要方式是

![image-20230627153617597](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230627153617597.png)

### **1.1 节点**（Node)

在ROS中，最小的进程单元是节点，可执行文件在运行后就是一个进程，那么这个进程就是ROS中的节点

### **1.2节点管理器（Master)**

Master在相当于管理中心，管理各个节点。

Node在启动时，需要在Master进行注册，然后Master会把Node纳入管理

在Node建立连接后，Master的任务也就随之完成，即使关闭Master,Node的通信还可以继续进行

### 1.3 Node与Master相关命令

```
roscore                           //启动ros master
rosrun[pkg_name][node_name]       //启动一个node
rosnode list                      //查看当前运行的node信息
rosnode info[node_name]           //显示node的详细信息
rosnode kill[node_name]           //结束node
roslaunch[pkg_name][file_name.launch] //启动master和多个node
```

### 1.4 ROS的通信方式

ROS通信是ROS最核心的概念

1.Topic话题模式

2.Service服务模式

3.Parameter Service (参数服务器)

4.Actionlib(动作库)

### 1.5 Topic话题通信（单向，异步）

话题通信：发布者，订阅者，Master

![image-20230627160537863](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230627160537863.png)

对于实时性、周期性的消息，Topic通信是最佳的方式，它是一种单向的通信方式。

**初始化过程：**

​         Publisher发布者节点和Subscriber订阅节点都要到Master进行注册；

​        Publisher发布Topic话题,Subscribe在节点管理器Master指挥下,会订阅Topic话题，建立节点与节点间的通信

​        （注意：该过程是单向的）

**回调过程**

​       Subscriber接收消息后会进行处理，该过程是回调（Callback);回调就是提前定义一个处理函数，当Node接收了新消息后，进程会触发这个处理函数，该函数会对该消息进行处理。



#### 以摄像机画面发布、处理、显示举例

![image-20230627162240677](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230627162240677.png)

摄像机的拍摄程序是一个Node(使用椭圆圈表示，记为NodeCamera),当NodeCamera启动后，作为发布器Publisher，开始发布Topic话题"/camera_rgb"

;同时NodeImg Process作为订阅者Subscriber,订阅“/camera_rgb".并经过节点管理器的中介管理；同理图像显示节点NodeImgDsp也能与NodeImgProcess建立连接



数据传输到话题上，会在Topic缓冲区进行存储；但是一旦超过缓冲区信息数量，那么最早的信息就会被丢弃

**异步通信**

其中，NodeCamera每发布一次消息，就会执行下一步动作，发出的消息是否被接收与它无关；而图像处理程序NodeImgProcess只接收和处理"/camera_rgb"话题的消息，而不管是谁发来的；两者不存在协同工作，**这种通信方式称为异步通信**

**常用命令行工具**

```
rostopic list 话题列表

rostopic pub -r 10 /turtle1/cmd_vel  发布内容

rostopic echo /turtle1/cmd_vel 打印话题内容

rostopic  type /turtle1/cmd_vel 打印消息类型

rosmsg show /turtle1/Pose 打印消息类型的具体格式
```



#### **新建包命令**

```c++
catkin_create_pkg learning roscpp rospy std_msgs geometry_msgs turtlesim
```

**velocity_publisher.cpp**

```c++
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
int main(int argc,char**argv)
{
    ros::init(argc,argv,"velocity_publisher");
    ros::NodeHandle nh;
    ros::Publisher turtle_vel_pub =nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
    //第一个参数是话题，第二个参数是消息队列大小
    ros::Rate loop_rate(10);//10Hz

    while(ros::ok())
    {
        geometry_msgs::Twist vel_msg; //ROS预定义消息类型
        vel_msg.linear.x=0.5;
        vel_msg.angular.z=0.2;
        turtle_vel_pub.publish(vel_msg);
        ROS_INFO("Publish turtle velocity command[%0.2f m/s,%0.2f rad/s]",vel_msg.linear.x,vel_msg.angular.z);
        loop_rate.sleep();
        //loop_rate.sleep() 是以阻塞的方式来实现循环的控制。
        //它会使当前线程处于休眠状态，直到达到设定的频率，然后再继续执行下一个循环迭代。这样可以确保循环的运行速率接近预期的频率。
    }

    return 0;
}
```

**pose_subscriber.cpp**

```c++
#include<ros/ros.h>
#include<turtlesim/Pose.h>
void poseCallback(const turtlesim::poseCallback::ConstPtr&msg)
{
    ROS_INFO("Turtle pose:x:%0.6f,y:%0.6f",msg->x,msg->y);
}
int main(int argc,char**argv)
{
   ros::init(argc,argv,"pose_subscriber");
   ros::NodeHandle nh;
   ros::Subscriber pose_sub=nh.subscribe("/turtle1/pose",10,poseCallback);
   ros::spin();//循环等待回调函数
    
    return 0;
}
```

本教程github源码https://github.com/Grizi-ju/ros_program

#### 话题通信示例二：订阅里程计话题查看机器人位姿

仿真环境 TIANBOT.git

git clone 这三个包

![image-20221107195847192](/home/su/.config/Typora/typora-user-images/image-20221107195847192.png)

```c++

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

void doMsg(const nav_msgs::Odometry::ConstPtr& odom){

       geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odom -> pose.pose.position.x;
    this_pose_stamped.pose.position.y = odom -> pose.pose.position.y;
    this_pose_stamped.pose.position.z = odom -> pose.pose.position.z;
 
    this_pose_stamped.pose.orientation = odom -> pose.pose.orientation;

    ROS_INFO("odom:%.3lf,%.3lf,%.3lf",this_pose_stamped.pose.position.x,this_pose_stamped.pose.position.y,this_pose_stamped.pose.position.z);
    // ROS_INFO("我听见:%s",(*msg_p).data.c_str());
}
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    //2.初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"listener");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;

    //4.实例化 订阅者 对象
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/tianbot_mini/odom",10,doMsg);
    //5.处理订阅的消息(回调函数)

    //     6.设置循环调用回调函数
    ros::spin();//循环读取接收的数据，并调用回调函数处理

    return 0;
}

```



#### 话题通信示例三：使小乌龟转圈前进



#### 话题通信示例四：使用PID算法使小车运动

网址

https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/sim_install.html



https://f1tenth.org/learn.html

安装依赖

```
sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server
```

```
cd ~/catkin_ws/src
git clone https://github.com/f1tenth/f1tenth_simulator.git
```

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

```
roslaunch f1tenth_simulator simulator.launch
```



**PID简单原理**

![image-20221108154753573](/home/su/.config/Typora/typora-user-images/image-20221108154753573.png)

简单代码

```python
#! /usr/bin/env python
# -*- coding: utf-8 -*
# https://linklab-uva.github.io/autonomousracing/assets/files/assgn4-print.pdf
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

#获取激光雷达测量的射线距离
FILTER_VALUE = 10.0
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis

def wall_following_callback(data):
    
    # 1、计算公式：根据雷达信息，得到离墙距离AB
    THETA = np.pi / 180 * 60
    b = get_range(data, -45)
    a = get_range(data, -45 + np.rad2deg(THETA))
    alpha = np.arctan((a * np.cos(THETA) - b) / (a * np.sin(THETA)))
    AB = b * np.cos(alpha)

    # 2、用PID思想来调整，保持AB为1
    if AB > 1.0:
        steering_angle = -0.1
    else:
        steering_angle = 0.1

    speed = 1

    # 3、把速度、转向角发布出去
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle=steering_angle 
    drive_msg.drive.speed=speed
    drive_pub.publish(drive_msg)

if __name__ == '__main__': 
  try:
    rospy.init_node("wall_following")
    scan_sub = rospy.Subscriber('/scan', LaserScan, wall_following_callback)
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
```



PID算法实现全部（需要深究  网址：https://f1tenth.org/learn.html）

```c++
/*
 * 服务通信：该例程在F1TENTH仿真环境下，执行/car_command服务，服务数据类型std_srvs/Trigger
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


ros::Publisher pub_drive;
bool pubCommand = false;
ackermann_msgs::AckermannDriveStamped vel_drive;

// service回调函数，输入参数req，输出参数res
bool commandCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	  pubCommand = !pubCommand;
    // 显示请求数据
    ROS_INFO("Publish turtle velocity command [%s]", pubCommand==true?"Yes":"No");
	  // 设置反馈数据
  	res.success = true;
  	res.message = "Change turtle command state!";

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_command_server");
    ros::NodeHandle n;

    // 创建一个名为/car_command的server，注册回调函数commandCallback
    ros::ServiceServer command_server = n.advertiseService("/car_command", commandCallback);

    // 创建一个Publisher，发布名为/drive的topic，阿克曼速度消息类型，队列长度10
    pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);

    // 循环等待回调函数
    ROS_INFO("Ready to receive car command.");

    // 设置循环的频率
    ros::Rate loop_rate(10);
    
    while(ros::ok())
	{
	  // 查看一次回调函数队列
     ros::spinOnce();

		// 如果为true，则发布速度指令, 否则停止
	  	if(pubCommand)
		{
			vel_drive.drive.speed = 1.0;	
			pub_drive.publish(vel_drive);
		}
  	else
		{
			vel_drive.drive.speed = 0.0;
			pub_drive.publish(vel_drive);
		}
      loop_rate.sleep();
	}

    return 0;
}
```



### 话题通信总结：话题通信案例总结

![image-20221108165919568](/home/su/.config/Typora/typora-user-images/image-20221108165919568.png)



### 1.6 服务通信（双向，同步）

有时候像话题Topic单向通信并不能满足实际需求；ROS提供Service服务模式

#### 原理

Service服务包括两个方面：一部分是请求方（Client),另一部分是应答方/服务提供方（Server).服务通信是**双向的**，Client发送一个请求给Server,要等待Server处理并反馈一个应答Reply,通过这种”**请求——应答**“的机制完成服务通信

![image-20230628100857412](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230628100857412.png)

通过图片，可以看到Node B(应答方)，提供一个服务的接口，一般会用String类型来指定Service服务的名称，类似于Topic,

#### Topic通信和Service通信的异同

![image-20230628102016181](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230628102016181.png)

#### 1.服务通信简介--生成多只乌龟

![image-20221108170356506](/home/su/.config/Typora/typora-user-images/image-20221108170356506.png)

rosservice list 服务列表

rosservice type/spawn 消息类型

rossrv info turtlesim/Spawn消息格式

rosservice call/clear 清除功能（可以清除小乌龟走过的线路）

rosservice call/kill"name:'turtle1'"杀死小乌龟

rosservice call/turtle1/set_pen"{r:255,g:0,b:0,width:10,'off':0)}"设置小乌龟路线颜色

rosservice call/spawn"x:0.0   生成新的小乌龟

y:0.0

theta:0.0

name:""



生成新的乌龟

```c++
#include<ros/ros.h>>
#include<turtlesim/Spawn.h>

int main(int argc,char**argv)
{
   ros::init(argc,argv,"turtle_spawn");
   ros::NodeHandle n;

   //发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
   ros::service::waitForService("/spawn");
   ros::ServiceClient add_turtle_client =n.serviceClient<turtlesim::Spawn>("/spawn");

   //初始化turtlesim::Spawn的请求数据
     turtlesim::Spawn srv;
     srv.request.x=2.0;
     srv.request.y=2.0;
     srv.request.name="turtle2";

     //请求服务调用
     ROS_INFO("Call service to spawn turtle[x:0.6f,y:0.6f,name:%s]",srv.request.x,srv.request.y,srv.request.name.c_str());
     add_turtle_client.call(srv);

     //显示器显示服务调用结束
     ROS_INFO("Spawn turtle successful [name:%s]",srv.response.name.c_str());
     

    return 0;
}

```



#### 2.服务通信--触发控制小车

```c++
/*
 * 服务通信：该例程在F1TENTH仿真环境下，执行/car_command服务，服务数据类型std_srvs/Trigger
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


ros::Publisher pub_drive;
bool pubCommand = false;
ackermann_msgs::AckermannDriveStamped vel_drive;

// service回调函数，输入参数req，输出参数res
bool commandCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	  pubCommand = !pubCommand;
    // 显示请求数据
    ROS_INFO("Publish turtle velocity command [%s]", pubCommand==true?"Yes":"No");
	  // 设置反馈数据
  	res.success = true;
  	res.message = "Change turtle command state!";

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_command_server");
    ros::NodeHandle n;

    // 创建一个名为/car_command的server，注册回调函数commandCallback
    ros::ServiceServer command_server = n.advertiseService("/car_command", commandCallback);

    // 创建一个Publisher，发布名为/drive的topic，阿克曼速度消息类型，队列长度10
    pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);

    // 循环等待回调函数
    ROS_INFO("Ready to receive car command.");

    // 设置循环的频率
    ros::Rate loop_rate(10);
    
    while(ros::ok())
	{
	  // 查看一次回调函数队列
     ros::spinOnce();

		// 如果为true，则发布速度指令, 否则停止
	  	if(pubCommand)
		{ 
			vel_drive.drive.speed = 1.0;	
			pub_drive.publish(vel_drive);
		}
  	else
		{
			vel_drive.drive.speed = 0.0;
			pub_drive.publish(vel_drive);
		}
      loop_rate.sleep();
	}

    return 0;
}
```

  ![image-20221108201421336](/home/su/.config/Typora/typora-user-images/image-20221108201421336.png)



### 1.7 参数服务器（Parameter Service)

Topic和Service服务模式的消息存储在各自独立的命名空间内，只可以**局部访问**

参数服务器的节点参数值可以**全局共享访问**

参数服务器通过维护数据字典来存储各种参数和配置，数据字典是键值对列表

![image-20230628105828408](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230628105828408.png)

参数命令行

```
rosparam list                   //列出当前的所有参数
rosparam get param_key          //显示某个参数值
rosparam set param_key param_value //设置某个参数值
rosparam dump file_name         //保存参数到文案
rosparam load file_name         //从文件读取参数
rosparam delete param_key       //删除参数
```



### 1.8 action通信

#### Action通信简介

三者区别：

话题通信：单向通信，发布后需要订阅

服务通信：请求一次任务，响应一次状态信息

Action通信：导航过程中连续反馈状态信息，导航终止时再返回执行结果

基于功能包actionlib实现通信

![image-20230628111111869](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230628111111869.png)

​	**Action通信主要是弥补Service通信的一个不足**，因为如果利用Sevice服务通信，当机器人执行一个较长时间的任务时，Client会长时间接收不到消息，从而导致通信阻塞。Actionlib通过action通信机制，适合较长时间的通信，它也类似于Service服务的请求响应通信机制，不同点在于Action具有反馈机制，可以不断反馈任务的实施进度。

####  工作原理

Action是Client-Server模式，是一种双向的通信模式，通信双方在ROS Action Protocol下通过消息进行数据的交流通信.Client和Server为用户提供一个简单的API来请求目标（在客户端）或者通过函数调用和回调来执行目标（在服务器端）

1.1 move_base

![image-20221108202956098](/home/su/.config/Typora/typora-user-images/image-20221108202956098.png)

```c
sudo apt-get install ros-move-base-msgs 
sudo apt-get install ros-melodic-dwa-local-planner
sudo apt-get install ros-melodic-gmapping
```



## 2 坐标变换TramsForm（TF）

坐标变换包括位置和姿态两个方面的变换

ROS中机器人每一个部件称为Link,每一个Link都有一个坐标系（Frame),TF通过树状结构维护坐标系之间的关系，依靠Topic话题通信机制来持续发布不同Link之间的坐标关系

![image-20230628114859344](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230628114859344.png)

通过这幅图其实能够很容易理解amcl定位的作用，一开始/map和/odom坐标系是高度重合的，但是由于odom是测不准的，那么AMCL就会将里程计推算出来的位姿于真实位姿之间的误差，通过里程计坐标系漂移的方式补偿回去；即里程计odom本身相对于map坐标系会进行漂移，此时odom与map坐标系就不重合了





## 3.定位与自主导航

在给定某目标位置时，机器人完成自主导航需要以下三个步骤：

（1）重新确定自己在地图的位置

（2）全局路径规划

（3）局部路径规划，同时还需要考虑避障





#### 3.1 move_base简介

![image-20230629113959284](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230629113959284.png)

![image-20230629114223637](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230629114223637.png)

move_base插件主要包括以下三个方面：

（1）全局路径规划：carrot_planner、navfn和global_planner插件；carrot_planner实现了较为简单的全局路径规划，navfn实现了Dijkstra和A*全局路径规划，global_planner可以看作是navfn的改进版

（2）局部路径规划：base_local_planner和dwa_local_planner插件。base_local_planner实现了Trajectory Rollout和DWA两种局部路径规划，dwa_local_planner可以看作base_local_planner的改良版

（3）恢复行为部分（Recovery Behavior)。如果规划失败就会恢复行为；其中clear_costmap_recovery实现了清除代价地图的恢复行为，rotate_recovery实现了旋转的恢复行为，move_slow_and_clear实现了缓慢移动的恢复行为。



move_base的工作主线程

![image-20230629145958625](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230629145958625.png)





#### 3.2 全局路径规划插件编写

参考地址：[导航/教程/编写全局路径规划器作为 ROS 中的插件 - ROS 维基](http://wiki.ros.org/navigation/Tutorials/Writing A Global Path Planner As Plugin in ROS/)

##### 3.2.1 C++知识补充---虚函数

  编译器处理虚函数的方法是：给每个对象添加一个隐藏成员。隐藏成员中保存了一个指向函数地址数组的指针（**vptr**)。这种数组称为虚函数表(virtual function table **vtbl**)。**虚函数表**中存储了为类对象进行声明的虚函数地址。

   例如,基类对象包含一个指针，**该指针指向基类中的所有虚函数表的地址表**。派生类对象中将**包含一个指向独立地址表的指针**。如果派生类提供了虚函数的新定义，该虚函数表将保存新函数的地址;如果派生类没有重新定义函数，该vtbl将保存原始函数版本。通过图片可以很容易理解

![image-20230630163107170](https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230630163107170.png)

**我的理解：vptr指针指向虚函数表，虚函数表中包含每一个虚函数的地址；在派生类中，如果虚函数重新定义，那么它的虚函数表中对应函数的地址需要发生改变，如果没有重新定义，则继续沿用基类中该虚函数的地址**

使用虚函数时，内存与执行速度有一定成本：

1. 每个对象内存空间都会增大，增大量为虚函数实现的存储地址空间
2. 每一个类，编译器都要创建一个虚函数地址表
3. 每个函数调用，都需要执行一项额外的操作，即到表中查找地址



##### 3.2.2 编写路径规划器类

###### 3.3.1 类标头

第一步是为路径规划器编写一个新类，该类遵循 [nav_core：：BaseGlobalPlanner](http://docs.ros.org/api/nav_core/html/classnav__core_1_1BaseGlobalPlanner.html)。在 [carrot_planner.h](http://docs.ros.org/hydro/api/carrot_planner/html/carrot__planner_8h_source.html) 中可以找到类似的例子作为参考。为此，您需要创建一个头文件，我们将在本例中调用该文件 global_planner.h。我将只介绍添加插件的最少代码，这是添加任何全局规划器的必要和常见步骤。最小头文件定义如下：

```c++
 /** include the libraries you need in your planner here */
   2  /** for global path planner interface */
   3  #include <ros/ros.h>
   4  #include <costmap_2d/costmap_2d_ros.h>
   5  #include <costmap_2d/costmap_2d.h>
   6  #include <nav_core/base_global_planner.h>
   7  #include <geometry_msgs/PoseStamped.h>
   8  #include <angles/angles.h>
   9  #include <base_local_planner/world_model.h>
  10  #include <base_local_planner/costmap_model.h>
  11 
  12  using std::string;
  13 
  14  #ifndef GLOBAL_PLANNER_CPP
  15  #define GLOBAL_PLANNER_CPP
  16 
  17  namespace global_planner {
  18 
  19  class GlobalPlanner : public nav_core::BaseGlobalPlanner {
  20  public:
  21 
  22   GlobalPlanner();
  23   GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  24 
  25   /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  26   void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  27   bool makePlan(const geometry_msgs::PoseStamped& start,
  28                 const geometry_msgs::PoseStamped& goal,
  29                 std::vector<geometry_msgs::PoseStamped>& plan
  30                );
  31   };
  32  };
  33  #endif
  34 
```

其中nav_core/base_global_planner.h（[nav_core: nav_core::BaseGlobalPlanner Class Reference (ros.org)](http://docs.ros.org/en/api/nav_core/html/classnav__core_1_1BaseGlobalPlanner.html)）**包含了一般的虚函数**，需要在添加的插件中实现

其中

<img src="https://cdn.jsdelivr.net/gh/su-ron/myBlogResource/ROS%E5%AD%A6%E4%B9%A0/picture/image-20230630154811158.png" alt="image-20230630154811158"  />



需要 [costmap_2d/costmap_2d_ros.h](http://wiki.ros.org/costmap_2d) 和 [costmap_2d/costmap_2d.h](http://wiki.ros.org/costmap_2d) 才能使用 [costmap_2d：：Costmap2D](http://docs.ros.org/hydro/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html) 类，该类将被路径规划器用作**输入地图**。当定义为插件时，路径规划器类将自动访问此映射。无需订阅 `costmap2d` 即可从 ROS 获取costmap。