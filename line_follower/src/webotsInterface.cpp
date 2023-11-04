/**********************************************************
**  Description: 此文件为 Webots仿真接口程序 源文件
**  Author     : ssc
**********************************************************/
#include "line_follower/webotsInterface.hpp"

using namespace cv;
using std::string;

#define TIME_STEP   32                        // 时钟
#define ROBOT_NAME  "/tianbot_mini/"           // ROBOT名称
#define NMOTORS 2                             // 电机数量

ros::NodeHandle *n;

ros::ServiceClient time_step_client;          // 时钟通讯客户端
webots_ros::set_int time_step_srv;            // 时钟服务数据

ros::ServiceClient set_velocity_client;       // 电机速度通讯service客户端
webots_ros::set_float set_velocity_srv;       // 电机速度服务数据
ros::ServiceClient set_position_client;       // 电机位置通讯service客户端
webots_ros::set_float set_position_srv;       // 电机位置服务数据
ros::Publisher speed_puber;                     // 话题发布

double speeds[NMOTORS]={0.0,0.0};             // 两个电机速度值，范围为 0～100
float linear_temp=0, angular_temp=0;          // 暂存的线速度和角速度
static const char *motorNames[NMOTORS] ={"left_motor", "right_motor"}; // 匹配电机名

ros::ServiceClient set_camera_client;         // 摄像头service客户端
webots_ros::set_int camera_srv;               // 摄像头service数据
ros::Subscriber sub_camera_img;               // 订阅摄像头图像话题

ros::ServiceClient keyboardEnableClient;      // 键盘控制service客户端
webots_ros::set_int keyboardEnablesrv;        // 键盘控制service数据


/*
* 函数功能：webots摄像头数据回调函数
*/
void cameraCB(const sensor_msgs::Image::ConstPtr &value){
    static int count;
    if (count % 100 == 0)
    {
        ROS_INFO("已经获取到第%d张图像", count);
    }
    ++count;
    // // 将webots提供的uchar格式数据转换成Mat
    // Mat img_rgb = Mat(value->data).clone().reshape(4, 64);
    // Mat img;
    // img = ImageProcessFun(img_rgb);
    // FindPoints(img);
}


/*
* 函数功能：初始化Webots仿真环境
*/
void webotsInit()
{
    setlocale(LC_CTYPE,"zh_CN.utf8");   // 控制台设置输出中文，否则就是乱码
    // 服务订阅time_step和webots保持同步
    time_step_client = n->serviceClient<webots_ros::set_int>("tianbot_mini/robot/time_step");
    time_step_srv.request.value = TIME_STEP;

    // 初始化电机
    for (int i = 0; i < NMOTORS; ++i) {
        // position速度控制时设置为缺省值INFINITY   
        set_position_client = n->serviceClient<webots_ros::set_float>(string(ROBOT_NAME) + string(motorNames[i]) + string("/set_position"));   
        set_position_srv.request.value = INFINITY;
        if (set_position_client.call(set_position_srv) && set_position_srv.response.success)     
            ROS_INFO("电机%s位置设置为INFINITY", motorNames[i]);   
        else     
            ROS_ERROR("电机%s的set_position服务无响应", motorNames[i]);
        // velocity初始速度设置为0   
        set_velocity_client = n->serviceClient<webots_ros::set_float>(string(ROBOT_NAME) + string(motorNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = 0.0;   
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)     
            ROS_INFO("电机%s速度设置为0。", motorNames[i]);   
        else     
            ROS_ERROR("电机%s的set_velocity服务无响应", motorNames[i]);
    }   
    speed_puber = n->advertise<nav_msgs::Odometry>("/vel",10);        // /vel 话题，用于配置odom

    // 初始化摄像头
    set_camera_client = n->serviceClient<webots_ros::set_int>(string(ROBOT_NAME)+string("camera/enable"));
    camera_srv.request.value = TIME_STEP;
    if (set_camera_client.call(camera_srv) && camera_srv.response.success) {
        sub_camera_img = n->subscribe(string(ROBOT_NAME)+string("camera/image"), 1, cameraCB);
        ROS_INFO("摄像机启动成功");
    } else {
        if (!camera_srv.response.success)
        ROS_ERROR("摄像机启动失败");
    }
}


/*
* 函数功能：键盘控制回调函数
*/
void keyboardDataCB(const webots_ros::Int32Stamped::ConstPtr &value)
{
    switch (value->data)
    {
        // 左转
        case 314:
            angular_temp+=0.1;
            break;
        // 前进
        case 315:
            linear_temp += 0.1;
            break;
        // 右转
        case 316:
            angular_temp-=0.1;
            break;
        // 后退
        case 317:
            linear_temp-=0.1;
            break;
        // 停止
        case 32:
            linear_temp = 0;
            angular_temp = 0;
            break;
        default:
            break;
    }
}


/*
* 函数功能：将速度请求以set_float的形式发送给set_velocity_srv
*/
void updateSpeed() {   
    nav_msgs::Odometry speed_data;
    float L = 0.6;                          // 两轮之间的距离
    // 根据双轮差动底盘算法计算
    // v(linear_temp)为底盘中心线速度；w(angular_temp)为底盘中心角速度
    // Vl(speeds[0]),Vr(speeds[1])为左右两轮的速度
    // d(float L = 0.06)为轮子离底盘中心的位置
    // v = (Vr+Vl)/2      w = (Vr-Vl)/2d
    speeds[0]  = 10.0*(2.0*linear_temp - L*angular_temp)/2.0;
    speeds[1]  = 10.0*(2.0*linear_temp + L*angular_temp)/2.0;
    for (int i = 0; i < NMOTORS; ++i) {
        // 发送给webots更新机器人速度
        set_velocity_client = n->serviceClient<webots_ros::set_float>(string("/tianbot_mini/") + string(motorNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = speeds[i];
        set_velocity_client.call(set_velocity_srv);
    }
    ROS_INFO("左轮转速：%lf, 右轮转速：%lf", speeds[0], speeds[1]);
    // 发送/vel 数据
    speed_data.header.stamp = ros::Time::now();
    speed_data.twist.twist.linear.x = linear_temp;
    speed_data.twist.twist.angular.z = angular_temp;
    speed_puber.publish(speed_data);
    // 速度值清零
    speeds[0]=0;
    speeds[1]=0;
}


/*
* 函数功能：启用键盘控制
*/
void enableKeyboard()
{
    keyboardEnableClient = n->serviceClient<webots_ros::set_int>("/tianbot_mini/keyboard/enable");
    keyboardEnablesrv.request.value = TIME_STEP;
    if (keyboardEnableClient.call(keyboardEnablesrv) && keyboardEnablesrv.response.success)
    {
        ros::Subscriber keyboardSub;
        keyboardSub = n->subscribe("/tianbot_mini/keyboard/key",1,keyboardDataCB);
        while (keyboardSub.getNumPublishers() == 0) {}
        setlocale(LC_CTYPE,"zh_CN.utf8");   // 控制台设置输出中文，否则就是乱码
        ROS_INFO("键盘控制节点成功启动！");
        ROS_INFO("控制方向：");
        ROS_INFO("  ↑  ");
        ROS_INFO("← ↓ →");
        ROS_INFO("刹车：空格键");
        ROS_INFO("注意：需要在Webots界面使用键盘控制！！！");
        while (ros::ok()) {   
            ros::spinOnce();
            updateSpeed();
            if (!time_step_client.call(time_step_srv) || !time_step_srv.response.success)
            {  
                ROS_ERROR("time_step服务无响应");     
                break;   
            }   
        } 
    }
    else
    {
        ROS_ERROR("未能使能键盘！");
    }
}