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
ros::Publisher vel_puber;                     // 话题发布

double speeds[NMOTORS]={0.0,0.0};             // 两个电机速度值，范围为 0～100
float linear_temp=0, angular_temp=0;          // 暂存的线速度和角速度
static const char *motorNames[NMOTORS] ={"left_motor", "right_motor"}; // 匹配电机名

ros::ServiceClient set_camera_client;         // 摄像头service客户端
webots_ros::set_int camera_srv;               // 摄像头service数据
ros::Subscriber sub_camera_img;               // 订阅摄像头图像话题


/*
* 函数功能：webots摄像头数据回调函数
*/
void CameraCallback(const sensor_msgs::Image::ConstPtr &value){
    static int count;
    ROS_INFO("获取到第%d张图像", count);
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
void webots_init()
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
            ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);   
        else     
            ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]);
        // velocity初始速度设置为0   
        set_velocity_client = n->serviceClient<webots_ros::set_float>(string(ROBOT_NAME) + string(motorNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = 0.0;   
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)     
            ROS_INFO("Velocity set to 0.0 for motor %s.", motorNames[i]);   
        else     
            ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);
    }   
    vel_puber = n->advertise<nav_msgs::Odometry>("/vel",10);        // /vel 话题，用于配置odom

    // 初始化摄像头
    set_camera_client = n->serviceClient<webots_ros::set_int>(string(ROBOT_NAME)+string("camera/enable"));
    camera_srv.request.value = TIME_STEP;
    if (set_camera_client.call(camera_srv) && camera_srv.response.success) {
        sub_camera_img = n->subscribe(string(ROBOT_NAME)+string("camera/image"), 1, CameraCallback);
        ROS_INFO("Camera enabled.");
    } else {
        if (!camera_srv.response.success)
        ROS_ERROR("Failed to enable Camera.");
    }
}