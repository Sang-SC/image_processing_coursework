#include <signal.h>
#include "ros/ros.h"
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Int32Stamped.h>

#include "line_follower/webotsInterface.hpp"

using namespace std;

ros::NodeHandle *n;

#define TIME_STEP 32    // 时钟
#define NMOTORS 2       // 电机数量
#define MAX_SPEED 2.0   // 电机最大速度


int main(int argc, char **argv) {
    setlocale(LC_CTYPE,"zh_CN.utf8");   // 控制台设置输出中文，否则就是乱码
    // 新建节点
    ros::init(argc, argv, "main");
    n = new ros::NodeHandle;
    webots_init();
    while(ros::ok())
    {
        ros::spinOnce();
    }

    // // 设置 PID 参数，用以控制电机
    // pid_init(0.5,0.2,0.05);

}