#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

#include "bbotdriver.h"

/*
Subscribed Topics:
    cmd_vel (geometry_msgs/Twist)

Published Topics:
    odom (nav_msgs/Odometry)
    battery_voltage (std_msgs/Float32)
    motors_enabled (std_msgs/Bool)

Services:
    enable_motors (std_srvs/Empty)
    disable_motors (std_srvs/Empty)

Parameters:
    ~port (string, default: /dev/ttyACM0)
    ~baud (int, default: 115200)
    ~cmd_vel_timeout (float, default: 0.6 sec)
    ~odom_frame (string, default: odom)
    ~base_link_frame (string, default: base_link)
*/

std::string paramPort;
int paramBaud;
double paramCmdVelTimeout;
std::string paramOdomFrameId;
std::string paramBaseLinkFrameId;

BBotDriver *driverPtr;
ros::Publisher *posePublisherPtr;
ros::Publisher *voltagePublisherPtr;
ros::Publisher *enabledPublisherPtr;
tf::TransformBroadcaster *transformBroadcasterPtr;
ros::Timer *velocityWatchdogTimerPtr;

bool gracefulShutdown = false;
bool serialError = false;

void cbData(const BBotDriver::Data &data)
{
    nav_msgs::Odometry odometryMsg;
    tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(data.th * M_PI / 180.0), tf::Vector3(data.x * 0.001, data.y * 0.001, 0)), odometryMsg.pose.pose);
    odometryMsg.twist.twist.linear.x = data.linear * 0.001;
    odometryMsg.twist.twist.angular.z = data.angular * M_PI / 180.0;
    odometryMsg.header.frame_id = paramOdomFrameId;
    odometryMsg.child_frame_id = paramBaseLinkFrameId;
    odometryMsg.header.stamp = ros::Time::now();
    posePublisherPtr->publish(odometryMsg);

    geometry_msgs::TransformStamped tfMsg;
    tfMsg.header.stamp = odometryMsg.header.stamp;
    tfMsg.header.frame_id = odometryMsg.header.frame_id;
    tfMsg.child_frame_id = odometryMsg.child_frame_id;
    tfMsg.transform.translation.x = odometryMsg.pose.pose.position.x;
    tfMsg.transform.translation.y = odometryMsg.pose.pose.position.y;
    tfMsg.transform.rotation = odometryMsg.pose.pose.orientation;
    transformBroadcasterPtr->sendTransform(tfMsg);

    std_msgs::Bool enabledMsg;
    enabledMsg.data = data.enabled;
    enabledPublisherPtr->publish(enabledMsg);

    std_msgs::Float32 voltageMsg;
    voltageMsg.data = data.voltage * 0.1;
    voltagePublisherPtr->publish(voltageMsg);
}

void cbStop()
{
    if (!gracefulShutdown)
    {
        serialError = true;
        ros::shutdown();
    }
}

void cbVelocity(const geometry_msgs::Twist::ConstPtr &msg)
{
    driverPtr->setVelocity(BBotDriver::Velocity{(short int) (msg->linear.x * 1000), (short int) (msg->angular.z * 180.0 / M_PI)});
    velocityWatchdogTimerPtr->stop();
    velocityWatchdogTimerPtr->start();
}

bool cbEnable(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    driverPtr->setMotors(BBotDriver::Motors{1});
    return true;
}

bool cbDisable(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    driverPtr->setMotors(BBotDriver::Motors{0});
    return true;
}

void cbVelocityWatchdog(const ros::TimerEvent& event)
{
    driverPtr->setVelocity(BBotDriver::Velocity{0, 0});
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bbot_driver");
    ros::NodeHandle node;
    ros::NodeHandle nodePrivate("~");

    nodePrivate.param("port", paramPort, std::string("/dev/ttyACM0"));
    nodePrivate.param("baud", paramBaud, 115200);
    nodePrivate.param("cmd_vel_timeout", paramCmdVelTimeout, 0.5);
    nodePrivate.param("odom_frame_id", paramOdomFrameId, std::string("odom"));
    nodePrivate.param("base_link_frame_id", paramBaseLinkFrameId, std::string("base_link"));

    ROS_INFO_STREAM("port: " << paramPort);
    ROS_INFO_STREAM("baud: " << paramBaud);
    ROS_INFO_STREAM("cmd_vel_timeout: " << paramCmdVelTimeout);
    ROS_INFO_STREAM("odom_frame_id: " << paramOdomFrameId);
    ROS_INFO_STREAM("base_link_frame_id: " << paramBaseLinkFrameId);

    BBotDriver driver(paramPort, paramBaud, cbData, cbStop);

    ros::Subscriber velocitySubscriber = node.subscribe<geometry_msgs::Twist>("cmd_vel", 10, cbVelocity);
    ros::Publisher posePublisher = node.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Publisher voltagePublisher = node.advertise<std_msgs::Float32>("battery_voltage", 10);
    ros::Publisher enabledPublisher = node.advertise<std_msgs::Bool>("motors_enabled", 10);
    tf::TransformBroadcaster transformBroadcaster;
    ros::ServiceServer enableService = node.advertiseService("enable_motors", cbEnable);
    ros::ServiceServer disableService = node.advertiseService("disable_motors", cbDisable);
    ros::Timer velocityWatchdogTimer = node.createTimer(ros::Duration(paramCmdVelTimeout), cbVelocityWatchdog);

    driverPtr = &driver;
    posePublisherPtr = &posePublisher;
    voltagePublisherPtr = &voltagePublisher;
    enabledPublisherPtr = &enabledPublisher;
    transformBroadcasterPtr = &transformBroadcaster;
    velocityWatchdogTimerPtr = &velocityWatchdogTimer;

    driver.start();

    ros::spin();

    if (serialError)
    {
        ROS_ERROR_STREAM("Serial port error");
        return -1;
    }

    gracefulShutdown = true;

    driver.stop();

    return 0;
}

