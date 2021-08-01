/*
引用URL：https://github.com/ROBOTIS-GIT/turtlebot3_deliver/blob/master/turtlebot3_deliver_service/src/call_initial_pose.cpp
*/

#include "ros/ros.h"
#include "cultivator_service/InitRobotPose.h"

ros::ServiceClient client_initial_pose;
cultivator_service::InitRobotPose initRobotPose;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "call_initial_pose");

    ros::NodeHandle n;

    initRobotPose.request.poseWithCovarianceStamped.header.frame_id = "map";
    initRobotPose.request.poseWithCovarianceStamped.header.stamp = ros::Time::now();
    initRobotPose.request.poseWithCovarianceStamped.pose.pose.position.x = -0.954769432545;
    initRobotPose.request.poseWithCovarianceStamped.pose.pose.position.y = 1.48137366772;
    initRobotPose.request.poseWithCovarianceStamped.pose.pose.position.z = 0.0;

    initRobotPose.request.poseWithCovarianceStamped.pose.pose.orientation.x = 0.0;
    initRobotPose.request.poseWithCovarianceStamped.pose.pose.orientation.y = 0.0;
    initRobotPose.request.poseWithCovarianceStamped.pose.pose.orientation.z = -0.694655700189;
    initRobotPose.request.poseWithCovarianceStamped.pose.pose.orientation.w = 0.719342378979;

    initRobotPose.request.poseWithCovarianceStamped.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};

    client_initial_pose = n.serviceClient<cultivator_service::InitRobotPose>("/initial_pose");

    client_initial_pose.call(initRobotPose);

    return 0;
}