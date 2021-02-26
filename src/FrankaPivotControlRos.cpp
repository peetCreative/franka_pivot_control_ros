//
// Created by peetcreative on 21.02.21.
//
#include "FrankaPivotController.h"
#include "PivotControlMessages.h"
#include "pivot_control_messages_ros/LaparoscopeDOFPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFBoundaries.h"
#include "PivotControlMessagesRos.h"

#include "ros/ros.h"
#include <ros/time.h>
#include <memory>

typedef std::unique_ptr<franka_pivot_control::FrankaPivotController> FPCPtr;

class FrankaPivotControllerRos
{
private:
    FPCPtr mFPC;
    int mSeq = 0;
    std::string mFrameId;
    ros::Timer mPublishTimer;
    ros::Subscriber mTargetDOFPoseSubscriber;
    ros::Publisher mCurrentDOFPosePublisher;
    ros::Publisher mDOFBoundariesPublisher;
public:
    void statePublisher(const ros::TimerEvent&)
    {
        pivot_control_messages::DOFPose pose;
        if (mFPC->getCurrentDOFPose(pose))
        {
            pivot_control_messages_ros::LaparoscopeDOFPose poseRos =
                pivot_control_messages_ros::toROSDOFPose(pose, mFrameId, mSeq++);
            mCurrentDOFPosePublisher.publish(poseRos);
        }
        pivot_control_messages::DOFBoundaries boundaries;
        if (mFPC->getDOFBoundaries(boundaries))
        {
            pivot_control_messages_ros::LaparoscopeDOFBoundaries boundariesROS =
                pivot_control_messages_ros::toROSDOFBoundaries(boundaries, mFrameId, mSeq++);
            mDOFBoundariesPublisher.publish(boundariesROS);

        }
    }

    void targetDOFPoseCallback(
            pivot_control_messages_ros::LaparoscopeDOFPose poseROS)
    {
        pivot_control_messages::DOFPose pose =
                pivot_control_messages_ros::toDOFPose(poseROS);
        mFPC->setTargetDOFPose(pose);
    };

    int run()
    {
        if (!mFPC->isReady())
        {
             ROS_DEBUG_NAMED("FrankaPivotControllerROS", "Stop");
            ros::shutdown();
            return 1;
        }
        while(ros::ok())
        {
            ros::spinOnce();
        }
        return 0;
    }

    FrankaPivotControllerRos(
            ros::NodeHandle *nh,
            std::string robotIP,
            std::string frameId,
            float distanceEE2PP = 0.2,
            float maxWaypointDist = 0.01,
            float cameraTilt = -0.52359):
            mFrameId(frameId)
    {
        mFPC = std::make_unique<
                franka_pivot_control::FrankaPivotController>(
                        robotIP, distanceEE2PP,
                        maxWaypointDist, cameraTilt);
        mTargetDOFPoseSubscriber =
            nh->subscribe<pivot_control_messages_ros::LaparoscopeDOFPose>(
                    "target_dof_pose", 1,
                    &FrankaPivotControllerRos::targetDOFPoseCallback,
                    this);
        mCurrentDOFPosePublisher =
            nh->advertise<pivot_control_messages_ros::LaparoscopeDOFPose>(
                    "current_dof_pose", 1);
        mDOFBoundariesPublisher =
            nh->advertise<pivot_control_messages_ros::LaparoscopeDOFBoundaries>(
                    "dof_boundaries", 1);
        mPublishTimer = nh->createTimer(
                ros::Duration(0.05),
               &FrankaPivotControllerRos::statePublisher, this,
               false);
        mPublishTimer.start();
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "franka_pivot_control_ros");
    ros::NodeHandle *nh = new ros::NodeHandle();
    ros::NodeHandle *pnh = new ros::NodeHandle("~");

    std::string paramName;
    std::string robotIP = "";
    std::string frameId = "laparoscope_pivot";
    float distanceEE2PP = 0.2;
    float maxWaypointDist = 0.01;
    float cameraTilt = -0.52359;
    if(!(pnh->searchParam("robot_ip", paramName) &&
            pnh->getParam(paramName, robotIP)))
    {
        ROS_ERROR("could not find robot_ip");
        return 1;
    }
    if(pnh->searchParam("frame_id", paramName))
        pnh->getParam(paramName, frameId);
    if(pnh->searchParam("distance_ee_to_pp", paramName))
        pnh->getParam(paramName, distanceEE2PP);
    if(pnh->searchParam("max_waypoint_distance", paramName))
        pnh->getParam(paramName, maxWaypointDist);
    if(pnh->searchParam("camera_tilt", paramName))
        pnh->getParam(paramName, cameraTilt);

    FrankaPivotControllerRos fpcr(
            nh, robotIP, frameId,
            distanceEE2PP, maxWaypointDist, cameraTilt);
    return fpcr.run();
}