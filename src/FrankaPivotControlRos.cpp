//
// Created by peetcreative on 21.02.21.
//
#include "FrankaPivotController.h"
#include "PivotControlMessages.h"
#include "pivot_control_messages_ros/LaparoscopeDOFPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFBoundaries.h"
#include "pivot_control_messages_ros/PivotError.h"
#include "PivotControlMessagesRos.h"

#include "ros/ros.h"
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

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
    ros::Publisher mPivotErrorPublisher;
public:
    void statePublisher(const ros::TimerEvent&)
    {
        mSeq++;
        auto now = ros::Time::now();
        pivot_control_messages::DOFPose pose;
        if (mFPC->getCurrentDOFPose(pose))
        {
            pivot_control_messages_ros::LaparoscopeDOFPose poseRos =
                pivot_control_messages_ros::toROSDOFPose(pose, mFrameId, mSeq);
            poseRos.header.stamp = now;
            mCurrentDOFPosePublisher.publish(poseRos);
        }
        pivot_control_messages::DOFBoundaries boundaries;
        if (mFPC->getDOFBoundaries(boundaries))
        {
            pivot_control_messages_ros::LaparoscopeDOFBoundaries boundariesROS =
                pivot_control_messages_ros::toROSDOFBoundaries(boundaries, mFrameId, mSeq);
            boundariesROS.header.stamp = now;
            mDOFBoundariesPublisher.publish(boundariesROS);
        }
        std::array<double, 3> translation;
        std::array<double, 4> rotation;
        if (mFPC->getCurrentTipPose(translation, rotation))
        {
            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = now;
            transformStamped.header.seq = mSeq;
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "camera";
            transformStamped.transform.translation.x = translation.at(0);
            transformStamped.transform.translation.y = translation.at(1);
            transformStamped.transform.translation.z = translation.at(2);
            transformStamped.transform.rotation.w = rotation.at(0);
            transformStamped.transform.rotation.x = rotation.at(1);
            transformStamped.transform.rotation.y = rotation.at(2);
            transformStamped.transform.rotation.z = rotation.at(3);
            br.sendTransform(transformStamped);
        }
        double error;
        if (mFPC->getError(error))
        {
            pivot_control_messages_ros::PivotError errorMsg;
            errorMsg.header.stamp = now;
            errorMsg.header.seq = mSeq;
            errorMsg.header.frame_id = "pivot_point";
            errorMsg.pivot_error = error;
            mPivotErrorPublisher.publish(errorMsg);
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
            return 0;
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
            double distanceEE2PP = 0.2,
            double distanceEE2Tip = 0.2,
            double maxWaypointDist = 0.01,
            double cameraTilt = -0.52359):
            mFrameId(frameId)
    {
        mFPC = std::make_unique<
                franka_pivot_control::FrankaPivotController>(
                        robotIP, distanceEE2PP, distanceEE2Tip,
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
        mPivotErrorPublisher =
            nh->advertise<pivot_control_messages_ros::PivotError>(
                    "pivot_error", 1);
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
    double distanceEE2PP = 0.5;
    double distanceEE2CameraTip = 0.505;
    double dynamicRel = 0.05;
    double cameraTilt = -0.52359;
    if(!(pnh->searchParam("robot_ip", paramName) &&
            pnh->getParam(paramName, robotIP)))
    {
        ROS_ERROR("could not find robot_ip");
        return 0;
    }
    if(pnh->searchParam("frame_id", paramName))
        pnh->getParam(paramName, frameId);
    if(pnh->searchParam("distance_ee_to_pp", paramName))
    {
        pnh->getParam(paramName, distanceEE2PP);
        ROS_INFO_STREAM_NAMED("franka_pivot_control_ros",
                              "set distanceEE2PP to " << distanceEE2PP);
    }
    if(pnh->searchParam("distance_ee_to_camera_tip", paramName))
    {
        pnh->getParam(paramName, distanceEE2CameraTip);
        ROS_INFO_STREAM_NAMED("franka_pivot_control_ros",
                              "set distanceEE2CameraTip to " << distanceEE2CameraTip);
    }
    if(pnh->searchParam("dynamic_rel", paramName))
    {
        pnh->getParam(paramName, dynamicRel);
        ROS_INFO_STREAM_NAMED("franka_pivot_control_ros",
                              "set dynamicRel to " << dynamicRel);
    }
    if(pnh->searchParam("camera_tilt", paramName))
        pnh->getParam(paramName, cameraTilt);

    ROS_INFO_STREAM_NAMED("franka_pivot_control_ros",
                          "start pivot controller");
    FrankaPivotControllerRos fpcr(
            nh, robotIP, frameId,
            distanceEE2PP, distanceEE2CameraTip,
            dynamicRel, cameraTilt);
    return fpcr.run();
}