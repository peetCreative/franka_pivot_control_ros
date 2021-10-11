//
// Created by peetcreative on 21.02.21.
//
#include "franka_pivot_control/FrankaPivotController.h"
#include "PivotControlMessages.h"
#include "pivot_control_messages_ros/LaparoscopeDOFPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFBoundaries.h"
#include "pivot_control_messages_ros/PivotError.h"
#include "pivot_control_messages_ros/FrankaError.h"
#include "pivot_control_messages_ros/SetFloat.h"
#include "pivot_control_messages_ros/SetJointSpacePose.h"
#include "pivot_control_messages_ros/SetPose.h"
#include "PivotControlMessagesRos.h"

#include "ros/ros.h"
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>

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
    ros::Publisher mFrankaErrorPublisher;
    ros::ServiceServer mMoveCartesianZService;
    ros::ServiceServer mMoveJointSpaceService;
    ros::ServiceServer mPivotToPoseService;
    ros::ServiceServer mStartPivotingService;
    ros::ServiceServer mStopPivotingService;
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
        std::string frankaError;
        if (mFPC->getFrankaError(frankaError))
        {
            pivot_control_messages_ros::FrankaError frankaErrorMsg;
            frankaErrorMsg.header.stamp = now;
            frankaErrorMsg.header.seq = mSeq;
            frankaErrorMsg.header.frame_id = "pivot_point";
            frankaErrorMsg.franka_error = frankaError;
            mFrankaErrorPublisher.publish(frankaErrorMsg);
        }
    }

    void targetDOFPoseCallback(
            pivot_control_messages_ros::LaparoscopeDOFPose poseROS)
    {
        pivot_control_messages::DOFPose pose =
                pivot_control_messages_ros::toDOFPose(poseROS);
        mFPC->setTargetDOFPose(pose);
    };

    bool startPivoting(
        std_srvs::Trigger::Request &, std_srvs::Trigger::Response &resp)
    {
        bool succ = mFPC->startPivoting();
        resp.success = succ;
        return succ;
    }

    bool stopPivoting(
        std_srvs::Trigger::Request &, std_srvs::Trigger::Response &resp)
    {
        bool succ = mFPC->stopPivoting();
        resp.success = succ;
        return succ;
    }

    bool moveCartesianZ(
        pivot_control_messages_ros::SetFloat::Request &request,
        pivot_control_messages_ros::SetFloat::Response &response)
    {
        bool succ = mFPC->moveCartesianZ(request.data);
        response.success = succ;
        return succ;
        // if we are at initial pose STEP1 moveoout
        // move out to -5cm
    }

    bool moveJointSpace(
        pivot_control_messages_ros::SetJointSpacePose::Request &req,
        pivot_control_messages_ros::SetJointSpacePose::Response &response)
    {
        std::array<double,7> bla {
            req.data[0], req.data[1], req.data[2], req.data[3],
            req.data[4], req.data[5], req.data[6]};
        bool succ = mFPC->moveJointSpace(bla);
        response.success = succ;
        return succ;
    }

    bool pivotToPose(
        pivot_control_messages_ros::SetJointSpacePose::Request &req,
        pivot_control_messages_ros::SetJointSpacePose::Response &response)
    {
        //TODO: implement
        return false;
    }

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
        mFrankaErrorPublisher =
            nh->advertise<pivot_control_messages_ros::FrankaError>(
                    "franka_error", 1);
        mPivotToPoseService =
            nh->advertiseService(
                "pivot_to_pose",
                &FrankaPivotControllerRos::pivotToPose, this);
        mMoveCartesianZService =
            nh->advertiseService(
                "move_cartesian_z",
                &FrankaPivotControllerRos::moveCartesianZ, this);
        mMoveJointSpaceService =
            nh->advertiseService(
                "move_joint_space",
                &FrankaPivotControllerRos::moveJointSpace, this);
        mStartPivotingService =
            nh->advertiseService(
                "start_pivoting",
                &FrankaPivotControllerRos::startPivoting, this);
        mStopPivotingService =
            nh->advertiseService(
                "stop_pivoting",
                &FrankaPivotControllerRos::stopPivoting, this);
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
    double distanceEE2PP;
    double distanceEE2TT;
    double dynamicRel = 0.05;
    double cameraTilt;
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
    else
    {
        ROS_ERROR_STREAM_NAMED("franka_pivot_control_ros",
                               "No distance end effector (flange) to pivot point given"
                                       << " by argument distance_ee_to_pp");
        ros::shutdown();
        return 1;
    }
    if(pnh->searchParam("distance_ee_to_tt", paramName))
    {
        pnh->getParam(paramName, distanceEE2TT);
        ROS_INFO_STREAM_NAMED("franka_pivot_control_ros",
                              "set distanceEE2ToolTip to " << distanceEE2TT);
    }
    else
    {
        ROS_ERROR_STREAM_NAMED("franka_pivot_control_ros",
                              "No distance end effector (flange) to tool tip given"
                              << " by argument distance_ee_to_tt");
        ros::shutdown();
        return 1;
    }
    if(pnh->searchParam("dynamic_rel", paramName))
    {
        pnh->getParam(paramName, dynamicRel);
        ROS_INFO_STREAM_NAMED("franka_pivot_control_ros",
                              "set dynamicRel to " << dynamicRel);
    }
    if(pnh->searchParam("camera_tilt", paramName))
    {
        pnh->getParam(paramName, cameraTilt);
    }
    else
    {
        ROS_ERROR_STREAM_NAMED("franka_pivot_control_ros",
                               "No camera tilt given"
                                       << " by argument camera_tilt");
        ros::shutdown();
        return 1;
    }

    ROS_INFO_STREAM_NAMED("franka_pivot_control_ros",
                          "start pivot controller");
    FrankaPivotControllerRos fpcr(
            nh, robotIP, frameId,
            distanceEE2PP, distanceEE2TT,
            dynamicRel, cameraTilt);
    return fpcr.run();
}