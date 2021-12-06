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

#include <franka_msgs/FrankaState.h>
#include <franka_control/franka_state_controller.h>
#include "ros/ros.h"
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>
#include <ros/console.h>

#include <memory>

typedef std::unique_ptr<franka_pivot_control::FrankaPivotController> FPCPtr;

franka_msgs::Errors errorsToMessage(const franka::Errors& error) {
    franka_msgs::Errors message;
    message.joint_position_limits_violation =
            static_cast<decltype(message.joint_position_limits_violation)>(
                    error.joint_position_limits_violation);
    message.cartesian_position_limits_violation =
            static_cast<decltype(message.cartesian_position_limits_violation)>(
                    error.cartesian_position_limits_violation);
    message.self_collision_avoidance_violation =
            static_cast<decltype(message.self_collision_avoidance_violation)>(
                    error.self_collision_avoidance_violation);
    message.joint_velocity_violation =
            static_cast<decltype(message.joint_velocity_violation)>(error.joint_velocity_violation);
    message.cartesian_velocity_violation =
            static_cast<decltype(message.cartesian_velocity_violation)>(
                    error.cartesian_velocity_violation);
    message.force_control_safety_violation =
            static_cast<decltype(message.force_control_safety_violation)>(
                    error.force_control_safety_violation);
    message.joint_reflex = static_cast<decltype(message.joint_reflex)>(error.joint_reflex);
    message.cartesian_reflex =
            static_cast<decltype(message.cartesian_reflex)>(error.cartesian_reflex);
    message.max_goal_pose_deviation_violation =
            static_cast<decltype(message.max_goal_pose_deviation_violation)>(
                    error.max_goal_pose_deviation_violation);
    message.max_path_pose_deviation_violation =
            static_cast<decltype(message.max_path_pose_deviation_violation)>(
                    error.max_path_pose_deviation_violation);
    message.cartesian_velocity_profile_safety_violation =
            static_cast<decltype(message.cartesian_velocity_profile_safety_violation)>(
                    error.cartesian_velocity_profile_safety_violation);
    message.joint_position_motion_generator_start_pose_invalid =
            static_cast<decltype(message.joint_position_motion_generator_start_pose_invalid)>(
                    error.joint_position_motion_generator_start_pose_invalid);
    message.joint_motion_generator_position_limits_violation =
            static_cast<decltype(message.joint_motion_generator_position_limits_violation)>(
                    error.joint_motion_generator_position_limits_violation);
    message.joint_motion_generator_velocity_limits_violation =
            static_cast<decltype(message.joint_motion_generator_velocity_limits_violation)>(
                    error.joint_motion_generator_velocity_limits_violation);
    message.joint_motion_generator_velocity_discontinuity =
            static_cast<decltype(message.joint_motion_generator_velocity_discontinuity)>(
                    error.joint_motion_generator_velocity_discontinuity);
    message.joint_motion_generator_acceleration_discontinuity =
            static_cast<decltype(message.joint_motion_generator_acceleration_discontinuity)>(
                    error.joint_motion_generator_acceleration_discontinuity);
    message.cartesian_position_motion_generator_start_pose_invalid =
            static_cast<decltype(message.cartesian_position_motion_generator_start_pose_invalid)>(
                    error.cartesian_position_motion_generator_start_pose_invalid);
    message.cartesian_motion_generator_elbow_limit_violation =
            static_cast<decltype(message.cartesian_motion_generator_elbow_limit_violation)>(
                    error.cartesian_motion_generator_elbow_limit_violation);
    message.cartesian_motion_generator_velocity_limits_violation =
            static_cast<decltype(message.cartesian_motion_generator_velocity_limits_violation)>(
                    error.cartesian_motion_generator_velocity_limits_violation);
    message.cartesian_motion_generator_velocity_discontinuity =
            static_cast<decltype(message.cartesian_motion_generator_velocity_discontinuity)>(
                    error.cartesian_motion_generator_velocity_discontinuity);
    message.cartesian_motion_generator_acceleration_discontinuity =
            static_cast<decltype(message.cartesian_motion_generator_acceleration_discontinuity)>(
                    error.cartesian_motion_generator_acceleration_discontinuity);
    message.cartesian_motion_generator_elbow_sign_inconsistent =
            static_cast<decltype(message.cartesian_motion_generator_elbow_sign_inconsistent)>(
                    error.cartesian_motion_generator_elbow_sign_inconsistent);
    message.cartesian_motion_generator_start_elbow_invalid =
            static_cast<decltype(message.cartesian_motion_generator_start_elbow_invalid)>(
                    error.cartesian_motion_generator_start_elbow_invalid);
    message.cartesian_motion_generator_joint_position_limits_violation =
            static_cast<decltype(message.cartesian_motion_generator_joint_position_limits_violation)>(
                    error.cartesian_motion_generator_joint_position_limits_violation);
    message.cartesian_motion_generator_joint_velocity_limits_violation =
            static_cast<decltype(message.cartesian_motion_generator_joint_velocity_limits_violation)>(
                    error.cartesian_motion_generator_joint_velocity_limits_violation);
    message.cartesian_motion_generator_joint_velocity_discontinuity =
            static_cast<decltype(message.cartesian_motion_generator_joint_velocity_discontinuity)>(
                    error.cartesian_motion_generator_joint_velocity_discontinuity);
    message.cartesian_motion_generator_joint_acceleration_discontinuity =
            static_cast<decltype(message.cartesian_motion_generator_joint_acceleration_discontinuity)>(
                    error.cartesian_motion_generator_joint_acceleration_discontinuity);
    message.cartesian_position_motion_generator_invalid_frame =
            static_cast<decltype(message.cartesian_position_motion_generator_invalid_frame)>(
                    error.cartesian_position_motion_generator_invalid_frame);
    message.force_controller_desired_force_tolerance_violation =
            static_cast<decltype(message.force_controller_desired_force_tolerance_violation)>(
                    error.force_controller_desired_force_tolerance_violation);
    message.controller_torque_discontinuity =
            static_cast<decltype(message.controller_torque_discontinuity)>(
                    error.controller_torque_discontinuity);
    message.start_elbow_sign_inconsistent =
            static_cast<decltype(message.start_elbow_sign_inconsistent)>(
                    error.start_elbow_sign_inconsistent);
    message.communication_constraints_violation =
            static_cast<decltype(message.communication_constraints_violation)>(
                    error.communication_constraints_violation);
    message.power_limit_violation =
            static_cast<decltype(message.power_limit_violation)>(error.power_limit_violation);
    message.joint_p2p_insufficient_torque_for_planning =
            static_cast<decltype(message.joint_p2p_insufficient_torque_for_planning)>(
                    error.joint_p2p_insufficient_torque_for_planning);
    message.tau_j_range_violation =
            static_cast<decltype(message.tau_j_range_violation)>(error.tau_j_range_violation);
    message.instability_detected =
            static_cast<decltype(message.instability_detected)>(error.instability_detected);
    return message;
}

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
    ros::Publisher mFrankaStatesPublisher;
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
        franka::RobotState robotState;
        if(mFPC->getRobotState(robotState))
        {
            franka_msgs::FrankaState frankaState;
            static_assert(
                    sizeof(robotState.cartesian_collision) == sizeof(robotState.cartesian_contact),
                    "Robot state Cartesian members do not have same size");
            static_assert(sizeof(robotState.cartesian_collision) == sizeof(robotState.K_F_ext_hat_K),
                          "Robot state Cartesian members do not have same size");
            static_assert(sizeof(robotState.cartesian_collision) == sizeof(robotState.O_F_ext_hat_K),
                          "Robot state Cartesian members do not have same size");
            static_assert(sizeof(robotState.cartesian_collision) == sizeof(robotState.O_dP_EE_d),
                          "Robot state Cartesian members do not have same size");
            static_assert(sizeof(robotState.cartesian_collision) == sizeof(robotState.O_dP_EE_c),
                          "Robot state Cartesian members do not have same size");
            static_assert(sizeof(robotState.cartesian_collision) == sizeof(robotState.O_ddP_EE_c),
                          "Robot state Cartesian members do not have same size");
            for (size_t i = 0; i < robotState.cartesian_collision.size(); i++) {
                frankaState.cartesian_collision[i] = robotState.cartesian_collision[i];
                frankaState.cartesian_contact[i] = robotState.cartesian_contact[i];
                frankaState.K_F_ext_hat_K[i] = robotState.K_F_ext_hat_K[i];
                frankaState.O_F_ext_hat_K[i] = robotState.O_F_ext_hat_K[i];
                frankaState.O_dP_EE_d[i] = robotState.O_dP_EE_d[i];
                frankaState.O_dP_EE_c[i] = robotState.O_dP_EE_c[i];
                frankaState.O_ddP_EE_c[i] = robotState.O_ddP_EE_c[i];
            }

            static_assert(sizeof(robotState.q) == sizeof(robotState.q_d),
                          "Robot state joint members do not have same size");
            static_assert(sizeof(robotState.q) == sizeof(robotState.dq),
                          "Robot state joint members do not have same size");
            static_assert(sizeof(robotState.q) == sizeof(robotState.dq_d),
                          "Robot state joint members do not have same size");
            static_assert(sizeof(robotState.q) == sizeof(robotState.ddq_d),
                          "Robot state joint members do not have same size");
            static_assert(sizeof(robotState.q) == sizeof(robotState.tau_J),
                          "Robot state joint members do not have same size");
            static_assert(sizeof(robotState.q) == sizeof(robotState.dtau_J),
                          "Robot state joint members do not have same size");
            static_assert(sizeof(robotState.q) == sizeof(robotState.tau_J_d),
                          "Robot state joint members do not have same size");
            static_assert(sizeof(robotState.q) == sizeof(robotState.theta),
                          "Robot state joint members do not have same size");
            static_assert(sizeof(robotState.q) == sizeof(robotState.dtheta),
                          "Robot state joint members do not have same size");
            static_assert(sizeof(robotState.q) == sizeof(robotState.joint_collision),
                          "Robot state joint members do not have same size");
            static_assert(sizeof(robotState.q) == sizeof(robotState.joint_contact),
                          "Robot state joint members do not have same size");
            static_assert(sizeof(robotState.q) == sizeof(robotState.tau_ext_hat_filtered),
                          "Robot state joint members do not have same size");
            for (size_t i = 0; i < robotState.q.size(); i++) {
                frankaState.q[i] = robotState.q[i];
                frankaState.q_d[i] = robotState.q_d[i];
                frankaState.dq[i] = robotState.dq[i];
                frankaState.dq_d[i] = robotState.dq_d[i];
                frankaState.ddq_d[i] = robotState.ddq_d[i];
                frankaState.tau_J[i] = robotState.tau_J[i];
                frankaState.dtau_J[i] = robotState.dtau_J[i];
                frankaState.tau_J_d[i] = robotState.tau_J_d[i];
                frankaState.theta[i] = robotState.theta[i];
                frankaState.dtheta[i] = robotState.dtheta[i];
                frankaState.joint_collision[i] = robotState.joint_collision[i];
                frankaState.joint_contact[i] = robotState.joint_contact[i];
                frankaState.tau_ext_hat_filtered[i] = robotState.tau_ext_hat_filtered[i];
            }

            static_assert(sizeof(robotState.elbow) == sizeof(robotState.elbow_d),
                          "Robot state elbow configuration members do not have same size");
            static_assert(sizeof(robotState.elbow) == sizeof(robotState.elbow_c),
                          "Robot state elbow configuration members do not have same size");
            static_assert(sizeof(robotState.elbow) == sizeof(robotState.delbow_c),
                          "Robot state elbow configuration members do not have same size");
            static_assert(sizeof(robotState.elbow) == sizeof(robotState.ddelbow_c),
                          "Robot state elbow configuration members do not have same size");

            for (size_t i = 0; i < robotState.elbow.size(); i++) {
                frankaState.elbow[i] = robotState.elbow[i];
                frankaState.elbow_d[i] = robotState.elbow_d[i];
                frankaState.elbow_c[i] = robotState.elbow_c[i];
                frankaState.delbow_c[i] = robotState.delbow_c[i];
                frankaState.ddelbow_c[i] = robotState.ddelbow_c[i];
            }

            static_assert(sizeof(robotState.O_T_EE) == sizeof(robotState.F_T_EE),
                          "Robot state transforms do not have same size");
            static_assert(sizeof(robotState.O_T_EE) == sizeof(robotState.F_T_NE),
                          "Robot state transforms do not have same size");
            static_assert(sizeof(robotState.O_T_EE) == sizeof(robotState.NE_T_EE),
                          "Robot state transforms do not have same size");
            static_assert(sizeof(robotState.O_T_EE) == sizeof(robotState.EE_T_K),
                          "Robot state transforms do not have same size");
            static_assert(sizeof(robotState.O_T_EE) == sizeof(robotState.O_T_EE_d),
                          "Robot state transforms do not have same size");
            static_assert(sizeof(robotState.O_T_EE) == sizeof(robotState.O_T_EE_c),
                          "Robot state transforms do not have same size");
            for (size_t i = 0; i < robotState.O_T_EE.size(); i++) {
                frankaState.O_T_EE[i] = robotState.O_T_EE[i];
                frankaState.F_T_EE[i] = robotState.F_T_EE[i];
                frankaState.F_T_NE[i] = robotState.F_T_NE[i];
                frankaState.NE_T_EE[i] = robotState.NE_T_EE[i];
                frankaState.EE_T_K[i] = robotState.EE_T_K[i];
                frankaState.O_T_EE_d[i] = robotState.O_T_EE_d[i];
                frankaState.O_T_EE_c[i] = robotState.O_T_EE_c[i];
            }
            frankaState.m_ee = robotState.m_ee;
            frankaState.m_load = robotState.m_load;
            frankaState.m_total = robotState.m_total;

            for (size_t i = 0; i < robotState.I_load.size(); i++) {
                frankaState.I_ee[i] = robotState.I_ee[i];
                frankaState.I_load[i] = robotState.I_load[i];
                frankaState.I_total[i] = robotState.I_total[i];
            }

            for (size_t i = 0; i < robotState.F_x_Cload.size(); i++) {
                frankaState.F_x_Cee[i] = robotState.F_x_Cee[i];
                frankaState.F_x_Cload[i] = robotState.F_x_Cload[i];
                frankaState.F_x_Ctotal[i] = robotState.F_x_Ctotal[i];
            }

            frankaState.time = robotState.time.toSec();
            frankaState.control_command_success_rate =
                    robotState.control_command_success_rate;
            frankaState.current_errors = errorsToMessage(robotState.current_errors);
            frankaState.last_motion_errors =
                    errorsToMessage(robotState.last_motion_errors);

            switch (robotState.robot_mode) {
                case franka::RobotMode::kOther:
                    frankaState.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_OTHER;
                    break;

                case franka::RobotMode::kIdle:
                    frankaState.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_IDLE;
                    break;

                case franka::RobotMode::kMove:
                    frankaState.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_MOVE;
                    break;

                case franka::RobotMode::kGuiding:
                    frankaState.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_GUIDING;
                    break;

                case franka::RobotMode::kReflex:
                    frankaState.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_REFLEX;
                    break;

                case franka::RobotMode::kUserStopped:
                    frankaState.robot_mode =
                            franka_msgs::FrankaState::ROBOT_MODE_USER_STOPPED;
                    break;

                case franka::RobotMode::kAutomaticErrorRecovery:
                    frankaState.robot_mode =
                            franka_msgs::FrankaState::ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY;
                    break;
            }

            frankaState.header.seq = mSeq;
            frankaState.header.stamp = now;
            mFrankaStatesPublisher.publish(frankaState);
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
        ROS_INFO_STREAM_NAMED("franka_pivot_control_ros",
                              "run");
        mFPC->startPivoting();
        if (!mFPC->isReady())
        {
            ROS_INFO_NAMED("FrankaPivotControllerROS", "Stop Franka Pivot Controller not ready");
            ros::shutdown();
            mFPC->stopPivoting();
            return 0;
        }
        while(ros::ok())
        {
            ros::spinOnce();
        }
        ROS_INFO_STREAM_NAMED("franka_pivot_control_ros", "Finishing");
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
        mFrankaStatesPublisher =
            nh->advertise<franka_msgs::FrankaState>(
                    "franka_states", 1);
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