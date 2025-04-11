// All functions and parameter definitions taken directly from MoveIt kinematics::KinematicsBase Class Reference

#ifndef FANUC_PROJECT_FANUC_KINEMATICS_PLUGIN_H
#define FANUC_PROJECT_FANUC_KINEMATICS_PLUGIN_H

// System
#include <memory>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

// MoveIt
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit/kinematics_base/kinematics_base.h>


namespace fanuc_kinematics {

   //MOVEIT_CLASS_FORWARD(FanucKinematicsPlugin);

   class FanucKinematicsPlugin : public kinematics::KinematicsBase {

   public:

      FanucKinematicsPlugin();

      // Math constants
      double pi = 3.14159265358979323846; 

      // Robot specs
      double la = 0.89;       // Length of upper joints (J1-3 A)
      double lb = 1.38;       // Length of lower joints (J1-3 B)
      double le = 0.10;       // Side length of the EE triangle
      double lf = 0.56;       // Side length of the fixed triangle

      // Custom FK function
      virtual bool ForwardKinematics(const std::vector<double> &joint_angles, std::vector<geometry_msgs::Pose> &poses) const;

      // Custom IK function
      virtual bool InverseKinematics(const geometry_msgs::Pose &ik_pose, 
                                     std::vector<double> &solution,
                                     const IKCallbackFn &solution_callback,
                                     moveit_msgs::MoveItErrorCodes &error_code) const;

      // Custom IK helper function
      virtual bool FindJointAngle(const std::vector<double> &ik_pose) const;

      // Joint names in the order MoveIt uses them internally 
      virtual const std::vector<std::string>& getJointNames() const;

      // Link names in the order MoveIt uses them internally 
      virtual const std::vector<std::string>& getLinkNames() const;

      /* 
         Intiialize the kinematics for use with non kinematic chain IK solvers

         Parameters:
            robot_description - robot idenfifier
            group_name - group that the kinematics will run on
            base_frame - frame (link) that the solver is operating in
            tip_frames - tip of the chain
            search_discretization - discreditization of the chain when solver steps through redundancy

         Returns:
            true - valid solution was found
            false - valid solution was not found
      */
      virtual bool initialize(const std::string &robot_description,
                              const std::string &group_name,
                              const std::string &base_frame,
                              const std::string &tip_frames,
                              double search_discretization);

      /* 
         Compute the end effector pose given the joint angles
         
         Parameters:
            link_names - set of links 
            joint_angles - state of joint angles
            poses - resultant set of poses in base frame 

         Returns:
            true - valid solution was found
            false - valid solution was not found
      */
      virtual bool getPositionFK(const std::vector<std::string> &link_names,
                                 const std::vector<double> &joint_angles,
                                 std::vector<geometry_msgs::Pose> &poses) const;

      /* 
         Compute the joint angles given a desired end effector pose
         
         Parameters:
            ik_pose - desired pose of the link
            ik_seed_state - initial guess solution of IK
            solution - solution vector 
            error_code - encodes the reason for failure or success
            lock_redundant_joints - keep values of redundant joints if setRedundantJoint() was previously called
          
         Returns:
            true - valid solution was found
            false - valid solution was not found
      */
      virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                                 const std::vector<double> &ik_seed_state,
                                 std::vector<double> &solution,
                                 moveit_msgs::MoveItErrorCodes &error_code,
                                 const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

      /* 
         Search for joint angles required to reach a desired end effector pose
         Intended for "searching" for a solutions by stepping through the redundancy (or other numerical routines)

         Parameters:
            ik_pose - desired pose of the link
            ik_seed_state - initial guess solution of IK
            timeout - amount of time (sec) to search
            solution - solution vector 
            error_code - encodes the reason for failure or success
            lock_redundant_joints - keep values of redundant joints if setRedundantJoint() was previously called

         Returns:
            true - initialization was successful
            false - initialization was not successful
      */
      virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    double timeout,
                                    std::vector<double> &solution,
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

      /* 
         Search for joint angles required to reach a desired end effector pose 
         Intended for "searching" for a solutions by stepping through the redundancy (or other numerical routines)
         
         Parameters:
            ik_pose - desired pose of the link
            ik_seed_state - initial guess solution of IK
            timeout - amount of time (sec) to search
            consistency_limits - distance that any solution joint can be from corresponding joint in current seed state
            solution - solution vector 
            error_code - encodes the reason for failure or success
            lock_redundant_joints - keep values of redundant joints if setRedundantJoint() was previously called

         Returns:
            true - valid solution was found
            false - valid solution was not found
      */
      virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    double timeout,
                                    const std::vector<double> &consistency_limits,
                                    std::vector<double> &solution,
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

      /* 
         Search for joint angles required to reach a desired end effector pose
         Intended for "searching" for a solutions by stepping through the redundancy (or other numerical routines)
         
         Parameters:
            ik_pose - desired pose of the link
            ik_seed_state - initial guess solution of IK
            timeout - amount of time (sec) to search
            solution - solution vector 
            solution_callback - callback for IK solution
            error_code - encodes the reason for failure or success
            lock_redundant_joints - keep values of redundant joints if setRedundantJoint() was previously called

         Returns:
            true - valid solution was found
            false - valid solution was not found
      */
      virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    double timeout,
                                    std::vector<double> &solution,
                                    const IKCallbackFn &solution_callback,
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

      /* 
         Search for joint angles required to reach a desired end effector pose
         ntended for "searching" for a solutions by stepping through the redundancy (or other numerical routines)
         
         Parameters:
            ik_pose - desired pose of the link
            ik_seed_state - initial guess solution of IK
            timeout - amount of time (sec) to search
            consistency_limits - distance that any solution joint can be from corresponding joint in current seed state
            solution - solution vector 
            solution_callback - callback for IK solution
            error_code - encodes the reason for failure or success
            lock_redundant_joints - keep values of redundant joints if setRedundantJoint() was previously called

         Returns:
            true - valid solution was found
            false - valid solution was not found
      */
      virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    double timeout,
                                    const std::vector<double> &consistency_limits,
                                    std::vector<double> &solution,
                                    const IKCallbackFn &solution_callback,
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
  

   protected:

      bool active;                                                // Solver is configured and ready
      robot_model::RobotModelPtr model;                           // Robot model

      moveit_msgs::KinematicSolverInfo fk_chain;                  // Store info for FK solver
      moveit_msgs::KinematicSolverInfo ik_chain;                  // Store info for IK solver

      std::vector<std::string> motor_joints;                      // Store joints between fixed base and upper links 
      std::vector<std::string> elbow_joints;                      // Store joints between upper links and lower links 
      std::vector<std::string> base_joints;                       // Store joints between lower links and pin 
      std::vector<std::string> ee_joints;                         // Store joints between pins and end effector  

      std::vector<const robot_model::JointModel*> mimic_joints;   // Store mimic joints in an  array 

      std::vector<double> joint_min, joint_max;
      robot_state::RobotStatePtr start_state, goal_state;
  };
}  

#endif