// All functions and parameter definitions taken directly from MoveIt kinematics::KinematicsBase Class Reference

#ifndef FANUC_KINEMATICS_PLUGIN_H
#define FANUC_KINEMATICS_PLUGIN_H

// ROS
#include <ros/ros.h>

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>

// Gazebo ROS
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>


namespace fanuc_kinematics {

  class FanucPlugin : public kinematics::KinematicsBase {

  public:

    // Base frame (link) that the solver is operating in
    const std::vector<std::string>& getBaseFrame() const;

    // Group name that the solver is operating on  
    const std::vector<std::string>& getGroupName() const;

    // Joint names in the order MoveIt uses them internally 
    const std::vector<std::string>& getJointNames() const;

    // Link names in the order MoveIt uses them internally 
    const std::vector<std::string>& getLinkNames() const;

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
    virtual bool getPositionFK (const std::vector<std::string> &link_names,
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
    virtual bool getPositionIK (const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                std::vector<double> &solution,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /* 
       Intiialize the kinematics for use with kinematic chain IK solvers

       Parameters:
          robot_description - robot idenfifier
          group_name - group that the kinematics will run on
          base_frame - frame (link) that the solver is operating in
          tip_frame - tip of the chain
          search_discretization - discreditization of the chain when solver steps through redundancy

       Returns:
          true - valid solution was found
          false - valid solution was not found
    */
    virtual bool initialize (const std::string &robot_description,
                             const std::string &group_name,
                             const std::string &base_frame,
                             const std::string &tip_frame,
                             double search_discretization);

    /* 
       Intiialize the kinematics for use with non kinematic chain IK solvers

       Parameters:
          robot_description - robot idenfifier
          group_name - group that the kinematics will run on
          base_frame - frame (link) that the solver is operating in
          tip_frames - vector of tips in kinematic tree
          search_discretization - discreditization of the chain when solver steps through redundancy

       Returns:
          true - initialization was successful
          false - initialization was not successful
    */
    virtual bool initialize (const std::string &robot_description,
                             const std::string &group_name,
                             const std::string &base_frame,
                             const std::vector<std::string>&tip_frames,
                             double search_discretization);

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
    virtual bool searchPositionIK (const geometry_msgs::Pose &ik_pose,
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
    virtual bool searchPositionIK (const geometry_msgs::Pose &ik_pose,
                                   const std::vector<double> &ik_seed_state,
                                   double timeout,
                                   const std::vector<double> &consistency_limits,
                                   std::vector<double> &solution,
                                   moveit_msgs::MoveItErrorCodes &error_code,
                                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;


    /* 
       Search for joint angles required to reach a desired end effector pose
       ntended for "searching" for a solutions by stepping through the redundancy (or other numerical routines)
      
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
    virtual bool searchPositionIK (const geometry_msgs::Pose &ik_pose,
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
    virtual bool searchPositionIK (const geometry_msgs::Pose &ik_pose,
                                   const std::vector<double> &ik_seed_state,
                                   double timeout,
                                   std::vector<double> &solution,
                                   const IKCallbackFn &solution_callback,
                                   moveit_msgs::MoveItErrorCodes &error_code,
                                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
  

  protected:

    moveit_msgs::KinemaicSolverInfo fk_chain;                   // Store info for FK solver
    moveit_msgs::KinemaicSolverInfo ik_chain;                   // Store info for IK solver

    std::vector<const robot_model::JointModel*> mimic_joints;   // Store mimic joints in an  array 
    std::vector<std::string> motor_joints;                      // Store joints between fixed base and upper links 
    std::vector<std::string> elbow_joints;                      // Store joints between upper links and lower links 
    std::vector<std::string> base_joints;                       // Store joints between lower links and pin 
    std::vector<std::string> ee_joints;                         // Store joints between pins and end effector  
  
  };
}

#endif