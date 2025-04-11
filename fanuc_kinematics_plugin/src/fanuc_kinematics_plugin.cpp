#include <cmath>
#include <srdfdom/model.h>
#include <urdf_model/model.h>
#include <class_loader/class_loader.hpp>
#include <pluginlib/class_list_macros.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <fanuc_kinematics_plugin/fanuc_kinematics_plugin.h>

// Register plugin as a KinematicBase implementation
CLASS_LOADER_REGISTER_CLASS(fanuc_kinematics::FanucKinematicsPlugin, kinematics::KinematicsBase)

namespace fanuc_kinematics {

    FanucKinematicsPlugin::FanucKinematicsPlugin() : active(false) {}

    // Calculate inverse kinematics to find the joint angles based on the end effector pose
    bool FanucKinematicsPlugin::InverseKinematics(const geometry_msgs::Pose &ik_pose,
                                                  std::vector<double> &solution,
                                                  const IKCallbackFn &solution_callback,
                                                  moveit_msgs::MoveItErrorCodes &error_code) const
    {
        // Desired pose
        double x = ik_pose.position.x;
        double y = ik_pose.position.y;
        double z = ik_pose.position.z;

        // Offset angle for joints on fixed base
        double theta = (2 * pi) / 3;

        // Calculate joint angles
        double q1 = FindJointAngle({x, y, z});
        double q2 = FindJointAngle({(x * cos(theta)) + (y * sin(theta)), (y * cos(theta)) - (x * sin(theta)), z});    // Offset +120 deg
        double q3 = FindJointAngle({(x * cos(theta)) - (y * sin(theta)), (y * cos(theta)) + (x * sin(theta)), z});    // Offset -120 deg

        // Solution
        solution = {q1, q2, q3};

        // Check if solution is valid
        if (!solution_callback.empty())
            solution_callback(ik_pose, solution, error_code);
        else
            error_code.val = error_code.SUCCESS;

        // Return result
        if (error_code.val == error_code.SUCCESS)
            return true;
        else {
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }
    }


    // Calculate joint angle for single leg
    bool FanucKinematicsPlugin::FindJointAngle(const std::vector<double> &ik_pose) const
    {
        // Desired pose
        double x0 = ik_pose[0];
        double y0 = ik_pose[1];
        double z0 = ik_pose[2];

        // Subtract EE length from initial y position
        y0 -= le * (sqrt(3) / 6);

        // Set next y position as base length
        double y1 = -lf * (sqrt(3) / 6);

        // z = by + a
        double a = (x0*x0 + y0*y0 + z0*z0 + la*la - lb*lb - y1*y1) / (2 * z0);
        double b = (y1 - y0) / z0;

        // Calculate discriminant
        double d = -(a + b*y1)*(a + b*y1) + (b*b * la*la) + la*la;

        // Solution does not exist (singularity)
        if (d < 0)
            return std::nan(""); 

        // Calculate for YZ plane
        double y = (y1 - (a * b) - sqrt(d)) / (b*b + 1);
        double z = (b * y) + a;

        // Calculate theta
        double q = atan2(-z, y1 - y);

        // Offset by 180 degrees
        if (y > y1)
            q += pi;

        return q;
    }

    // Calculate forward kinematics to find end effector pose from the joint angles
    bool FanucKinematicsPlugin::ForwardKinematics(const std::vector<double> &joint_angles,
                                                  std::vector<geometry_msgs::Pose> &poses) const
    {        
        // Desired angles
        double q1 = joint_angles[0];
        double q2 = joint_angles[1];
        double q3 = joint_angles[2];

        // Constants
        double t = (lf - le) * sqrt(3)/6 ;

        // Calculate position of J1
        double x1 =  0;
        double y1 = -(t + (la * cos(q1)));
        double z1 = -la * sin(q1);
     
        // Calculate position of J2
        double x2 =  (t + (la * cos(q2))) * sin(pi/3);
        double y2 =  (t + (la * cos(q2))) * sin(pi/6);
        double z2 = -la * sin(q2);
     
        // Calculate position of J3
        double x3 = -(t + (la * cos(q3))) * sin(pi/3);
        double y3 =  (t + (la * cos(q3))) * sin(pi/6);
        double z3 = -la * sin(q3);
     
        // Elbow points 
        double w1 = x1*x1 + y1*y1 + z1*z1;
        double w2 = x2*x2 + y2*y2 + z2*z2;
        double w3 = x3*x3 + y3*y3 + z3*z3;

        // Simplify equation
        double i = (y2 - y1)*x3 - (y3 - y1)*x2;
         
        //  x = a1*z + b1
        double a1 =  ((z2 - z1)*(y3 - y1) - (z3 - z1)*(y2 - y1)) / i;
        double b1 = -((w2 - w1)*(y3 - y1) - (w3 - w1)*(y2 - y1)) / (2 * i);
     
        // y = a2*z + b2
        double a2 = -((z2 - z1)*x3 - (z3 - z1)*x2) / i;
        double b2 =  ((w2 - w1)*x3 - (w3 - w1)*x2) / (2 * i);

        // a*z^2 + b*z + c = 0
        double a = a1*a1 + a2*a2 + 1;
        double b = 2 * ((a1 * b1) + a2*(b2 - y1) - z1);
        double c = b1*b1 + (b2 - y1)*(b2 - y1) + z1*z1 - lb*lb;
      
        // Calculate discriminants
        double d = b*b - (4 * a * c);

        // Solution does not exist (singularity)
        if (d < 0)
            return false;
     
        // Position
        double z = -(b + sqrt(d)) / (2 * a);
        double x =  (a1 * z) + b1;
        double y =  (a2 * z) + b2;

        // Set up solution
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        // Send solution
        poses.push_back({pose});

        // Solution is vaiid
        return true;
    }

    // Initialize joint names and limits
    bool FanucKinematicsPlugin::initialize(const std::string &robot_description,
                                           const std::string &group_name,
                                           const std::string &base_frame,
                                           const std::string &tip_frame,
                                           double search_discretization) 
    {
        // Set the parameters for the solver, for use with kinematic chain IK solvers
        setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

        //Load SRDF
        rdf_loader::RDFLoader rdf_loader(robot_description);
        const srdf::ModelSharedPtr &srdf = rdf_loader.getSRDF();
        const urdf::ModelInterfaceSharedPtr &urdf = rdf_loader.getURDF();

        // Throw error if model did not load
        if (!urdf || !srdf)
        {
            ROS_ERROR_NAMED("fanuc_kinematics_plugin","URDF and SRDF not loaded");
            return false;
        }

        model.reset(new robot_model::RobotModel(urdf, srdf));
        robot_model::JointModelGroup* joint_model_group = model->getJointModelGroup(group_name);

        // Throw error if joint model did not load
        if (!joint_model_group)
        {
            ROS_ERROR_NAMED("fanuc_kinematics_plugin","Joint model not loaded");
            return false;
        }

        ros::NodeHandle nh;
        std::string motors, elbows, bases;
        nh.getParam("Motor", motors);
        nh.getParam("ElbowL", elbows);
        nh.getParam("BaseL", bases);

        for (std::size_t i=0; i < joint_model_group->getJointModels().size(); ++i)
        {
            const robot_model::JointModel* jm = joint_model_group->getJointModels()[i];
            if(jm->getType() == moveit::core::JointModel::REVOLUTE)
            {
                ik_chain.joint_names.push_back(joint_model_group->getJointModelNames()[i]);
                const std::vector<moveit_msgs::JointLimits> &jvec = jm->getVariableBoundsMsg();
                ik_chain.limits.insert(ik_chain.limits.end(), jvec.begin(), jvec.end());

                if(jm->getType() == moveit::core::JointModel::REVOLUTE)
                {
                    // Check if it belongs to the set of active joints in the group
                    if (jm->getMimic() == NULL && jm->getVariableCount() > 0) {
                        std::string name = jm->getName();

                        if(name.find(motors) != std::string::npos)
                            motor_joints.push_back(name);
            
                        else if(name.find(elbows) != std::string::npos)
                            elbow_joints.push_back(name);
                        
                        else if(name.find(bases) != std::string::npos)
                            base_joints.push_back(name);
                    }
                    else if (jm->getMimic() && joint_model_group->hasJointModel(jm->getMimic()->getName()))
                        mimic_joints.push_back(jm);
                }
                else
                    ee_joints.push_back(joint_model_group->getJointModelNames()[i]);
            }
        }

        fk_chain.joint_names = ik_chain.joint_names;
        fk_chain.limits = ik_chain.limits;

        if(!joint_model_group->hasLinkModel(getTipFrame()))
        {
            ROS_ERROR_NAMED("fanuc_kinematics_plugin","Could not find tip name in joint group '%s'", group_name.c_str());
            return false;
        }

        ik_chain.link_names.push_back(getTipFrame());
        fk_chain.link_names = joint_model_group->getLinkModelNames();

        joint_min.resize(ik_chain.limits.size());
        joint_max.resize(ik_chain.limits.size());

        // Set joint limits
        for(unsigned int i=0; i < ik_chain.limits.size(); i++)
        {
            joint_min.at(i) = ik_chain.limits[i].min_position;
            joint_max.at(i) = ik_chain.limits[i].max_position;
        }

        // Set the initial joint state 
        start_state.reset(new robot_state::RobotState(model));
        goal_state.reset((new robot_state::RobotState(model));

        active = true;
        ROS_INFO_NAMED("fanuc_kinematics_plugin", "Kinematics initialized");

        return true;
    }

    // Calculate EE pose based on joint angles
    bool FanucKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                              const std::vector<double> &joint_angles,
                                              std::vector<geometry_msgs::Pose> &poses) const 
    {
        return ForwardKinematics(joint_angles, poses);
    }

    // Calculate joint angles based on EE pose
    bool FanucKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
    {
        const IKCallbackFn solution_callback = 0;

        return InverseKinematics(ik_pose, solution, solution_callback, error_code);
    }

    bool FanucKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 std::vector<double> &solution,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
    {
        const IKCallbackFn solution_callback = 0;
        std::vector<double> consistency_limits;

        return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                                solution, solution_callback, error_code, options);
    }

    bool FanucKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 const std::vector<double> &consistency_limits,
                                                 std::vector<double> &solution,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
    {
        const IKCallbackFn solution_callback = 0;

        return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                                solution, solution_callback, error_code, options);
    }

    bool FanucKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 std::vector<double> &solution,
                                                 const IKCallbackFn &solution_callback,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
    {
        std::vector<double> consistency_limits;
        return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                                solution, solution_callback, error_code, options);
    }

    bool FanucKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 const std::vector<double> &consistency_limits,
                                                 std::vector<double> &solution,
                                                 const IKCallbackFn &solution_callback,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
    {
        // TODO: Replace
        return false;
    }

    const std::vector<std::string>& FanucKinematicsPlugin::getJointNames() const
    {
        return ik_chain.joint_names;
    }

    const std::vector<std::string>& FanucKinematicsPlugin::getLinkNames() const
    {
        return ik_chain.link_names;
    }
}