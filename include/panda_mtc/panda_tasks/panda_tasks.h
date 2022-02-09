/**
 * \file panda_tasks.h
 * \author Yannick R.
 * \version 1
 * \date 03/12/21
 *
 */

#pragma once

#include <ros/ros.h>

#include <boost/thread/thread.hpp>

// Moveit task constructor related include
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_custom_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <moveit/task_constructor/cost_terms.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Moveit  related include
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/CollisionObject.h>

//#include <moveit/collision_detection_bullet/collision_env_bullet.h>
//#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
//#include <moveit/collision_detection/collision_tools.h>

// TF2 related include
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Geometry messages related include
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

// standard Messages related include
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <shape_msgs/SolidPrimitive.h>
// Standard type related include
#include<string>
#include <unordered_map>

// Action related include
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

// Geometric shapes related include
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>

#include "ontologenius/OntologyManipulator.h"

// Action server for supervisor to call pick, place and move tasks
#include <pr2_motion_tasks_msgs/planAction.h>
#include <pr2_motion_tasks_msgs/executeAction.h>

#include <pr2_motion_tasks_msgs/GetPose.h>

#include <overworld/BoundingBox.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Name of the ontology of the robot
#define ROBOT_ONTOLOGY_NAME "dual_pandas"

// Default planner used when creating a task
#define PLANNER "TRRT"

// Default longest segment fraction used when planning task 
// This parameter will affect collision detection and planning time
// low value = less collision but more planning time
#define DEFAULT_LONGEST_VALID_SEGMENT_FRACTION 0.00001

// name of the topic to access the getPose service 
// handeld by the situation assesment
#define GET_POSE_TOPIC "/overworld/getPose"

#define SUPPORT_SURFACE "table"

// name of the topic to access the getBoundingBox service
// handeld by the situation assesment
#define GET_BOUNDINGBOX_TOPIC "/dual_pandas/getBoundingBox"

// define max number of solution that Moveit
// task constructor will search for when planning 
// More solutions means more planning time 
#define NUMBER_OF_MAX_SOLUTION 10.0

using namespace moveit::task_constructor;

class motionPlanning
{
    public:
        motionPlanning(ros::NodeHandle& nh);
        ~motionPlanning();

        void createPlaceTask(std::unique_ptr<moveit::task_constructor::Task>& placeTask, const std::string planGroup, const std::string object, std::vector<geometry_msgs::PoseStamped> placePoses);

        void createMoveTask(std::unique_ptr<moveit::task_constructor::Task>& moveTask, const std::string planGroup, const geometry_msgs::PoseStamped moveToPose);

        void createMovePredefinedTask(std::unique_ptr<moveit::task_constructor::Task>& moveTask, const std::string planGroup,const std::string pose_id);

        void createPickTaskCustom(std::unique_ptr<moveit::task_constructor::Task>& pickTask, const std::string planGroup,const std::string object,const std::string supportId, std::vector<geometry_msgs::PoseStamped> graspPoses);


        void createPickTask(std::unique_ptr<moveit::task_constructor::Task>& pickTask, const std::string planGroup,const std::string object, const std::string supportId);

        void createPickPlaceTask(std::unique_ptr<moveit::task_constructor::Task>&pickPlaceTask, const std::string planGroup,const std::string object, const std::string supportId, std::vector<geometry_msgs::PoseStamped> placePoses);

        int updateWorld(ros::ServiceClient& saClient);

        void getPoseIntoBase(const geometry_msgs::PoseStamped in_pose, geometry_msgs::PoseStamped& out_pose);

        void planCallback(const pr2_motion_tasks_msgs::planGoalConstPtr& goal, ros::ServiceClient& saClient);

        void planPreemptCallback();

        void executeCallback(const pr2_motion_tasks_msgs::executeGoalConstPtr& goal);

        void taskStatisticCallback(const moveit_task_constructor_msgs::TaskStatisticsConstPtr& taskStat);

    private:
        ros::NodeHandle nh_;

        // Variable to store the last task that was planned
        // To be able to execute it afterward
        std::unique_ptr<Task> lastPlannedTask_;

        // Variable to store the arm used in the task
        std::string taskArmGroup_;

        // Variable to store the planner to be used
        std::string plannerConfig_;

        // Variable to store the object 
        // we interacted with in the task
        std::string taskObjId_;

        // Plan and execute action server 
        // to handle motion planning and execution request
        std::unique_ptr<actionlib::SimpleActionServer<pr2_motion_tasks_msgs::planAction>> planServer_;
        std::unique_ptr<actionlib::SimpleActionServer<pr2_motion_tasks_msgs::executeAction>> executeServer_;

        // Service to get object pose from situation assesment
        ros::ServiceClient getPoseSrv_;

        // Service to get object size from situation assesment
        ros::ServiceClient getBoundingBoxSrv_;

        // Handle to call Ontologenius API
        OntologyManipulator onto_;

        // Transform related variable to 
        // listen to tf and transform poses
        geometry_msgs::TransformStamped mainTransform_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener transformListenner_;

        // List to keep track of mesh/obj association to avoid asking for mesh
        // if we already have it
        std::unordered_map<std::string, std::string> objMeshMap_;

        // Define PlanningSceneInterface object to add and remove collision objects
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

        // Robot model shared by all tasks
        robot_model_loader::RobotModelLoader robot_model_loader_;
        moveit::core::RobotModelPtr kinematic_model_;

        // planner used for approach and retreat
        std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesianPlanner_;

        // planner used for connect and arm movements
        std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> pipelinePlanner_;

        // planner used for gripper open/close movements
        std::shared_ptr<solvers::JointInterpolationPlanner> gripper_planner_;

        // Variable to set the eef used during task
        std::string eef_;

        // Variable to store the base frame of the robot 
        std::string baseFrame_;

        // Variable to store the fixed frame of the system (realsense frame) 
        std::string fixedFrame_;

        // Variable to set the eef
        // used during dual arm task
        std::string first_eef_;
        std::string second_eef_;

        Eigen::Isometry3d panda_hand_transform_;

        // Variable to set the ikFrame used during task
	    std::string ikFrame_;

        // Variable to set the ikFrame
        // used during dual arm task
        std::string first_ikFrame_;
        std::string second_ikFrame_;
};
