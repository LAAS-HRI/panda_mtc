/**
 * \file panda_tasks.cpp
 * \brief Handling of panda tasks creation with helps of Moveit Task Constructor
 * \author Yannick R.
 * \version 0.1
 * \date 03/12/21
 *
 */

#include <panda_mtc/panda_tasks/panda_tasks.h>

// Class constructor
motionPlanning::motionPlanning(ros::NodeHandle& nh)
  : nh_(nh),
  	onto_(ROBOT_ONTOLOGY_NAME),
	robot_model_loader_("robot_description"),
	transformListenner_(tfBuffer_)
{
	ROS_INFO("[Entering class constructor]");

	// Close ontology
	onto_.close();

	// Set to verbose when debugging
	//onto_.verbose(true);

	if (!nh_.getParam("/panda_tasks_node/planner_config", plannerConfig_))
	{
		if ((plannerConfig_ != "RRT") && (plannerConfig_ != "RRTConnect") && (plannerConfig_ != "TRRT" ))
		{
			ROS_INFO("[motionPlanning constructor][planner_config parameter does not exist, setting planner to default one]");
			plannerConfig_ = PLANNER;
		}
	}

	fixedFrame_ = "realsense_link";

	// Load the common kinematic model of the robot that will be used when creating a task with MTC
	kinematic_model_ = robot_model_loader_.getModel();

	// Create the common cartesian planner that will be used for translation movement when creating task with MTC
	cartesianPlanner_ = std::make_shared<solvers::CartesianPath>();
	cartesianPlanner_->setProperty("jump_threshold", 0.00);

	// Create the common pipeline planner (by default RRTConnect) that will be used when creating a task with MTC
	pipelinePlanner_ = std::make_shared<solvers::PipelinePlanner>();
	pipelinePlanner_->setProperty("goal_joint_tolerance", 1e-5);
	pipelinePlanner_->setPlannerId(plannerConfig_);

	// Define the distance between two waypoint in trajectory.
	// Large value might lead to a trajectory going through collision objects
	// Small value will increase computing time
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",DEFAULT_LONGEST_VALID_SEGMENT_FRACTION);

	// Create the common planner for gripper open/close movement when creating task with MTC
	gripper_planner_ = std::make_shared<solvers::JointInterpolationPlanner>();

	// Connect to service published by Overworld to get the poses of the objects seen by the Kinect sensor
	getPoseSrv_ = nh_.serviceClient<pr2_motion_tasks_msgs::GetPose>(GET_POSE_TOPIC);
	ros::service::waitForService(GET_POSE_TOPIC, -1);

	ROS_INFO("[motionPlanning constructor][Node connected to getPose service]");

	getBoundingBoxSrv_ = nh_.serviceClient<overworld::BoundingBox>(GET_BOUNDINGBOX_TOPIC);
	ros::service::waitForService(GET_BOUNDINGBOX_TOPIC, -1);

	ROS_INFO("[motionPlanning constructor][Node connected to getBoundingBox service]");

	// Create the action server to handle motion task planning request.
	// When a planning request is received, call the planCallback function
	planServer_ = std::make_unique<actionlib::SimpleActionServer<pr2_motion_tasks_msgs::planAction>>(nh_, "plan", boost::bind(&motionPlanning::planCallback,this, _1,getPoseSrv_), false);
	planServer_->registerPreemptCallback(boost::bind(&motionPlanning::planPreemptCallback, this));

	// Create the action server to handle motion task execution request.
	// When an execution request is received, call the executeCallback function
	executeServer_ =  std::make_unique<actionlib::SimpleActionServer<pr2_motion_tasks_msgs::executeAction>>(nh_, "execute", boost::bind(&motionPlanning::executeCallback,this, _1), false);


	// Start the plan and execute server created e
	planServer_->start();
	executeServer_->start();

	// Custom transform to get the ik frame of the gripper according to panda_link8	
	Eigen::Quaternion<double> quaternion = Eigen::AngleAxisd(1.571, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0.785,  Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(1.571, Eigen::Vector3d::UnitZ());
	panda_hand_transform_ = Eigen::Translation3d(0, 0, 0.10) * quaternion;

	ROS_INFO("[motionPlanning constructor][Plan and execute action servers started successfully]");
}

// Class destructor
motionPlanning::~motionPlanning()
{
}

 /**
 * \fn void motionPlanning::createPlaceTask(std::unique_ptr<moveit::task_constructor::Task>& placeTask, const std::string planGroup, const std::string object, std::vector<geometry_msgs::PoseStamped> placePoses)
 * \brief Function to create a place task with specific poses and object
 *
 * \param placeTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param object Object to be placed (it needs to be already attached to the eef of the associated planGroup)
 * \param placePoses Poses where to place the object
 */
void motionPlanning::createPlaceTask(std::unique_ptr<moveit::task_constructor::Task>& placeTask, const std::string planGroup, const std::string object, std::vector<geometry_msgs::PoseStamped> placePoses)
{
	placeTask->setRobotModel(kinematic_model_);

	// Property and variable definitions
	std::string ungrasp;

	placeTask->setProperty("object",object);

	placeTask->setProperty("group",planGroup);
	placeTask->setProperty("eef","hand");
	eef_ = "hand";
	ikFrame_ = "panda_link8";
	ungrasp = "open";


	// Increase precision for place to avoid collision
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.0001);

	placeTask->setProperty("ik_frame",ikFrame_);

	pipelinePlanner_->setPlannerId(plannerConfig_);

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");

	// Save state for future stages
	current_state = initial.get();

	// Copy properties defined for placeTask to initial stages (then it will be possible to get them into other stages)
	placeTask->properties().exposeTo(initial->properties(), { "eef", "group","ik_frame" });

	// Add the initial stage to the task
	placeTask->add(std::move(initial));

	{
		// connect to place
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect to place", planners);
		//connect->properties().configureInitFrom(Stage::PARENT);
	  	connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		placeTask->add(std::move(connect));
	}

	{
		auto place = std::make_unique<SerialContainer>("place object");

		placeTask->properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
		place->properties().configureInitFrom(Stage::PARENT, {"eef", "group", "ik_frame"});

		{
			auto stage = std::make_unique<stages::GenerateCustomPose>("place the object");
			stage->setCustomPoses(placePoses);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setMonitoredStage(current_state);

			auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage) );
			wrapper->setMaxIKSolutions(32);
			wrapper->setIKFrame(panda_hand_transform_, ikFrame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group", "ik_frame" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		{
			auto stage = std::make_unique<stages::MoveTo>("release object", gripper_planner_);
			stage->setGroup(eef_);
			stage->setGoal(ungrasp);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
			stage->detachObject(object, ikFrame_);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat from object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.02, 0.20);
			stage->setIKFrame(ikFrame_);

			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}
		placeTask->add(std::move(place));
	}
}

 /**
 * \fn void motionPlanning::createMoveTask(Task &moveTask, const std::string planGroup,const geometry_msgs::PoseStamped moveToPose)
 * \brief Function to create a move task with specific pose and planGroup (arm)
 *
 * \param moveTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param moveToPose Pose where to place the frame (r_gripper_tool_frame or l_gripper_tool_frame depending on planGroup)
 */
void motionPlanning::createMoveTask(std::unique_ptr<moveit::task_constructor::Task>&moveTask, const std::string planGroup,const geometry_msgs::PoseStamped moveToPose)
{
	moveTask->setRobotModel(kinematic_model_);

	eef_ = "hand";
	ikFrame_ = "panda_link8";
	
	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");

    // Save state for future stages
	current_state = initial.get();
	
	moveTask->add(std::move(initial));

	{
		// connect
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		moveTask->add(std::move(connect));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("go to pose");
		stage->setProperty("group",planGroup);
		stage->setProperty("eef",eef_);
		stage->setPose(moveToPose);
		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage) );
		wrapper->setMaxIKSolutions(10);
					wrapper->setIKFrame(panda_hand_transform_, ikFrame_);
	
		wrapper->setProperty("group",planGroup);
		wrapper->setProperty("eef",eef_);
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		moveTask->add(std::move(wrapper));
	}
}

 /**
 * \fn void motionPlanning::createMovePredefinedTask(Task &moveTask, const std::string planGroup,const std::string pose_id)
 * \brief Function to create a move task with specific pre-defined pose and planGroup (arm)
 *
 * \param moveTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param pose_id Id of the pose
 */
void motionPlanning::createMovePredefinedTask(std::unique_ptr<moveit::task_constructor::Task>&moveTask, const std::string planGroup,const std::string pose_id)
{
	moveTask->setRobotModel(kinematic_model_);

	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.0001);
	pipelinePlanner_->setPlannerId(plannerConfig_);

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");

	// Save state for future stages
	current_state = initial.get();
	
	moveTask->add(std::move(initial));

	{
		auto stage = std::make_unique<stages::MoveTo>("Move to pre-defined pose", pipelinePlanner_);
		stage->setGroup(planGroup);
		stage->setGoal(pose_id);
		moveTask->add(std::move(stage));
	}

}

 /**
 * \fn void motionPlanning::createPickTaskCustom(std::unique_ptr<moveit::task_constructor::Task>&pickTask, const std::string planGroup,const std::string object,const std::string supportId, std::vector<geometry_msgs::PoseStamped> graspPoses)
 * \brief Function to create a pick task with specific grasp pose and planGroup (arm)
 *
 * \param pickTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param object Object to pick
 * \param supportId name of the support where the object is laying on
 * \param graspPoses Grasp poses to be used when picking the object
 */
void motionPlanning::createPickTaskCustom(std::unique_ptr<moveit::task_constructor::Task>&pickTask, const std::string planGroup,const std::string object,const std::string supportId, std::vector<geometry_msgs::PoseStamped> graspPoses)
{
	pickTask->setRobotModel(kinematic_model_);

	// Property and variable definitions
	std::string pregrasp;
	std::string postgrasp;

	pickTask->setProperty("object",object);

	pickTask->setProperty("group",planGroup);
	pickTask->setProperty("eef","hand");
	eef_ = "hand";
	ikFrame_ = "panda_link8";
	pregrasp = "open";
	postgrasp = "close";
	
	// Increase precision to avoid collision
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.00001);
	pipelinePlanner_->setPlannerId(plannerConfig_);

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	
	// Save state for future stages
	current_state = initial.get();

	pickTask->add(std::move(initial));


	// ---------------------- open Hand ---------------------- //
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", gripper_planner_);
		stage->setGroup(eef_);
		stage->setGoal(pregrasp);
		stage->setTimeout(5.0);
		current_state = stage.get();
		pickTask->add(std::move(stage));
	}

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
	  	connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		connect->properties().configureInitFrom(Stage::PARENT);
		pickTask->add(std::move(connect));
	}

	{
		auto grasp = std::make_unique<SerialContainer>("pick object");

		pickTask->properties().exposeTo(grasp->properties(), { "eef", "group"});
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "group"});

		grasp->setProperty("eef",eef_);

		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.20);
			stage->setIKFrame(ikFrame_);

			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "panda_link0";
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::GenerateCustomPose>("Generate Custom Poses");
			stage->setCustomPoses(graspPoses);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setMonitoredStage(current_state);

			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage) );
			wrapper->properties().configureInitFrom(Stage::PARENT, { "group" });
			wrapper->setMaxIKSolutions(10);
			wrapper->setIKFrame(panda_hand_transform_,ikFrame_);
	
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
			grasp->insert(std::move(wrapper));
		}

		// ---------------------- Allow Collision (hand object) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(object, pickTask->getRobotModel()->getJointModelGroup(eef_)->getLinkModelNamesWithCollisionGeometry(),true);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Close Hand ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", gripper_planner_);
			stage->setGroup(eef_);
			stage->setGoal(postgrasp);
			stage->setTimeout(5.0);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Attach Object ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object, ikFrame_);
			current_state = stage.get();
			grasp->insert(std::move(stage));
		}

		// ---------------------- Allow Collision (object support) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object }, supportId, true);
			grasp->insert(std::move(stage));
		}
		

		// ---------------------- Lift object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.05);
			stage->setIKFrame(ikFrame_);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "panda_link0";
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		// ---------------------- Forbid Collision (object support) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
			stage->allowCollisions({ object }, supportId, false);
			grasp->insert(std::move(stage));
		}
		pickTask->add(std::move(grasp));
	}

}

 /**
 * \fn void motionPlanning::createPickTask(std::unique_ptr<moveit::task_constructor::Task>&pickTask, const std::string planGroup,const std::string object, const std::string supportId)
 * 
 * \brief Function to create a pick task with grasp pose generator and planGroup (arm)
 *
 * \param pickTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param object Object to pick
 * \param supportId name of the support where the object is laying on
 */
void motionPlanning::createPickTask(std::unique_ptr<moveit::task_constructor::Task>&pickTask, const std::string planGroup,const std::string object, const std::string supportId)
{
	pickTask->setRobotModel(kinematic_model_);

	// Property and variable definitions
	std::string pregrasp;
	std::string grasp;

	pickTask->setProperty("object",object);

	pickTask->setProperty("group",planGroup);
	pickTask->setProperty("eef","hand");
	eef_ = "hand";
	ikFrame_ = "panda_link8";
	pregrasp = "open";
	grasp = "close";

	// Increase precision to avoid collision
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.00001);
	pipelinePlanner_->setPlannerId(plannerConfig_);

	overworld::BoundingBox boundingBoxSrv;
	boundingBoxSrv.request.object_id = object;
	getBoundingBoxSrv_.call(boundingBoxSrv);

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");

	// Save state for future stages
	current_state = initial.get();
	
	pickTask->add(std::move(initial));

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{eef_,gripper_planner_ }, {planGroup,pipelinePlanner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		pickTask->add(std::move(connect));
	}

	{
		auto pick = std::make_unique<SerialContainer>("pick object");
		pickTask->properties().exposeTo(pick->properties(), { "eef", "group"});
		pick->properties().configureInitFrom(Stage::PARENT, { "eef", "group"});

		// ---------------------- approach the object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesianPlanner_);
			stage->properties().set("link", ikFrame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.15);

			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			pick->insert(std::move(stage));
		}

		// ---------------------- generate grasp poses ---------------------- //
		{
			// Sample grasp pose
			auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setPreGraspPose(pregrasp);

			stage->setObjectHeight(boundingBoxSrv.response.z);
			stage->setObject(object);
			stage->setAngleDelta(M_PI / 2);
			stage->setMonitoredStage(current_state);  // Hook into current state

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(32);
			wrapper->setMinSolutionDistance(1.0);

			wrapper->setIKFrame(panda_hand_transform_,ikFrame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			pick->insert(std::move(wrapper));
		}

		// ---------------------- Allow Collision (hand object) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(
			    object, pickTask->getRobotModel()->getJointModelGroup(eef_)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			pick->insert(std::move(stage));
		}

		// ---------------------- close hand ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", gripper_planner_);
			stage->setGroup(eef_);
			stage->setGoal(grasp);
			pick->insert(std::move(stage));
		}

		// ---------------------- attach object ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object, ikFrame_);
			current_state = stage.get();
			pick->insert(std::move(stage));
		}

		// ---------------------- Allow Collision (object support) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object }, supportId, true);
			pick->insert(std::move(stage));
		}

		// ---------------------- lift object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.05);
			stage->setIKFrame(ikFrame_);

			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "panda_link0";
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			pick->insert(std::move(stage));
		}

		// ---------------------- forbid Collision (object surface) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
			stage->allowCollisions({ object }, supportId, false);
			pick->insert(std::move(stage));
		}


		// Add grasp container to task
		pickTask->add(std::move(pick));
	}
}

/**
 * \fn void motionPlanning::createPickPlaceTask(std::unique_ptr<moveit::task_constructor::Task>&pickPlaceTask, const std::string planGroup,const std::string object, const std::string supportId, std::vector<geometry_msgs::PoseStamped> placePoses)
 * \brief Function to create a pick and place task with grasp pose generator and planGroup (arm)
 *
 * \param pickPlaceTask Task to fill
 * \param planGroup Moveit planning group (ie. arm doing the place)
 * \param object Object to pick
 * \param supportId support surface on which the object lies on
 * \param placePoses Poses to place the object
 */
void motionPlanning::createPickPlaceTask(std::unique_ptr<moveit::task_constructor::Task>&pickPlaceTask, const std::string planGroup,const std::string object, const std::string supportId, std::vector<geometry_msgs::PoseStamped> placePoses)
{
	pickPlaceTask->setRobotModel(kinematic_model_);

	// Property and variable definitions
	std::string pregrasp;
	std::string grasp;
	std::string ungrasp; 

	pickPlaceTask->setProperty("object",object);
	
	pickPlaceTask->setProperty("group",planGroup);
	pickPlaceTask->setProperty("eef","hand");
	eef_ = "hand";
	ikFrame_ = "panda_link8";
	pregrasp = "open";
	grasp = "close";
	ungrasp = "open";

	pickPlaceTask->setProperty("ik_frame",ikFrame_);

	// Increase precision to avoid collision
	pipelinePlanner_->setProperty("longest_valid_segment_fraction",0.0001);
	pipelinePlanner_->setPlannerId(plannerConfig_);

	// Call the overworld service to 
	// get the bounding box of the object to pick
	// in order to get object height for pose generation
	overworld::BoundingBox boundingBoxSrv;
	boundingBoxSrv.request.object_id = object;
	getBoundingBoxSrv_.call(boundingBoxSrv);

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");

	// Save state for future stages
	current_state = initial.get();

	pickPlaceTask->properties().exposeTo(initial->properties(), { "eef", "group","ik_frame" });

	pickPlaceTask->add(std::move(initial));

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		pickPlaceTask->add(std::move(connect));
	}

	{
		auto pick = std::make_unique<SerialContainer>("pick object");
		pickPlaceTask->properties().exposeTo(pick->properties(), { "eef", "group", "ik_frame"});
		pick->properties().configureInitFrom(Stage::PARENT, { "eef", "group", "ik_frame"});

		// ---------------------- approach the object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesianPlanner_);
			stage->properties().set("link", ikFrame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.15);

			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			pick->insert(std::move(stage));
		}

		// ---------------------- generate grasp poses ---------------------- //
		{
			// Sample grasp pose
			auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setPreGraspPose(pregrasp);

			stage->setObjectHeight(boundingBoxSrv.response.z);
			stage->setObject(object);
			stage->setAngleDelta(M_PI / 2);
			stage->setMonitoredStage(current_state);  // Hook into current state

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(100);
			wrapper->setMinSolutionDistance(1.0);
	
			wrapper->setIKFrame(panda_hand_transform_, ikFrame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			pick->insert(std::move(wrapper));
		}

		// ---------------------- allow collision (hand object) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(
			    object, pickPlaceTask->getRobotModel()->getJointModelGroup(eef_)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			pick->insert(std::move(stage));
		}

		// ---------------------- close hand ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", gripper_planner_);
			stage->setGroup(eef_);
			stage->setGoal(grasp);
			pick->insert(std::move(stage));
		}

		// ---------------------- attach object to gripper ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object, ikFrame_);
			current_state = stage.get();
			pick->insert(std::move(stage));
		}

		// ---------------------- allow collision (object support) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object }, supportId, true);
			pick->insert(std::move(stage));
		}

		// ---------------------- lift object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.05);
			stage->setIKFrame(ikFrame_);


			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "panda_link0";
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			pick->insert(std::move(stage));
		}

		// ---------------------- forbid collision (object support) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
			stage->allowCollisions({ object }, supportId, false);
			pick->insert(std::move(stage));
		}

		// Add grasp container to task
		pickPlaceTask->add(std::move(pick));
	}

	{
		// connect to place
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipelinePlanner_},{eef_, gripper_planner_}};
		auto connect = std::make_unique<stages::Connect>("connect to place", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
	  	connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
		pickPlaceTask->add(std::move(connect));
	}

	{
		auto place = std::make_unique<SerialContainer>("place object");

		pickPlaceTask->properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
		place->properties().configureInitFrom(Stage::PARENT, {"eef", "group", "ik_frame"});

		{
			auto stage = std::make_unique<stages::GenerateCustomPose>("place the object");
			stage->setCustomPoses(placePoses);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setMonitoredStage(current_state);

			auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage) );
			wrapper->setMaxIKSolutions(32);
			wrapper->setIKFrame(panda_hand_transform_, ikFrame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group", "ik_frame" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		{
			auto stage = std::make_unique<stages::MoveTo>("release object", gripper_planner_);
			stage->setGroup(eef_);
			stage->setGoal(ungrasp);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
			stage->detachObject(object, ikFrame_);
			place->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat from object", cartesianPlanner_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.02, 0.20);
			stage->setIKFrame(ikFrame_);

			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = ikFrame_;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}
		pickPlaceTask->add(std::move(place));
	}
}

 /**
 * \fn void motionPlanning::updateWorld(ros::ServiceClient& saClient)
 * \brief Function to ask ontologenius about object id/meshes that are on the table then ask the situation assessment their positions, and add them to moveit planning scene
 *
 * \param saClient Handle on the situation assesment service to get poses of the object in the scene.
 *
 * \return 0 if everything went fine,
 * 		   1 if failed to get transform between map and base_footprint
 * 		   2 if failed to get meshes from ontologenius
 * 		   3 if failed to get Ids from situation assesment service
 */
int motionPlanning::updateWorld(ros::ServiceClient& saClient)
{
	ROS_INFO_STREAM("[updateWorld]=========[UPDATE OF THE WORLD INCOMING]=========");

    // Mesh related variable 
	shape_msgs::Mesh mesh;
  	shapes::ShapeMsg mesh_msg;
	shapes::Mesh* m;
	std::string meshURI;
	std::vector<std::string> meshTemp;

	// object dimension related variable 
	std::string dimensionStringRaw;
	std::vector<std::string> dimensionTemp;
	std::vector<float> objDimensions;

    // Variable to store the collision object 
	// that will be added to planning scene
	moveit_msgs::CollisionObject collisionObj;
	moveit_msgs::CollisionObject collisionTable;


	// variable to store the collision object poses
	// before and after transformation into base_footprint frame
	geometry_msgs::PoseStamped colliObjPosetransformed;
	geometry_msgs::PoseStamped colliObjPoseUntransformed;

	// vector to store the object ids
	// sent back by ontologenius
	std::vector<std::string> objIds;

	// variable to store the request
	// for poses to the situation assesment
	pr2_motion_tasks_msgs::GetPose srv;

	// Before doing anything delete all the 
	// non attached object from the scene to avoid artifacts.
	std::vector<std::string> objToDelete = planning_scene_interface_.getKnownObjectNames();
	std::map< std::string,moveit_msgs::AttachedCollisionObject> knownAttachedObj = planning_scene_interface_.getAttachedObjects();
	std::map< std::string,moveit_msgs::AttachedCollisionObject>::iterator it;
	for (int i =0; i < objToDelete.size(); i++)
	{
		// Only delete not attached object
		it = knownAttachedObj.find(objToDelete[i]);
		if (it == knownAttachedObj.end())
		{
			collisionObj.id = objToDelete[i];
			collisionObj.operation = collisionObj.REMOVE;
			planning_scene_interface_.applyCollisionObject(collisionObj);
		}
	}

	// Add table as fixed object 
	/*collisionTable.id = "table";
	collisionTable.header.frame_id = "world";
	collisionTable.operation = collisionTable.ADD;
	shape_msgs::SolidPrimitive primitiveTable;
	primitiveTable.type = primitiveTable.BOX;
	primitiveTable.dimensions.resize(3);
	primitiveTable.dimensions[0] = 1.68;
	primitiveTable.dimensions[1] = 0.83;
	primitiveTable.dimensions[2] = 0.768;
	collisionTable.primitives.push_back(primitiveTable);
	geometry_msgs::Pose table_pose;
	table_pose.position.x = 0.0;
	table_pose.position.y = 0.0;
	table_pose.position.z = 0.0;
	table_pose.orientation.x = 0.0;
	table_pose.orientation.y = 0.0;
	table_pose.orientation.z = 0.0;
	table_pose.orientation.w = 1.0;
	collisionTable.primitive_poses.push_back(table_pose);
	planning_scene_interface_.applyCollisionObject(collisionTable);*/

	// Ask Onotologenius all the objects that are cubes
	objIds = onto_.individuals.getType("Cube");

	ROS_INFO_STREAM("[updateWorld] =========[There is " << objIds.size() << " cubes in the scene]=========");

	// Ask situation assesment about poses
	// of object that are on the previous furniture
	srv.request.ids = objIds;
	if (saClient.call(srv))							// Transform pose given by UWDS from map to basefootprint
	{
		for (int i=0; i < objIds.size(); i++)
		{
			ROS_INFO_STREAM("[updateWorld] =========[Id is " << objIds[i] << " ]=========");

			// Clear vector to be able to reuse it
			collisionObj.meshes.clear();
			collisionObj.mesh_poses.clear();

			collisionObj.primitives.clear();
			collisionObj.primitive_poses.clear();

			// Fill in mesh URI (ask ontology or get it from the cache)
			//Verify if frame_id isn't empty
			// Situation assesment publish with frame_id as /map so transform to base_footprint
			if(!srv.response.poses[i].header.frame_id.empty())
			{
				// Check object is already known and if a mesh is already in the map
				if (objMeshMap_.find(objIds[i]) != objMeshMap_.end())
				{
					// Mesh URI is already known so get it from the map
					m = shapes::createMeshFromResource(objMeshMap_.at(objIds[i]));

					shapes::constructMsgFromShape(m, mesh_msg);
					mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

					// Add the mesh to the Collision object message
					collisionObj.meshes.push_back(mesh);

					// Do the transform only if frame_id isn't fixedFrame_ already
					if(srv.response.poses[i].header.frame_id != fixedFrame_)
					{
						colliObjPoseUntransformed.pose = srv.response.poses[i].pose;
						colliObjPoseUntransformed.header.frame_id = srv.response.poses[i].header.frame_id;

						getPoseIntoBase(colliObjPoseUntransformed,colliObjPosetransformed);
												
						collisionObj.mesh_poses.push_back(colliObjPosetransformed.pose);
					}
					else
					{
						collisionObj.mesh_poses.push_back(srv.response.poses[i].pose);
					}
				}
				// Object is a mesh and unknown so get URI and create mesh
				else if (onto_.individuals.getOn(objIds[i],"hasMesh").size() > 0)
				{
					// Ask ontologenius for mesh URI
					meshTemp = onto_.individuals.getOn(objIds[i],"hasMesh");
					if(meshTemp.size() > 0)
					{
						meshURI = meshTemp[0];

						size_t pos = meshURI.find("string#");
						if (pos != std::string::npos)
						{
							// If found then erase it from string
							meshURI.erase(pos, std::string("string#").length());
						}

						ROS_INFO_STREAM("[updateWorld] ObjId is [" << objIds[i] << "]" );
						ROS_INFO_STREAM("[updateWorld] MESH_URI is [" << meshURI << "]" );

						m = shapes::createMeshFromResource(meshURI);

						// And add it to the map
						objMeshMap_.insert(std::make_pair<std::string,std::string>((std::string)objIds[i],(std::string)meshURI));

						shapes::constructMsgFromShape(m, mesh_msg);
						mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

						// Add the mesh to the Collision object message
						collisionObj.meshes.push_back(mesh);

						// Do the transform only if frame_id isn't fixedFrame_ already
						if(srv.response.poses[i].header.frame_id != fixedFrame_)
						{
							colliObjPoseUntransformed.pose = srv.response.poses[i].pose;
							colliObjPoseUntransformed.header.frame_id = srv.response.poses[i].header.frame_id;

							getPoseIntoBase(colliObjPoseUntransformed,colliObjPosetransformed);
													
							collisionObj.mesh_poses.push_back(colliObjPosetransformed.pose);
						}
						else
						{
							collisionObj.mesh_poses.push_back(srv.response.poses[i].pose);
						}
					}
					else
					{ 
						ROS_ERROR_STREAM("[updateWorld] Error when getting mesh URI from Ontologenius");
						continue;
					}
				}
				// Check if object has shape defined in ontology
				else if (onto_.individuals.getOn(objIds[i],"hasShape").size() > 0)
				{
					ROS_WARN_STREAM("[updateWorld] No meshes were returned by Ontologenius...");	
					ROS_WARN_STREAM("[updateWorld] Asking for dimensions if objects comes from Robosherlock...");

					collisionObj.primitives.resize(1);

					// Do the transform only if frame_id isn't base_footprint already
					if(srv.response.poses[i].header.frame_id != fixedFrame_)
					{
						colliObjPoseUntransformed.pose = srv.response.poses[i].pose;
						colliObjPoseUntransformed.header.frame_id = srv.response.poses[i].header.frame_id;

						getPoseIntoBase(colliObjPoseUntransformed,colliObjPosetransformed);
												
						collisionObj.primitive_poses.push_back(colliObjPosetransformed.pose);
					}
					else
					{
						collisionObj.primitive_poses.push_back(srv.response.poses[i].pose);
					}

					// Set the type of object 
					if(onto_.individuals.getOn(objIds[i],"hasShape")[0] == "box")
					{
						collisionObj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
						
						collisionObj.primitives[0].dimensions.resize(3);
						sscanf(onto_.individuals.getOn(objIds[i],"hasDimensionX")[0].c_str(), "float#%lf",&collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]);
						sscanf(onto_.individuals.getOn(objIds[i],"hasDimensionY")[0].c_str(), "float#%lf",&collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
						sscanf(onto_.individuals.getOn(objIds[i],"hasDimensionZ")[0].c_str(), "float#%lf",&collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]);
					}
					else if (onto_.individuals.getOn(objIds[i],"hasShape")[0] == "round")
					{
						collisionObj.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
						
						collisionObj.primitives[0].dimensions.resize(2);
						sscanf(onto_.individuals.getOn(objIds[i],"hasDimensionY")[0].c_str(), "float#%lf",&collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]);
						sscanf(onto_.individuals.getOn(objIds[i],"hasDimensionZ")[0].c_str(), "float#%lf",&collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]);

						collisionObj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]/=2.0;
					}
					else
					{
						ROS_ERROR("[updateWorld] No shape specified for Robosherlock objects....");
						continue;
					}	
				}
				else
				{
					ROS_ERROR_STREAM("[updateWorld] No shape, no mesh specified for object with id [" << objIds[i] << "]");
					continue;
				}
											
				// Set object id
				collisionObj.id = objIds[i];

				collisionObj.operation = collisionObj.ADD;
			
				// Set frame_id to fixedFrame_ as it has been transformed
				collisionObj.header.frame_id = fixedFrame_;
				// Add synchronously the collision object to planning scene (wait for it to be added before continuing)
				planning_scene_interface_.applyCollisionObject(collisionObj);
				ROS_INFO_STREAM("[updateWorld] Successfully added to scene object with ID [" + collisionObj.id + "]");
			}
			else
			{
				ROS_WARN_STREAM("[updateWorld] Error while updating the world, frame_id is empty...");
				continue;
			}
			ROS_INFO_STREAM("----------------------------------------------------------------------------");
		}
	}
	else
	{
		ROS_ERROR_STREAM("[updateWorld] =========[Call of situation assesment service failed to get pose for objects !]=========");
		return -6 ;
	}
	
	ROS_INFO_STREAM("###########################################################################################");	
	ROS_INFO_STREAM("[updateWorld] =========[WORLD HAS BEEN UPDATED]=========");
	return 0;
}

 /**
 * \fn void motionPlanning::taskStatisticCallback(const moveit_task_constructor_msgs::TaskStatisticsConstPtr& taskStat)
 * \brief Callback that is used to send back progress value when planning for a task
 *
 * \param taskStat Pointer to get the number of stages already solved
 */
void motionPlanning::taskStatisticCallback(const moveit_task_constructor_msgs::TaskStatisticsConstPtr& taskStat)
{
	pr2_motion_tasks_msgs::planFeedback planFeedback;
	int progressValue=0;

	// check how much solutions we have found (number of stages solved) in comparison with the number of max solution
	// and convert it to percentage
	progressValue = (int)(((double)taskStat->stages[0].solved.size()/NUMBER_OF_MAX_SOLUTION)*100.0);

	planFeedback.status = progressValue;
	planServer_->publishFeedback(planFeedback);
}

 /**
 * \fn void solutionCallback(const moveit_task_constructor_msgs::SolutionConstPtr& solution, int& cost)
 * \brief Callback that is used to ask Moveit Task Constructor about the cost of the choosen solution (least cost for now)
 *
 * \param solution Solutions lists found during the last planning
 * \param cost Cost that will be sent back when planning is done
 */
void solutionCallback(const moveit_task_constructor_msgs::SolutionConstPtr& solution, int& cost)
{
	// Get the cost of the the first solution (the one with least cost)
	cost = solution->sub_solution[0].info.cost;
}

 /**
 * \fn void motionPlanning::getPoseIntoBase(const geometry_msgs::PoseStampedConstPtr& in_pose, geometry_msgs::PoseStamped* out_pose)
 * \brief Function to get input pose into base_footprint frame (to get same orientation everytime)
 *
 * \param in_pose Input pose unstransformed
 * \param out_pose Pose transformed into base_footprint frame
 */
void motionPlanning::getPoseIntoBase(geometry_msgs::PoseStamped in_pose, geometry_msgs::PoseStamped& out_pose)
{
	geometry_msgs::TransformStamped transform;

	try
	{
		if (in_pose.header.frame_id[0] == '/')
			in_pose.header.frame_id = in_pose.header.frame_id.substr(1);

		transform = tfBuffer_.lookupTransform(fixedFrame_,in_pose.header.frame_id, ros::Time(0),ros::Duration(5.0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN_STREAM("[getPoseIntoBase] Exception while looking up for transform between " << in_pose.header.frame_id << "and " << fixedFrame_ << " [" << ex.what() << "]");
		return ;
	}

	tf2::doTransform(in_pose,out_pose,transform);
}

void motionPlanning::planPreemptCallback()
{
	ROS_WARN("[planPreemptCallback] GOAL IS BEEING PREEMPTED");
	lastPlannedTask_->preempt();
}

 /**
 * \fn void motionPlanning::planCallback(const pr2_motion_tasks_msgs::planGoalConstPtr& goal, ros::ServiceClient& saClient)
 * \brief Callback that is called when supervisor ask for a plan
 *
 * \param goal Goal to be executed. Contains action to be planned (pick, place, move) and other parameter depending on the action
 * \param saClient Service handle to pass on to the update world function
 */
void motionPlanning::planCallback(const pr2_motion_tasks_msgs::planGoalConstPtr& goal, ros::ServiceClient& saClient)
{
 	pr2_motion_tasks_msgs::planResult planResult;
	pr2_motion_tasks_msgs::planFeedback planFeedback;


	std::vector<geometry_msgs::PoseStamped> customPoses;
    geometry_msgs::PoseStamped customPose;

	geometry_msgs::PoseStamped transformedPose;

	std::vector<std::string> supportSurfaceId;
	std::vector<std::string> boxesIds;
	std::vector<std::string> objInBoxIds;
	moveit_msgs::CollisionObject collisionObj;

	int updateWorldResult = 0;

	std::string taskName;

	// Only update the world for pick or drop actions
	if((goal->action == "pick")        || 
	   (goal->action == "pick_dt")     ||
	   (goal->action == "pickPlace")   ||
	   (goal->action == "pickAuto")    ||
	   (goal->action == "pickDual")    || 
	   (goal->action == "updateWorld") || 
	   (goal->action == "drop"))
	{
		// Update the world before doing anything
		updateWorldResult = updateWorld(saClient);

		if(updateWorldResult == -6)
		{
			planResult.error_code = -6;
			planServer_->setAborted(planResult);
			return;
		}
		else if (goal->action == "updateWorld")
		{
			planResult.error_code = updateWorldResult;
			planServer_->setSucceeded(planResult);
			return;
		}

		if((goal->action == "pickDual"))
		{
			// check if objId is a box. If we want to pick a box, we need to delete temporarily the object that are inside
			if(onto_.individuals.isA(goal->objId,"Box"))
			{
				// the object we are trying to pick is a box, delete all object that are inside it
				objInBoxIds = onto_.individuals.getFrom("isIn",goal->objId);

				if(objInBoxIds.size() != 0)
				{
					ROS_ERROR_STREAM("[planCallback] --=========[There is " << objInBoxIds.size() << " object in " << goal->objId << " . Deleting them before planning pick]=========--");
					
					for (int i =0; i < objInBoxIds.size(); i++)
					{
						ROS_WARN_STREAM("[planCallback] --========= DELETING [" << objInBoxIds[i] << "] from the scene =========--");

						collisionObj.id = objInBoxIds[i];
						collisionObj.operation = collisionObj.REMOVE;
						planning_scene_interface_.applyCollisionObject(collisionObj);
					}
				}
			} 
		}
	}

	
	
	taskArmGroup_ = goal->planGroup;


	// reset the task to avoid problem on introspection
	lastPlannedTask_.reset();

	// Keep track of the object id 
	// to be able to detach it from gripper
	// if the pick didn't really happen (perception error)
	taskObjId_ = goal->objId;

	if(goal->action == "pick")
	{

		getPoseIntoBase(goal->pose,transformedPose);

		transformedPose.header.frame_id = baseFrame_;

		customPoses.push_back(transformedPose);

		taskName = goal->action + "_" + goal->objId;

		// Create Task
		lastPlannedTask_ = std::make_unique<Task>();
		lastPlannedTask_->setName(taskName);
		createPickTaskCustom(lastPlannedTask_,taskArmGroup_,goal->objId,SUPPORT_SURFACE, customPoses);
	}
	//====== Create a pick/place task with custom place pose ======//
    else if(goal->action == "pickPlace")
	{
		getPoseIntoBase(goal->pose,customPose);

		customPose.header.frame_id = baseFrame_;

		customPoses.push_back(customPose);

		taskName = goal->action + "_" + goal->objId;

		// Create Task
		lastPlannedTask_ = std::make_unique<Task>();
		lastPlannedTask_->setName(taskName);
		createPickPlaceTask(lastPlannedTask_,taskArmGroup_,goal->objId,SUPPORT_SURFACE, customPoses);
	}	
	//====== Create a pick task with generated pick pose ======//
	else if(goal->action == "pickAuto")
	{
		taskName = goal->action + "_" + goal->objId;

		// Create Task
		lastPlannedTask_ = std::make_unique<Task>();
		lastPlannedTask_->setName(taskName);
		createPickTask(lastPlannedTask_,taskArmGroup_,goal->objId,SUPPORT_SURFACE);
	}
	//====== Create a place task with custom place pose ======//
	else if(goal->action == "place")
	{

		getPoseIntoBase(goal->pose,transformedPose);

		transformedPose.header.frame_id = baseFrame_;
		//transformedPose.pose.position.z += 0.05;
	
		customPoses.push_back(transformedPose);

		taskName = goal->action + "_" + goal->objId;

		// Create Task
		lastPlannedTask_ = std::make_unique<Task>();
		lastPlannedTask_->setName(taskName);
		createPlaceTask(lastPlannedTask_, taskArmGroup_, goal->objId, customPoses);
	}
	//====== Create a move task with custom move goal pose or a predefined moveit pose ======//
	else if (goal->action == "move")
	{
		taskName = goal->action + "_" + goal->planGroup;
		// Create Task
		lastPlannedTask_ = std::make_unique<Task>();
		lastPlannedTask_->setName(taskName);

		if(goal->predefined_pose_id.empty())
		{
			createMoveTask(lastPlannedTask_, taskArmGroup_,goal->pose);
		}
		else
		{
			if((goal->predefined_pose_id == "left_arm_home") && (taskArmGroup_ == ""))
			{
				taskArmGroup_ = "left_arm";
			}
			else if((goal->predefined_pose_id == "right_arm_home") && (taskArmGroup_ == ""))
			{
				taskArmGroup_ = "right_arm";
			}
			createMovePredefinedTask(lastPlannedTask_, taskArmGroup_,goal->predefined_pose_id);
		}
	}
	else
	{
		ROS_ERROR_STREAM("[planCallback] Unknown action provided. Available action are pick, pickAuto, pick_dt, pickDual, move, drop, place, place_dt");
		return;
	}

	// Create Thread to handle the feedback process 
	std::string statTopic = "/panda_tasks_node/" + taskName + "/statistics";
	ros::Subscriber sub = nh_.subscribe<moveit_task_constructor_msgs::TaskStatistics>(statTopic, 10, &motionPlanning::taskStatisticCallback, this);

	try
	{
		ROS_INFO_STREAM("[planCallback] Begin planning of task [" << taskName << "] !");
		planFeedback.status = 0;
		planServer_->publishFeedback(planFeedback);

		if(lastPlannedTask_->plan(NUMBER_OF_MAX_SOLUTION) && !planServer_->isPreemptRequested())
		{
			ROS_INFO_STREAM("[planCallback] Planning of task [" << taskName << "] SUCCEEDED !");

			planResult.error_code = 1;
			planResult.cost = lastPlannedTask_->solutions().front()->cost();
			planResult.armUsed = taskArmGroup_;

			// As there might be less solution found 
			// than the max provided, the feedback 
			// can end below 100%, so set it to 100%
			// at the end to be more fancy
			planFeedback.status = 100;
			planServer_->publishFeedback(planFeedback);
			planServer_->setSucceeded(planResult);
		}
		else
		{
			planResult.error_code = -1;
			
			if(planServer_->isPreemptRequested())
			{
				ROS_WARN_STREAM("[planCallback] Planning of task [" << taskName << "] PREEMPTED !");
				planServer_->setPreempted(planResult);
			}
			else
			{
				ROS_WARN_STREAM("[planCallback] Planning of task [" << taskName << "] ABORTED !");
				planServer_->setAborted(planResult);
			}
		}
	}
	catch (const InitStageException& e)
	{
		ROS_ERROR_STREAM("[planCallback] Planning of the task [" << taskName << "] failed with error [" << e << "]");
	}

}
 /**
 * \fn void feedbackCb(const moveit_task_constructor_msgs::ExecuteTaskSolutionFeedbackConstPtr& feedback)
 * \brief Callback called when executing a task (unused). This is feedback send by moveit task constructor
 *
 * \param feedback feedback received
 */
void feedbackCb(const moveit_task_constructor_msgs::ExecuteTaskSolutionFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("[feedbackCb] Got Feedback ID  " <<  feedback->sub_id);
  ROS_INFO_STREAM("[feedbackCb] Got Feedback Number  " <<  feedback->sub_no);
}

 /**
 * \fn void doneCb(const actionlib::SimpleClientGoalState& state,const moveit_task_constructor_msgs::ExecuteTaskSolutionResultConstPtr& result, bool& doneFlag)
 * \brief Callback called when execution of task is finished
 *
 * \param state state of the action at the end of the task execution
 * \param result contains the error code
 * \param doneFlag flag to stop the loop (and feedback) when execution is complete
 */
void doneCb(const actionlib::SimpleClientGoalState& state,
            const moveit_task_constructor_msgs::ExecuteTaskSolutionResultConstPtr& result, bool& doneFlag)
{
  ROS_INFO_STREAM("[doneCb] Task execution finished in state : " << state.toString().c_str());
  ROS_INFO_STREAM("[doneCb] Result : " << result->error_code);

  doneFlag = true;
}

 /**
 * \fn void activeCb()
 * \brief Called on when the action is active
 */
void activeCb()
{
  ROS_INFO("[activeCb] Execution of the task just began");
}

 /**
 * \fn void motionPlanning::executeCallback(const pr2_motion_tasks_msgs::executeGoalConstPtr& goal)
 * \brief Callback that is called when asking to execute last planned task
 *
 * \param goal Goal is void as we only allow to execute the last planned task
 */
void motionPlanning::executeCallback(const pr2_motion_tasks_msgs::executeGoalConstPtr& goal)
{
	pr2_motion_tasks_msgs::executeFeedback executeFeedback;
  	pr2_motion_tasks_msgs::executeResult executeResult;

	moveit_msgs::AttachedCollisionObject collisionAttObj;
	moveit_msgs::CollisionObject collisionObj;

	bool doneFlag = false;

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;

	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> executeTask("execute_task_solution", true);
	executeTask.waitForServer();

	// Verify that the last planned task had solutions
	if(lastPlannedTask_->solutions().size() > 0)
	{
		// Fill the solution message
		lastPlannedTask_->solutions().front()->fillMessage(execute_goal.solution);

		ROS_INFO_STREAM("[executeCallback] Sending goal to execute the previous task ["<< lastPlannedTask_->name() << "]");

		// Sending goal to moveit task constructor execution server
		executeTask.sendGoal(execute_goal, boost::bind(&doneCb,_1,_2,boost::ref(doneFlag)), &activeCb, &feedbackCb);
		
		// Keep track of the start time of the action 
		executeFeedback.action_start = ros::Time::now();

		int dummyProgress = 0;
		ros::Rate loop_rate(1);

		// Loop until the end of the task execution
		while(!doneFlag)
		{
			// Send back a dummy progress (increasing number)
			// to be able to detect freezing
			// Moveit Task constructor doesn't have any
			// useful feedback for execution 
			executeFeedback.status = dummyProgress;
			dummyProgress++;
			executeServer_->publishFeedback(executeFeedback);
			if(executeServer_->isPreemptRequested())
			{
				executeTask.cancelGoal();
			}

			// Publish the progress every second
			loop_rate.sleep();
		}

		moveit_msgs::MoveItErrorCodes execute_result = executeTask.getResult()->error_code;

		if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
		{
			ROS_ERROR_STREAM("[executeCallback] Task execution failed and returned: " << executeTask.getState().toString());

			executeResult.error_code = -2;

			if(executeServer_->isPreemptRequested())
			{
				executeServer_->setPreempted(executeResult);
			}
			else
			{
				executeServer_->setAborted(executeResult);
			}
			
		}
		else
		{
			// If task was involving picking or placing an object, check if something has been really attached/detached to the gripper
			// If not, it means that something went wrong in real world, so detach the object from gripper and delete object from moveit world
			/*if(lastPlannedTask_->name().find("pick") != std::string::npos && lastPlannedTask_->name().find("Dual") == std::string::npos && lastPlannedTask_->name().find("lace") == std::string::npos)
			{
				// Ask Ontologenius if something is in left hand
				if(onto_.individuals.getOn("panda_robot","hasInHand").size() > 0 && taskArmGroup_ == "panda_arm")
				{
					executeResult.error_code = 1;
					executeResult.action_end = ros::Time::now();
					executeServer_->setSucceeded(executeResult);
					ROS_INFO_STREAM("Task execution succeeded and returned: " << executeTask.getState().toString());
				}
				// Nothing was really grasped by either arms, so virtually detach the object from gripper and delete it
				else
				{
					//Detach object from gripper 
					collisionAttObj.object.id = taskObjId_;
					collisionAttObj.link_name = "";
					collisionAttObj.object.operation = collisionAttObj.object.REMOVE;
					planning_scene_interface_.applyAttachedCollisionObject(collisionAttObj);	

					// Delete object from moveit world
					collisionObj.id = taskObjId_;
					collisionObj.operation = collisionObj.REMOVE;
					planning_scene_interface_.applyCollisionObject(collisionObj);					
			
					ROS_ERROR_STREAM("Task execution succeeded but situation assesment returned that object isn't held by gripper");
					ROS_ERROR_STREAM("Obj [" << taskObjId_ << "] has been deleted from moveit world");
					executeResult.error_code = -2;
					executeServer_->setAborted(executeResult);
				}
			}	
			else*/
			{
				executeResult.error_code = 1;
				executeResult.action_end = ros::Time::now();
				executeServer_->setSucceeded(executeResult);
				ROS_INFO_STREAM("Task execution succeeded and returned: " << executeTask.getState().toString());
			}
		}
	}
	else
	{
		ROS_ERROR_STREAM("Execution of task failed because the last planned task had no solutions !");
		executeResult.error_code = -3;
		executeServer_->setAborted(executeResult);
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "panda_tasks_node");
	ros::AsyncSpinner spinner(2);
	spinner.start();

	ros::NodeHandle nh("~");

	motionPlanning pr2Motion(nh);

	while(ros::ok());

	return 0;
}
