#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <pr2_motion_tasks_msgs/planAction.h>
#include <pr2_motion_tasks_msgs/executeAction.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>


void waitUser(const std::string msg)
{
    do{
        // Use AINSI Escape code to make the text green in a Linux terminal
        std::cout<<"\033[1;32m\n"
                   "Press ENTER: "<<msg<<" \033[0m\n";
                   ros::spinOnce();
    }while (std::cin.get() != '\n');
}

void scenario_full_pickplace(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction>& planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient)
{
  pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time

  ROS_INFO("Sending pick goal !");

  planGoal.planGroup = "panda_arm";
  planGoal.objId = "cube_r_1";
  planGoal.action = "pickPlace";
  planGoal.pose.header.frame_id = "cube_g";
  planGoal.pose.pose.position.x = 0.0;
  planGoal.pose.pose.position.y = 0.0;
  planGoal.pose.pose.position.z = 0.15;
  planGoal.pose.pose.orientation.x = 0.0;
  planGoal.pose.pose.orientation.y = 0.707;
  planGoal.pose.pose.orientation.z = 0.0;
  planGoal.pose.pose.orientation.w = 0.707;
  planClient.sendGoal(planGoal);
  planClient.waitForResult();

  waitUser("To execute planned trajectory");

  if(planClient.getResult()->error_code == 1)
  {
    executeClient.sendGoal(executeGoal);
    executeClient.waitForResult();
  }
  else
  {
    ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
    return;
  }

  if (executeClient.getResult()->error_code == 1)
  {
    planGoal.planGroup = "panda_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "ready";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    waitUser("To execute planned trajectory");

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Error while executing " << planGoal.action << " " << planGoal.objId);
    return;
  }

}

void scenario_make_pile(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction>& planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient)
{
  pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time

  ROS_INFO("Sending pick goal !");

  planGoal.planGroup = "panda_arm";
  planGoal.objId = "cube_r_1";
  planGoal.action = "pickAuto";
  planClient.sendGoal(planGoal);
  planClient.waitForResult();

  waitUser("To execute planned trajectory");

  if(planClient.getResult()->error_code == 1)
  {
    executeClient.sendGoal(executeGoal);
    executeClient.waitForResult();
  }
  else
  {
    ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
    return;
  }

  if (executeClient.getResult()->error_code == 1)
  {
    planGoal.planGroup = "panda_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "ready";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    waitUser("To execute planned trajectory");

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Error while executing " << planGoal.action << " " << planGoal.objId);
    return;
  }

 
  planGoal.planGroup = "panda_arm";
  planGoal.objId = "cube_r_1";
  planGoal.action = "place";
  planGoal.pose.header.frame_id = "cube_g";
  planGoal.pose.pose.position.x = 0.0;
  planGoal.pose.pose.position.y = 0.0;
  planGoal.pose.pose.position.z = 0.07;
  planGoal.pose.pose.orientation.x = 0.0;
  planGoal.pose.pose.orientation.y = 0.707;
  planGoal.pose.pose.orientation.z = 0.0;
  planGoal.pose.pose.orientation.w = 0.707;
  planClient.sendGoal(planGoal);
  planClient.waitForResult();

   waitUser("To execute planned trajectory");

  if(planClient.getResult()->error_code == 1)
  {
    executeClient.sendGoal(executeGoal);
    executeClient.waitForResult();
  }
  else
  {
    ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.objId);
    return;
  }

  if (executeClient.getResult()->error_code == 1)
  {
    planGoal.planGroup = "panda_arm";
    planGoal.action = "move";
    planGoal.predefined_pose_id = "ready";

    planClient.sendGoal(planGoal);
    planClient.waitForResult();

    waitUser("To execute planned trajectory");

    if(planClient.getResult()->error_code == 1)
    {
      executeClient.sendGoal(executeGoal);
      executeClient.waitForResult();
    }
    else
    {
      ROS_ERROR_STREAM("Error while trying to " << planGoal.action << " " << planGoal.planGroup);
      return;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Error while executing " << planGoal.action << " " << planGoal.objId);
    return;
  }

}

void home_arm(actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> &planClient, actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction>& executeClient)
{
  pr2_motion_tasks_msgs::planGoal planGoal;
  pr2_motion_tasks_msgs::executeGoal executeGoal;

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  planClient.waitForServer(); //will wait for infinite time
  executeClient.waitForServer(); //will wait for infinite time
  planGoal.planGroup = "panda_arm";
  planGoal.action = "move";
  planGoal.predefined_pose_id = "ready";

  planClient.sendGoal(planGoal);
  planClient.waitForResult();

  if(planClient.getResult()->error_code == 1)
  {
  executeClient.sendGoal(executeGoal);
  executeClient.waitForResult();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "actionClient");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle n;

  actionlib::SimpleActionClient<pr2_motion_tasks_msgs::planAction> plan("/panda_tasks_node/plan", true);

  actionlib::SimpleActionClient<pr2_motion_tasks_msgs::executeAction> execute("/panda_tasks_node/execute", true);

  //scenario_make_pile(plan,execute);
  scenario_full_pickplace(plan,execute);
  return 0;
}
