#include <franka/gripper.h>
#include <franka_gripper/franka_gripper.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

#include <functional>


using franka_gripper::GraspAction;
using franka_gripper::HomingAction;
using franka_gripper::MoveAction;
using franka_gripper::StopAction;
using GraspClient = actionlib::SimpleActionClient<GraspAction>;
using HomingClient = actionlib::SimpleActionClient<HomingAction>;
using MoveClient = actionlib::SimpleActionClient<MoveAction>;
using StopClient = actionlib::SimpleActionClient<StopAction>;

/* Una action definisce: Goal, Feedback e Result messages.
Client e Server comunicano attraverso questi campi.

Goal
Un goal può essere inviato ad un ActionServer da un ActionClient e serve per eseguire l'azione.


Feedback
Il feedback fornisce al server un modo per comunicare al client lo stato d'esecuzione del goal.


Result
Il Result è inviato dall'ActionServer al Client al completamento del goal.
E' differente dal feedback perchè questo messaggio viene inviato esattamente una volta.
Il contenuto del result può essere qualsiasi informazione.

Ex se il goal assegnato consiste nel portare la base del robot nella posa x, il result può essere
la posa finale ottenuta.

*/



int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_gripper_node");
  
// create the action client
// true causes the client to spin its own thread
  HomingClient homing_client("franka_gripper/homing",true);
  GraspClient grasp_client("franka_gripper/grasp",true);
  MoveClient move_client("franka_gripper/move",true);
  StopClient stop_client("franka_gripper/stop",true);

  ROS_INFO("Waiting for action server to start.");

// wait for the action server to start
  bool homing_connected_before_timeout = homing_client.waitForServer(ros::Duration(2.0)); //will wait for 2 s
  bool grasping_connected_before_timeout = grasp_client.waitForServer(ros::Duration(2.0));
  ROS_INFO("Action server started, sending goal.");

// Homing del gripper
  homing_client.sendGoal(franka_gripper::HomingGoal());

// wait for the action to return (30 s)
  bool finished_before_timeout = homing_client.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = homing_client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }

  else
    ROS_INFO("Action did not finish before the time out.");

std::cout << " WARNING: This example will move the gripper! "
              << "Press Enter to continue..." << std::endl;

std::cin.ignore();

// Parametri del gripper
  auto grasp_goal = franka_gripper::GraspGoal();
  grasp_goal.width = 0.011; // [m]
  grasp_goal.speed = 0.05; // [m/s] 
  grasp_goal.force = 60.0; // [N]
  grasp_goal.epsilon.inner = 0.002;
  grasp_goal.epsilon.outer = 0.002;

// Invio del comando    
  grasp_client.sendGoal(grasp_goal);

  finished_before_timeout = grasp_client.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = grasp_client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }

  else
    ROS_INFO("Action did not finish before the time out.");

//exit
  return 0;
  
}
