/*********************************************************************
 * Simple Inverse Kinematic Demo for Cobot with MoveIt Visual Tools.
 *
 * Run this demo with an active moveit group (example: simple_ik_launch.py).
 *
 * Adapted from:
 * https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html
 *********************************************************************/

 /********************************************************************
 *WAS GENAU IST EIGENTLICH EIN NODE
 *ein einzelnes,eingenständiges Programm,das im ROS System läuft,
 *ein Baustein in einem verteilten Robotersystem
 *die einzelnen Bauteile eines Roboters Arm,Kamera etc. arbeiten in Nodes
 *damit sie unabhängig voneinander laufen ,diese einzelnen Nodes kommunizieren dann
 *über ROS
 *Vorteil von Nodes:Modularität,Wiederverwendbarkeit,Fehlertoleranz,Verteilt laufend
 *********************************************************************/
#include <rclcpp/rclcpp.hpp>   //Kern API ROS2
#include <geometry_msgs/msg/pose.hpp>  //Pose Objekt(Position,Orientierung)
#include <moveit/move_group_interface/move_group_interface.hpp> //Schnittstelle für Moveit
#include <moveit_visual_tools/moveit_visual_tools.h> //Rviz visualisierung

int main(int argc, char **argv)
{
  // init ROS2 context, create node, logger and run node as thread
  rclcpp::init(argc, argv);
  //erstellen eines nodes simple_ik
  auto simple_ik_node = rclcpp::Node::make_shared("simple_ik_node");
  //erstellen eines nodes logger
  auto logger = rclcpp::get_logger("simple_ik_node_log");

  //ROS2 einmal durchlaufen lassen
  rclcpp::spin_some(simple_ik_node);

  // get move group
  const auto kArmGroup = "arm_group";  //name der gelenkgruppe
  moveit::planning_interface::MoveGroupInterface move_group(simple_ik_node, kArmGroup);

  // handle the visual part (do not start with planning until topic has been subscribed in rviz)
  moveit_visual_tools::MoveItVisualTools visual_tools(simple_ik_node, "world", "visual_tools_topic",
                                                      move_group.getRobotModel());
  RCLCPP_INFO(logger, "Waiting for rviz to subscribe topic: add MarkerArray and select /visual_tools_topic to continue");
  while (simple_ik_node->get_node_graph_interface()->count_subscribers("visual_tools_topic") == 0)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(logger, "Topic subscribed, publishing MarkerArray.");
  visual_tools.deleteAllMarkers();

  //erstellt eine zeilposition für die inverse kinematik
  geometry_msgs::msg::Pose football_pose;
  football_pose.position.x = 0.6;
  football_pose.position.y = 0.3;
  football_pose.position.z = 1.0;

  // publish football
  visual_tools.publishMesh(football_pose, "package://demo/meshes/football.dae", rviz_visual_tools::BLACK, 0.15);
  //das package muss in demo vorhanden sein!!
  visual_tools.trigger();
  // run inverse kinematic solver
  move_group.setPositionTarget(football_pose.position.x, football_pose.position.y, football_pose.position.z);
  move_group.setGoalTolerance(0.05);
  //hier passiert die Planung der Bewegung
  moveit::planning_interface::MoveGroupInterface::Plan reach_football_plan;
  bool success = (move_group.plan(reach_football_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // execute the planned trajectory
  //(wenn planung erfolgreich)
  if (success)
  {
    move_group.execute(reach_football_plan);
  }
  //wenn die planung nicht ausführbar ist z.b weil das ziel außerhalb der reichweite 
  //des roboters ist, dann kann nicht ausgeführt werden
  else
  {
    auto text_pose = football_pose;
    text_pose.position.z = 1.5;
    visual_tools.publishText(text_pose, "Planing failed!", rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
    visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planing failed! Cannot execute trajectory.");
  }

  // exit demo
  rclcpp::shutdown();
  return 0;
}