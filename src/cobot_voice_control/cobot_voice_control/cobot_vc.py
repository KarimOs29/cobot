#!/usr/bin/env python3

import time
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory



from moveit.planning import MoveItPy
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive

from cobot_voice_control.roboter_position import verarbeite_befehl

#create class which inherit form ros2 node
class SprachsteuerungNode(Node):
    def __init__(self):
        super().__init__('sprachsteuerung_node')  #naming
        self.subscription = self.create_subscription(  #registration subscriber
            String,                                         #Nachrichtentyp
            'sprachbefehl',                                 #Topic-Name
            self.listener_callback,                         #wird bei jeder eingehenden nachricht ausgelöst wird
            10)                                            
#Queue-Größe(Warteschlange)

        #self.subscription  # prevent unused warning
        #self.result
        self.result = None

    #ohne diese funktion abonnieren wir das topic aber nichts passiert weil keine
    #Callback-Funktion verknüpft ist
    def listener_callback(self, msg):
        befehl = msg.data  #die eigentliche nachricht z.b. gehe nachrechts wird jetzt in befehl gespeichert
        #self.get_logger().info(f'Empfange Sprachbefehl: "{befehl}"')

        #hier wird der befehl letzenlich verarbeitet und
        #in eine bewegung umgewandelt
        self.result = verarbeite_befehl(befehl)
        print('l49: ', self.result) #ausgabe result zur kontrolle

    def get_result(self):
        return self.result

    def clear_result(self):
        self.result = None





def plan_and_execute(
    robot,
    planning_component,
    logger,
    sleep_time=0.0
):

    logger.info("Planning Trajectory")
    plan_result = planning_component.plan()  # <- KEINE Parameter!

    if plan_result:
        logger.info("Executing Plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)







    # hier unsere funktion move_relative drum
def cobot_move_relative(cobot, cobot_arm, logger, dx, dy, dz):
    current_state = cobot_arm.get_start_state()
    current_pose = current_state.get_pose("TCP")

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose = current_pose
    pose_goal.pose.position.x = pose_goal.pose.position.x + dx
    pose_goal.pose.position.y = pose_goal.pose.position.y + dy
    pose_goal.pose.position.z = pose_goal.pose.position.z + dz

    # set a tolerance for the goal
    # this is required in order to relax the constraints on the planner
    # if not set, there will be hardly a solution found
    # this might be an issue with our cobot but could also be an issue
    # resulting from the usage of std param values for the planner config
    constraints = Constraints()
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = "base_link"
    position_constraint.link_name = "TCP"
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.1]  # 10 cm radius
    position_constraint.constraint_region.primitives.append(sphere)
    position_constraint.constraint_region.primitive_poses.append(pose_goal.pose)
    position_constraint.weight = 0.8
    constraints.position_constraints.append(position_constraint)
    # set the goal state with tolerance of 5cm

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "base_link"
    orientation_constraint.link_name = "TCP"
    orientation_constraint.orientation = pose_goal.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.3  # 0.2 radians
    orientation_constraint.absolute_y_axis_tolerance = 0.3
    orientation_constraint.absolute_z_axis_tolerance = 0.3
    orientation_constraint.weight = 1.0
    constraints.orientation_constraints.append(orientation_constraint)

    from moveit.core.kinematic_constraints import construct_joint_constraint

    joint1_constraint = JointConstraint()

    joint1_constraint.joint_name = "joint_0"
    joint1_constraint.position = 0.0
    joint1_constraint.tolerance_below = 0.1
    joint1_constraint.tolerance_above = 0.1
    joint1_constraint.weight = 0.5

    constraints.joint_constraints.append(joint1_constraint)

    cobot_arm.set_goal_state(motion_plan_constraints=[constraints])

    # plan to goal
    plan_and_execute(cobot, cobot_arm, logger, sleep_time=2.0)



def main():

    print("robot_control.py wurde gestartet und initialisiert MoveIt...", flush=True)

            # === MoveIt Setup ===
    moveit_config = (
        MoveItConfigsBuilder(robot_name="cobot", package_name="cobot_model")
        .robot_description(
            file_path=get_package_share_directory("cobot_model")
            + "/urdf/festo_cobot_model.urdf.xacro"
        )
        .robot_description_semantic(
            file_path=get_package_share_directory("cobot_moveit_config")
            + "/config/festo_cobot_model.srdf"
        )
        .robot_description_kinematics(
            file_path=get_package_share_directory("cobot_moveit_config")
            + "/config/kinematics.yaml"
        )
        .joint_limits(
            file_path=get_package_share_directory("cobot_moveit_config")
            + "/config/joint_limits.yaml"
        )
        .trajectory_execution(
            file_path=get_package_share_directory("py_demo")
            + "/config/moveit_controllers.yaml"
        )
        .pilz_cartesian_limits(
            file_path=get_package_share_directory("py_demo")
            + "/config/pilz_cartesian_limits.yaml"
        )
        .moveit_cpp(
            file_path=get_package_share_directory("py_demo")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
        .to_dict()
    )

    # === MoveItPy starten ===
    cobot = MoveItPy(
        node_name="moveit_py",
        config_dict=moveit_config
    )
    cobot_arm = cobot.get_planning_component("arm_group")
    #logger.info("✅ MoveItPy initialisiert")

#wir initialsieren ros2 nodes,pub,sub, usw...
    rclpy.init()
#wir erzeugen das node mit namen subsc springen also hoch in die klasse
    subsc = SprachsteuerungNode()

#node läuft jetzt dauerhaft und wartet auf topic
#wenn topic -> listener_callback
    logger = get_logger("moveit_py.sprachsteuerung")


    ###########################################################################
    # Option 2: run FK
    ###########################################################################

    # set plan start state to current state
    #cobot_arm.set_start_state_to_current_state()
    #robot_model = cobot.get_robot_model()
    #robot_state = RobotState(robot_model)

    # applying FK is handled using joint constraints
    # note: unlike IK, running FK almost always works as long as jointlimits are
    # respected. Refer file cobot_moveit_config/config/joint_limits.yaml
    #from moveit.core.kinematic_constraints import construct_joint_constraint

    # move to stable position
  

    # plan to goal
    #plan_and_execute(cobot, cobot_arm, logger, sleep_time=3.0)

    while True:
        
        rclpy.spin_once(subsc)
        result = subsc.get_result()

        if result is not None:
            print(result)
            #dx = result
            if result['befehl']=='bewegung':
                dx = result['position'][0]
                dy = result['position'][1]
                dz = result['position'][2]
                cobot_move_relative(cobot, cobot_arm, logger, dx, dy, dz)
            elif result['befehl']=='greifer':
                print('not yet implemented')
            elif result['befehl']=='UNKNOWN_COMMAND':
                print('unknown command')
            else: 
                print('fehlgeschlagen')
                sys.exit(-1)
            subsc.clear_result()

    rclpy.shutdown()


if __name__ == "__main__":
    main()