#!/usr/bin/env python3
"""
Shows how to use a planning scene in MoveItPy to add collision objects and perform collision checking.
"""

import  time
import  numpy as np
import  rclpy
from    rclpy.logging import get_logger

from    moveit.planning import MoveItPy
from    moveit.core.kinematic_constraints import construct_joint_constraint
from    moveit.core.robot_state import RobotState

from    geometry_msgs.msg import Pose, PoseStamped
from    moveit_msgs.msg import CollisionObject
from    shape_msgs.msg import SolidPrimitive


class WorldHandler():
    def __init__(   
                    self,
                    planning_scene_monitor,
                    logger,
                ):
        self.planning_scene_monitor = planning_scene_monitor  
        self.logger                 = logger

    def set_scenario(
                        self, 
                        collision_objects   : list=list(),
                    ):
        collisions                  = self.create_collision_objects(collision_objects)
        with self.planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collisions)
            scene.current_state.update()  

    def create_collision_objects(
                                    self, 
                                    collision_objects   : list,
                                    ref_frame           : str="panda_link0",
                                ):
        global CollisionObject, SolidPrimitive
        collision_object                    = CollisionObject()
        collision_object.header.frame_id    = ref_frame
        collision_object.id                 = "boxes" 

        for (position, dimensions) in collision_objects:

            self.logger.info(f"\n{position} {dimensions}")

            box_pose = Pose()
            box_pose.position.x,\
            box_pose.position.y,\
            box_pose.position.z     = position

            box                     = SolidPrimitive()
            box.type                = SolidPrimitive.BOX
            box.dimensions          = dimensions
            
            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def reset_world(    
                    self,
                    ):
        with self.planning_scene_monitor.read_write() as scene:
            scene.remove_all_collision_objects()
            scene.current_state.update()


class Planner():
    def __init__(
                    self,
                    moveit_controller   : MoveItPy,
                    planning_scene_monitor,
                    robot,
                    logger,
                ):
        self.moveit_controller      = moveit_controller
        self.planning_scene_monitor = planning_scene_monitor  
        self.robot                  = robot
        self.current_goal_state     = None
        self.logger                 = logger

    def set_pose_goal(
                        self, 
                        goal                    : tuple=(0.5, 0.5, 0.5, 0.5),
                        joint_model_group_name  : str="panda_arm_hand",
                        end_effector_link       : str="panda_hand",
                    ):
        pose_goal                       = PoseStamped()
        pose_goal.header.frame_id       = joint_model_group_name
        pose_goal.pose.position.x,\
        pose_goal.pose.position.y,\
        pose_goal.pose.position.z,\
        pose_goal.pose.orientation.w    = goal
        pose_stamped_goal               = pose_goal

        self.current_goal_state         = pose_stamped_goal
        self.robot.set_goal_state(
                                    pose_stamped_msg=self.current_goal_state,
                                    pose_link=end_effector_link,
                                )
        return pose_stamped_goal

    def set_plan_and_execute_goals(
                                        self,
                                        goals       : list,
                                        sleep_time  : float=1.0,
                                    ):
        time.sleep(2*sleep_time)
        for goal in goals:
            self.set_pose_goal(
                                goal=goal,
                            )
            self.plan_and_execute(
                                    use_collisions_ik=False,
                                    sleep_time=sleep_time,
                                )       
            time.sleep(2*sleep_time)
            self.reset_robot_pose()
            break

    def plan_and_execute(
                            self,
                            use_collisions_ik=False,
                            sleep_time      : float=1.0
                        ) -> None:
        self.robot.set_start_state_to_current_state()

        plan_result     = self.robot.plan()
        time.sleep(2*sleep_time)
        if plan_result:
            self.logger.info("Execute plan")
            trajectory  = plan_result.trajectory
            self.moveit_controller.execute(trajectory, controllers=[])   

    def reset_robot_pose(   
                            self,
                        ) -> None:

        # PLAN YOUR WAY BACK
        # self.robot.set_goal_state(configuration_name="ready")
        # self.plan_and_execute(use_collisions_ik=False)
        
        # OR, RESET POSITION (ONLY FOR SIMULATION)
        with self.planning_scene_monitor.read_write() as scene:            
            robot_state = scene.current_state

            joint_positions = np.asarray([  
                                             0.000, -0.785, 0.000,\
                                            -2.356,  0.000, 1.571,\
                                             0.785,
                                        ],
                                        dtype=np.float32
                                    )
            robot_state.set_joint_group_positions(
                                                    "panda_arm",                                                    
                                                    joint_positions, 
                                                )
            robot_state.update() 
        self.logger.info(f'robot state should be reset now!')
        time.sleep(2.0)

def main():
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger                  = get_logger("benchmark_script")
    moveit_controller       = MoveItPy(node_name="moveit_py_node")
    arm                     = moveit_controller.get_planning_component("panda_arm")
    planning_scene_monitor  = moveit_controller.get_planning_scene_monitor()
    
    world                   = WorldHandler(planning_scene_monitor, logger)
    planner                 = Planner(moveit_controller, planning_scene_monitor, arm, logger)

    ###################################################################
    # BenchMark Setup
    ###################################################################
    obstacles               = [ # position       ,  dimensions
                                ((0.10,-0.30, 0.65),(0.10, 0.10, 0.60)),
                                ((0.10, 0.30, 0.65),(0.10, 0.10, 0.60))
                            ]

    goal_states             = [
                                (0.50, 0.50, 0.75, 0.50),
                                (0.50,-0.50, 0.75, 0.50),
                                (0.50, 0.00, 0.50, 0.50),
                                (0.50, 0.00, 0.65, 0.50),
                            ]

    ###################################################################
    # Run Benchmark
    ###################################################################
    world.set_scenario(collision_objects=obstacles)  
    planner.set_plan_and_execute_goals(goal_states,sleep_time=1.0)

if __name__ == "__main__":
    main()
