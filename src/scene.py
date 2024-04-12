#!/usr/bin/env python3
"""
Shows how to use a planning scene in MoveItPy to add collision objects and perform collision checking.
"""

import  time
import  rclpy
from    rclpy.logging import get_logger
from    moveit.planning import MoveItPy
from    moveit.core.kinematic_constraints import construct_joint_constraint

from    geometry_msgs.msg import Pose, PoseStamped
from    moveit_msgs.msg import CollisionObject
from    shape_msgs.msg import SolidPrimitive



class WorldHandler():
    def __init__(   
                    self,
                    planning_scene_monitor,
                ):
        self.planning_scene_monitor = planning_scene_monitor  
        self.scenarios      = [
                                dict(
                                    positions=[
                                        (0.15, 0.1, 0.5),
                                        (0.25, 0.0, 1.0),
                                        ],
                                    dimensions=[
                                        (0.1, 0.4, 0.1),
                                        (0.1, 0.4, 0.1),
                                        ],
                                ),
                            ]
        self.scenario_id    = 0

    def set_scenario(
                        self, 
                        scenario_id:int=None,
                    ):
        if scenario_id is None:
            scenario_id = self.scenario_id
        else:
            assert scenario_id <= len(self.scenarios)
        scenario                        = self.scenarios[scenario_id] 
        collisions                      = self.create_collision_objects(scenario)
        with self.planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collisions)
            scene.current_state.update()  
        # cycle to future default scenario
        self.scenarios = 0 if scenario_id == len(self.scenarios)-1 else scenario_id + 1  


    def create_collision_objects(
                                    self, 
                                    scenario    : dict,
                                    ref_frame   : str="panda_link0"
                                ):
        global CollisionObject, SolidPrimitive
        collision_object                    = CollisionObject()
        collision_object.header.frame_id    = ref_frame
        collision_object.id                 = "boxes" 
        for position, dimensions in zip(scenario['positions'],scenario['dimensions']):
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
                        joint_model_group_name  : str="panda_arm",
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
                                    pose_stamped_msg=self.current_goal_state,#["pose_stamped"], 
                                    pose_link="panda_hand"
                                )
        return pose_stamped_goal


    def check_collisions(
                            self,
                            #joint_model_group_name="panda_arm",
                        ) -> bool:

        robot_collision_status          = False
        with self.planning_scene_monitor.read_only() as scene:
            robot_state                 = scene.current_state
            # original_joint_positions    = robot_state\
            #                                 .get_joint_group_positions(
            #                                     self.current_goal_state.header.frame_id
            #                                     )
            robot_state.set_from_ik(
                                        self.current_goal_state.header.frame_id, 
                                        self.current_goal_state.pose, 
                                        "panda_hand"
                                    )
            robot_state.update()  
            robot_collision_status = scene.is_state_colliding(
                robot_state=robot_state,  
                joint_model_group_name=self.current_goal_state.header.frame_id,
                verbose=True,
            )
            self.logger.info(f"\nRobot is in collision: {robot_collision_status}\n")

        return robot_collision_status

    def plan_and_execute(
                            self,
                            use_collisions_ik=False,
                            sleep_time      : float=1.0
                        ) -> None:

        self.robot.set_start_state_to_current_state()
        if use_collisions_ik:
            if self.check_collisions():
                return

        plan_result     = self.robot.plan()
        time.sleep(2*sleep_time)
        if plan_result:
            self.logger.info("Execute plan")
            trajectory  = plan_result.trajectory
            self.moveit_controller.execute(trajectory, controllers=[])   


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
                                    use_collisions_ik=True,
                                    sleep_time=sleep_time,
                                )       
            time.sleep(2*sleep_time)


def main():
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger                  = get_logger("moveit_py_planning_scene")
    moveit_controller       = MoveItPy(node_name="moveit_py_planning_scene")
    arm                     = moveit_controller.get_planning_component("panda_arm")
    planning_scene_monitor  = moveit_controller.get_planning_scene_monitor()
    
    
    arm.set_start_state(configuration_name="ready")
    world_scenario          = WorldHandler(planning_scene_monitor)
    planner                 = Planner(moveit_controller, planning_scene_monitor, arm, logger)

    world_scenario.set_scenario()  


    goals                   = [
                                (0.5,0.5,0.75,0.5),
#                                (0.25,0.5,0.5,0.5),
#                                (0.25,0.0,0.5,0.5),
#                                (0.25,-0.5,0.5,0.5),
                                (0.5,-0.5,0.75,0.5),
                                (0.5,0.0,0.5,0.5),
                                (0.5,0.0,0.65,0.5),

                            ]

    planner.set_plan_and_execute_goals(goals,sleep_time=1.0)


if __name__ == "__main__":
    main()
