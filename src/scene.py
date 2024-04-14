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


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


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
            self.logger.info(f"{bcolors.WARNING}Planning and executing goal:{bcolors.ENDC} {goal}")
            self.set_pose_goal(
                                goal=goal,
                            )
            result  = self.plan_and_execute(
                                    use_collisions_ik=True,
                                    sleep_time=sleep_time,
                                ) 
            self.logger.info(f"{bcolors.OKGREEN if result else bcolors.FAIL}                    \
                                {'Goal execution done!' if result else 'Goal failed!'}\
                                {bcolors.ENDC}")
            # time.sleep(sleep_time)
            # self.reset_robot_pose()
            time.sleep(2*sleep_time)

    def plan_and_execute(
                            self,
                            use_collisions_ik=False,
                            sleep_time      : float=1.0
                        ) -> bool:
        self.robot.set_start_state_to_current_state()

        if use_collisions_ik:
            collision = self.check_for_collision()
            if collision:
                return False

        # time.sleep(sleep_time)        
        plan_result     = self.robot.plan()
        # time.sleep(sleep_time)        
        if plan_result:
            trajectory  = plan_result.trajectory
            self.moveit_controller.execute(trajectory, controllers=[])   
        return True

    def check_for_collision(self):
        with self.planning_scene_monitor.read_only() as scene:
            robot_state = scene.current_state
            original_joint_positions = robot_state \
                                        .get_joint_group_positions("panda_arm")
            robot_state.set_from_ik(
                                        "panda_arm", 
                                        self.current_goal_state.pose, 
                                        "panda_hand"
                                    )
            robot_state.update()  
            robot_collision_status = scene.is_state_colliding(
                                        robot_state=robot_state, 
                                        joint_model_group_name="panda_arm", 
                                        verbose=True
                                    )
            robot_state.set_joint_group_positions(
                "panda_arm",
                original_joint_positions,           # reset to original
            )
        self.logger.info(f"\n{bcolors.WARNING}Robot is in collision:{bcolors.ENDC}          \
                            {bcolors.FAIL if robot_collision_status else bcolors.OKGREEN}   \
                            {robot_collision_status}{bcolors.ENDC}\n"
                        )
        return robot_collision_status

    def reset_robot_pose(   
                            self,
                            random  : bool=False,
                        ) -> None:

        joint_positions = np.asarray([  
                                         0.000, -0.785, 0.000,\
                                        -2.356,  0.000, 1.571,\
                                         0.785,
                                    ],
                                    dtype=np.float32
                                )
        with self.planning_scene_monitor.read_write() as scene:
            # update scene's model
            robot_state = scene.current_state
            robot_state.set_joint_group_positions(
                "panda_arm",
                joint_positions,
            )
            robot_state.update()
        self.logger.info(f'{bcolors.OKCYAN}Robot state should be reset now!{bcolors.ENDC}')
        

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
    # Create and Run Benchmark
    ###################################################################
    world.set_scenario(collision_objects=obstacles)  
    planner.set_plan_and_execute_goals(goal_states,sleep_time=1.0)

if __name__ == "__main__":
    main()
