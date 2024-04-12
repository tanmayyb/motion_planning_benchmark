#!/usr/bin/env python3
"""
Shows how to use a planning scene in MoveItPy to add collision objects and perform collision checking.
"""

import time
import rclpy
from rclpy.logging import get_logger

from moveit.planning import MoveItPy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive



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

    def plan_and_execute(
                            self,
                            use_collisions_ik=True,
                            sleep_time      : float=3.0
                        ):

        time.sleep(sleep_time)
        self.robot.set_start_state(configuration_name="ready")

        if use_collisions_ik:
            self.set_pose_goal()
            self.check_collisions()
        else:
            self.robot.set_goal_state(configuration_name="extended")
            plan_result     = self.robot.plan()
            if plan_result: # if result is successful
                self.logger.info("Execute plan")
                trajectory  = plan_result.trajectory
                self.moveit_controller.execute(trajectory, controllers=[])   

        time.sleep(sleep_time)


    def set_pose_goal(
                    self, 
                    goal    : tuple=(0.25, 0.25, 0.5, 1.0),
                    ):
        # self.robot.set_start_state(configuration_name="current")
        # self.robot.set_goal_state(configuration_name="extended")

        pose_goal   = Pose()
        pose_goal.position.x,\
        pose_goal.position.y,\
        pose_goal.position.z,\
        pose_goal.orientation.w = goal
        
        self.current_goal_state = pose_goal

    def check_collisions(
                            self,
                            joint_model_group_name="panda_arm",
                        ):
        with self.planning_scene_monitor.read_only() as scene:
            robot_state                 = scene.current_state
            original_joint_positions    = robot_state\
                                            .get_joint_group_positions(joint_model_group_name)
            robot_state.set_from_ik(
                                    joint_model_group_name, 
                                    self.current_goal_state, 
                                    "panda_hand"
                                    )
            robot_state.update()  # required to update transforms
            robot_collision_status = scene.is_state_colliding(
                robot_state=robot_state, 
                joint_model_group_name=joint_model_group_name, 
                verbose=True,
            )
            self.logger.info(f"\nRobot is in collision: {robot_collision_status}\n")

        
def main():
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger                  = get_logger("moveit_py_planning_scene")
    moveit_controller       = MoveItPy(node_name="moveit_py_planning_scene")
    arm                     = moveit_controller.get_planning_component("panda_arm")
    planning_scene_monitor  = moveit_controller.get_planning_scene_monitor()


    world_scenario          = WorldHandler(planning_scene_monitor)
    planner                 = Planner(moveit_controller, planning_scene_monitor, arm, logger)

    world_scenario.set_scenario()  
    planner.plan_and_execute(use_collisions_ik=True)


if __name__ == "__main__":
    main()



# #!/usr/bin/env python3
# """
# Shows how to use a planning scene in MoveItPy to add collision objects and perform collision checking.
# """

# import time
# import rclpy
# from rclpy.logging import get_logger

# from moveit.planning import MoveItPy

# from geometry_msgs.msg import Pose
# from moveit_msgs.msg import CollisionObject
# from shape_msgs.msg import SolidPrimitive


# def plan_and_execute(
#     robot,
#     planning_component,
#     logger,
#     sleep_time=0.0,
# ):
#     """Helper function to plan and execute a motion."""
#     # plan to goal
#     logger.info("Planning trajectory")
#     plan_result = planning_component.plan()

#     # execute the plan
#     if plan_result:
#         logger.info("Executing plan")
#         robot_trajectory = plan_result.trajectory
#         robot.execute(robot_trajectory, controllers=[])
#     else:
#         logger.error("Planning failed")

#     time.sleep(sleep_time)


# def add_collision_objects(planning_scene_monitor):
#     """Helper function that adds collision objects to the planning scene."""
#     object_positions = [
#         (0.15, 0.1, 0.5),
#         (0.25, 0.0, 1.0),
#         (-0.25, -0.3, 0.8),
#         (0.25, 0.3, 0.75),
#     ]
#     object_dimensions = [
#         (0.1, 0.4, 0.1),
#         (0.1, 0.4, 0.1),
#         (0.2, 0.2, 0.2),
#         (0.15, 0.15, 0.15),
#     ]

#     with planning_scene_monitor.read_write() as scene:
#         collision_object = CollisionObject()
#         collision_object.header.frame_id = "panda_link0"
#         collision_object.id = "boxes"

#         for position, dimensions in zip(object_positions, object_dimensions):
#             box_pose = Pose()
#             box_pose.position.x = position[0]
#             box_pose.position.y = position[1]
#             box_pose.position.z = position[2]

#             box = SolidPrimitive()
#             box.type = SolidPrimitive.BOX
#             box.dimensions = dimensions

#             collision_object.primitives.append(box)
#             collision_object.primitive_poses.append(box_pose)
#             collision_object.operation = CollisionObject.ADD

#         scene.apply_collision_object(collision_object)
#         scene.current_state.update()  # Important to ensure the scene is updated


# def main():
#     ###################################################################
#     # MoveItPy Setup
#     ###################################################################
#     rclpy.init()
#     logger = get_logger("moveit_py_planning_scene")

#     # instantiate MoveItPy instance and get planning component
#     panda = MoveItPy(node_name="moveit_py_planning_scene")
#     panda_arm = panda.get_planning_component("panda_arm")
#     planning_scene_monitor = panda.get_planning_scene_monitor()
#     logger.info("MoveItPy instance created")

#     ###################################################################
#     # Plan with collision objects
#     ###################################################################

#     add_collision_objects(planning_scene_monitor)
#     panda_arm.set_start_state(configuration_name="ready")
#     panda_arm.set_goal_state(configuration_name="extended")
#     plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

#     ###################################################################
#     # Check collisions
#     ###################################################################
#     with planning_scene_monitor.read_only() as scene:
#         robot_state = scene.current_state
#         original_joint_positions = robot_state.get_joint_group_positions("panda_arm")

#         # Set the pose goal
#         pose_goal = Pose()
#         pose_goal.position.x = 0.25
#         pose_goal.position.y = 0.25
#         pose_goal.position.z = 0.5
#         pose_goal.orientation.w = 1.0

#         # Set the robot state and check collisions
#         robot_state.set_from_ik("panda_arm", pose_goal, "panda_hand")
#         robot_state.update()  # required to update transforms
#         robot_collision_status = scene.is_state_colliding(
#             robot_state=robot_state, joint_model_group_name="panda_arm", verbose=True
#         )
#         logger.info(f"\nRobot is in collision: {robot_collision_status}\n")

#         # Restore the original state
#         robot_state.set_joint_group_positions(
#             "panda_arm",
#             original_joint_positions,
#         )
#         robot_state.update()  # required to update transforms

#     time.sleep(3.0)

#     ###################################################################
#     # Remove collision objects and return to the ready pose
#     ###################################################################

#     with planning_scene_monitor.read_write() as scene:
#         scene.remove_all_collision_objects()
#         scene.current_state.update()

#     panda_arm.set_start_state_to_current_state()
#     panda_arm.set_goal_state(configuration_name="ready")
#     plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)


# if __name__ == "__main__":
#     main()