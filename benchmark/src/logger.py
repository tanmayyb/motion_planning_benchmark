#!/usr/bin/env python3
import  rclpy
from    rclpy.node import Node
from    sensor_msgs.msg import JointState
import  time
import  pandas as pd
import  numpy as np
import  os, json

class JointStateLogger(Node):

    def __init__(
                    self,
                    ROOT_PATH   : str,
                    ):
        super().__init__('joint_state_logger')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.logger         = self.get_logger()
        self.OUTPUT_DIRPATH = os.path.join(ROOT_PATH, "benchmark", ".out")
        self.buffer         = dict(
                                    position=list(), 
                                    velocity=list(),
                                    )
        self.BUFFER_TIMEOUT = 60.0
        self.start_time     = time.time()


    def joint_state_callback(self, msg):
        if (time.time() - self.start_time) < self.BUFFER_TIMEOUT:
            self.buffer['position'].append(json.dumps(msg.position.tolist()))
            self.buffer['velocity'].append(json.dumps(msg.velocity.tolist()))
            

    def save_buffer(self): 
        OUTPUT_DIRPATH  = self.OUTPUT_DIRPATH
        os.makedirs(OUTPUT_DIRPATH, exist_ok=True)
        filepath        = os.path.join(OUTPUT_DIRPATH,"datalog.csv") 
        df              = pd.DataFrame(self.buffer)
        df.to_csv(filepath,index=False)
        self.logger.info("Data saved")

def main(args=None):
    rclpy.init(args=args)
    joint_state_logger = JointStateLogger(os.getcwd())
    try:
        rclpy.spin(joint_state_logger)
    except KeyboardInterrupt:
        joint_state_logger.logger.info("Saving buffer data and exiting!")       
        joint_state_logger.save_buffer()
    joint_state_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()