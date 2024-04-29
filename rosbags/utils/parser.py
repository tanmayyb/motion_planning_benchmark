from    rosbags.rosbag2 import Reader
from    rosbags.typesys import Stores, get_types_from_msg, get_typestore

import json

import pandas as pd

class DataParser():
# https://answers.ros.org/question/358686/how-to-read-a-bag-file-in-ros2/
    def __init__(self):
        pass
    
    def load_typestore(self, LOAD_CUSTOM_TYPESTORE:bool=False):
        
        typestore = get_typestore(Stores.LATEST)

        if LOAD_CUSTOM_TYPESTORE:
            # https://github.com/ros-controls/control_msgs/tree/master/control_msgs/msg

            INTERFACE_VALUE_MSG = """
            string[] interface_names
            float64[] values
            """
            typestore.register(get_types_from_msg(INTERFACE_VALUE_MSG, 'control_msgs/msg/InterfaceValue'))

            DYN_JOINT_STATE_MSG = """
            std_msgs/Header header
            string[] joint_names
            InterfaceValue[] interface_values
            """
            typestore.register(get_types_from_msg(DYN_JOINT_STATE_MSG, 'control_msgs/msg/DynamicJointState'))

        return typestore

    def parse_rosbag(self) -> dict:

        def create_storage_dict():
            with Reader('.') as reader:
                rosbag_data  = {connection.topic: dict(
                                                        data=dict(), 
                                                        msg_type=connection.msgtype
                                                    ) for connection in  reader.connections }
            print(f"rosbag_created:\n{json.dumps(rosbag_data, indent=4)}")
            return rosbag_data
        rosbag_data = create_storage_dict()

        with Reader('.') as reader:
            typestore = self.load_typestore(LOAD_CUSTOM_TYPESTORE=True)
            for connection, timestamp, rawdata in reader.messages():
                try:
                    rosbag_data[connection.topic]['data'][timestamp] =  typestore.deserialize_cdr(
                                                                        rawdata, 
                                                                        connection.msgtype,
                                                        )
                except Exception as e:
                    # print(f'error with {connection.msgtype}: {e}')
                    # continue
                    pass
        print(f"parsed rosbag and stored in rosbag_data dict!\n")
        return rosbag_data

    def parse_as_dfs(self, data_dict:dict) -> dict:
        rosbag_data = dict()
        def parse_item_into_df(data:dict) -> pd.DataFrame:
            event_data_df = dict()

            for timestamp, event_item in data.items():
                event = dict()
                for name, value in vars(event_item).items():
                    if name=='__msgtype__':
                        continue
                    if name=='header':
                        event['header_secs']    = vars(value)['stamp'].sec
                        event['header_nano']    = vars(value)['stamp'].nanosec
                        event['header_time']    = float(
                                                        str(event['header_secs'])\
                                                        +'.'\
                                                        +str(event['header_nano']))
                        continue
                    event[name]                 = value
                event_data_df[timestamp]        = event 
            return pd.DataFrame(event_data_df).T

        for topic, item in data_dict.items():
            try:
                rosbag_data[topic]  = dict( 
                                            df=parse_item_into_df(item['data']),
                                            msg_type = item['msg_type'],
                                        )
            except Exception as e:
                print(f'Error parsing {topic}:\n{e}')
            else:
                if len(rosbag_data[topic]['df'])==0: print(f'topic {topic} has no events!')

        print(f'parsed rosbag data as dfs!')            

        return rosbag_data
    

    def do_data_prep(   self,
                        data:dict
                    ) -> dict:
        for k, d  in data.items():
            if k=='/motion_sequence_state':
                df = d['df']
                df['signal'] = df.data.apply(lambda x: 1.0 if x==True else 0.0)
                data[k]['df'] = df

            d['df'].index = d['df'].index/1000000000

            # df
        return data


    def run(self):
        return self.do_data_prep(self.parse_as_dfs(self.parse_rosbag()))
    

# parser = DataParser()
# raw_data = parser.run()
