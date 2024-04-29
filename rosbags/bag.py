ROS_BAG_PATH = 'rosbag2_2024_04_24-23_08_22'

import os
os.chdir(os.path.join(os.getcwd(),ROS_BAG_PATH))


from utils.parser import DataParser

parser = DataParser()
raw_data = parser.run()


from utils.analyzer import Analyzer

analyzer = Analyzer()
analyzer.analyze(raw_data)


from utils.plotter import make_df_fig, make_series_fig, Plotter

plotter = Plotter(raw_data)



PRINT_HTML 	= True
PRINT_IMG	= True


plotter.plot_joint_position_and_motion_sequence_states(print_html=PRINT_HTML, print_img=PRINT_IMG)

plotter.create_2d_subplots_of_windowed_data(
												analyzer.joint_position_windows,
												subplot_titles_prefix='motion sequence ',
												title='Joint Positions',
                                                x_title='Time [s]',
                                                y_title='Position [rad]',    
                                                print_html=PRINT_HTML, 
                                                print_img=PRINT_IMG,
                                            )

plotter.create_2d_subplots_of_windowed_data(
												analyzer.joint_velocity_windows,
												subplot_titles_prefix='motion sequence ',
												title='Joint Velocities',
                                                x_title='Time [s]',
                                                y_title='Velocity [rad/s]',  
                                                print_html=PRINT_HTML, 
                                                print_img=PRINT_IMG,  
                                            )


plotter.create_3d_suplots_of_windowed_data(    
												analyzer.ee_fk_positions,
												subplot_titles_prefix='motion sequence ',
												title='End Effector Positions (Forward Kinematics)',
												x_title='Time [s]',
												y_title='Position [m]',    
                                                print_html=PRINT_HTML, 
                                                print_img=PRINT_IMG,
										)

# average joint velocity
make_df_fig(
				analyzer.avg_joint_velocities, 
				return_trace_list=False, 
				show_fig=True, 
				name_prefix='joint: ', 
				title='Avg Joint Speed',
				x_title='motion sequence id [n]',
				y_title='avg joint speed [rad/s]',	
				print_html=PRINT_HTML, 
				print_img=PRINT_IMG,
			)