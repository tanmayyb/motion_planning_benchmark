import  tf2_py as tf
import  math
from    sympy import Matrix, cos, sin
import numpy as np
import pandas as pd

import  tf2_py as tf
import  math
from    sympy import Matrix, cos, sin
import numpy as np
import pandas as pd

class ForwardKinematics():
# https://github.com/Pradn1l/Rospy-FK-7Axis-Robot/blob/main/direct_kinematics.py
	def dh_params(
					self,
					joint_variable  : list
				) -> list:

		joint_var = joint_variable
		M_PI = math.pi

		# Create DH parameters (data given by maker franka-emika)
		self.dh = [ [ 0,        0,        0.333,   joint_var[0]],
					[-M_PI/2,   0,        0,       joint_var[1]],
					[ M_PI/2,   0,        0.316,   joint_var[2]],
					[ M_PI/2,   0.0825,   0,       joint_var[3]],
					[-M_PI/2,  -0.0825,   0.384,   joint_var[4]],
					[ M_PI/2,   0,        0,       joint_var[5]],
					[ M_PI/2,   0.088,    0.107,   joint_var[6]]]
		
		return self.dh
	  
	def tf_matrix(
					self,
					i       : int,
					dh      : list,
				) -> Matrix:
		alpha = dh[i][0]
		a = dh[i][1]
		d = dh[i][2]
		q = dh[i][3]
		
		TF = Matrix([
					[cos(q),            -sin(q),            0,          a           ],
					[sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
					[sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
					[0,                 0,                  0,          1           ]])
		return TF

	def fk_using_joint_position(
								self,
								joint_positions : list
								) -> Matrix:
		dh_parameters = self.dh_params(joint_positions)

		T_01 = self.tf_matrix(0,dh_parameters)
		T_12 = self.tf_matrix(1,dh_parameters)
		T_23 = self.tf_matrix(2,dh_parameters)
		T_34 = self.tf_matrix(3,dh_parameters)
		T_45 = self.tf_matrix(4,dh_parameters)
		T_56 = self.tf_matrix(5,dh_parameters)
		T_67 = self.tf_matrix(6,dh_parameters)

		T_07 = T_01*T_12*T_23*T_34*T_45*T_56*T_67 

		# quaternions = tf.transformations.quaternion_from_matrix(T_07)
		# translations = tf.transformations.translation_from_matrix(T_07)

		return T_07


	def fk_using_joint_position2(
									self,
									joint_positions_series : pd.DataFrame
								) -> Matrix:

		joint_positions = joint_positions_series.to_list()

		return self.fk_using_joint_position(joint_positions)


class Analyzer():
	def __init__(self):
		self.fk = ForwardKinematics()


	def select_windows( self, 
						df      : pd.DataFrame, 
						signal  : pd.Series,                       
					) -> dict:
		# create df masks based on 
		# motion seq signal
		def pairwise(iterable):
			a = iter(iterable)
			return zip(a,a)        
		indices = list()
		for (p_t, _), (t, _) in pairwise(signal.items()):
			indices.append( (df.index>=p_t)&(df.index<=t) )
		print(f'n={len(indices)} motion sequences detected!')

		windows    = {i: df[index] for i, index in enumerate(indices)}
		return windows

	def get_joint_position( self,
							df      : pd.DataFrame
						) -> pd.DataFrame:
		index       = df.index.to_numpy()
		joint_data  = np.array(df.position.to_list())
		njoints     = joint_data.shape[1]

		data = {position: joint_data[:, position] for position in range(njoints)}

		return pd.DataFrame(data=data, index=index)


	def get_joint_velocity( self, 
							df		: pd.DataFrame
						) -> pd.DataFrame:
		return df.diff().iloc[1:]/df.index.diff()[1:].to_numpy().reshape(-1,1)
	

	def do_forward_kinematics(	self, 
						   		df	: pd.DataFrame
							) -> pd.DataFrame:
		index   = list()
		p1      = list()
		p2      = list()
		p3      = list()
		for i, (_, row) in enumerate(df.iterrows()):
			positions = np.array(self.fk.fk_using_joint_position2(row))[:,3][:-1]
			index.append(i)
			p1.append(float(positions[0]))
			p2.append(float(positions[1]))
			p3.append(float(positions[2]))
				  
		return pd.DataFrame({0:p1, 1:p2, 2:p3}, index=index)
				  

	def calculate_path_length(
								self,
								df		: pd.DataFrame,
							)	-> float:
		df		= df.diff().iloc[1:]
		dist 	= np.sqrt((df[0])**2+(df[1])**2+(df[2])**2)
		return dist.sum()

	def analyze(    
					self, 
					data	: dict,                    
				):
		
		# get windows for joint states df
		self.joint_states_windows = self.select_windows(    
										df=data['/joint_states']['df'],
										signal=data['/motion_sequence_state']['df'].signal,
									)

		# extract joint_positions from windows
		self.joint_position_windows = { index: self.get_joint_position(data) \
										for index, data in self.joint_states_windows.items()}

		# calculate joint_velocities from joint position windows
		self.joint_velocity_windows = { index: self.get_joint_velocity(data) \
										for index, data in self.joint_position_windows.items()}
		self.avg_joint_velocities	= {	index: df.mean() \
										for index, df in self.joint_velocity_windows.items()}
		self.avg_joint_velocities	= pd.DataFrame(self.avg_joint_velocities).T

		# do forward kinematics to get position of end effector 
		self.ee_fk_positions		= {	index: self.do_forward_kinematics(data)	\
						   				for index, data in self.joint_position_windows.items()}

		# calculate end effector path length
		self.ee_path_lengths 		= { index: self.calculate_path_length(data) \
									 		for index, data in self.ee_fk_positions.items()}
		self.ee_path_lengths 		= pd.Series(self.ee_path_lengths)
		self.ee_total_path_length	= self.ee_path_lengths.sum()

# analyzer = Analyzer()
# analyzer.analyze(raw_data)