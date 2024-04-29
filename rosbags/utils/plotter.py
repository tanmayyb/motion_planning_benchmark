import pandas as pd
import numpy as np

import  plotly.graph_objects as go
import  plotly.express as px
from    plotly.subplots import make_subplots
import  plotly.io as pio
pio.templates.default = "plotly_white"

import 	os

def make_series_fig(
					series			: pd.Series, 
					name			: str='', 
					return_trace	: bool=False,
					marker_color	: str=None,
					title			: str='',
					x_title			: str='',
					y_title			: str='',
					show_fig		: bool=False, 
					print_html		: bool=False,
					print_img		: bool=False,
				):

	if marker_color is None:
		marker_color = '#1F77B4'

	trace = go.Scattergl(
							x=series.index, 
							y=series.values, 
							name=name,
							marker=dict(color=marker_color),
						)
	if show_fig:
		fig = go.Figure()
		fig.add_trace(trace)
		fig.update_layout(
							title=title,
							xaxis=dict(title=x_title),
							yaxis=dict(title=y_title),
						)
		fig.show()
		if print_html:
			fig.write_html(f'plots/htmls/{title}.html')
		if print_html:
			fig.write_image(f'plots/pngs/{title}.png', width=2000, height=500)

	if return_trace:
		return trace

def make_df_fig(
					df					: pd.DataFrame, 
					show_fig			: bool=False, 
					return_trace_list	: bool=False,
					name_prefix			: str='',
					name_suffix			: str='', 
					title				: str='',
					x_title				: str='',
					y_title				: str='',
					print_html			: bool=False,
					print_img			: bool=False,
				):
	
	colors = px.colors.qualitative.Plotly

	trace_list = list()
	for i, col in enumerate(df.columns):
		trace_list.append(
							make_series_fig(
								df[col], 
								return_trace=True, 
								name=f'{name_prefix}{col}{name_suffix}',
								marker_color=None if i>len(colors) else colors[i]
							)) 
	if show_fig:
		fig=go.Figure()
		for trace in trace_list:
			fig.add_trace(trace)
		fig.update_layout(
							title=title,
							xaxis=dict(title=x_title),
							yaxis=dict(title=y_title),
						)
		fig.show()
		if print_html:
			fig.write_html(f'plots/htmls/{title}.html')
		if print_html:
			fig.write_image(f'plots/pngs/{title}.png', width=2000, height=500)

	if return_trace_list:
		return trace_list
	

class Plotter():
	def __init__(   self, 
					data:dict,
				):
		self.raw_data = data
		# make directories
		os.makedirs(os.path.join(*['.', 'plots']), exist_ok=True)
		os.makedirs(os.path.join(*['.','plots', 'htmls']), exist_ok=True)
		os.makedirs(os.path.join(*['.','plots', 'pngs']), exist_ok=True)
		

	def plot_joint_position_and_motion_sequence_states(
														self,
														print_html	: bool=False,
														print_img	: bool=False,
													):
		
		def get_joint_states_traces(  
									df          : pd.DataFrame=None,
								) -> list:
			index       = df.index.to_numpy()
			joint_data  = np.array(df.position.to_list())
			njoints     = joint_data.shape[1]
			trace_list = list()
			for joint_id in range(njoints):
				trace_list.append(go.Scattergl(
												x = index,
												y = joint_data[:,joint_id],
												name = f'joint: {joint_id}',
										)
								)
			return trace_list

		def get_motion_seq_state_trace(  
										df	: pd.DataFrame,
									) -> list:
			index       = df.index.to_numpy()
			state_data  = np.array(df.signal.to_list())
			trace_list  = list()
			trace       = go.Scattergl(
										x = index,
										y = state_data,
										name = f'motion_seq_state',
										mode='lines+markers',
										line_shape = 'hv',
										marker=dict(size=3.5),
										line=dict(color='rgba(99, 110, 250, 0.5)', dash='dash'),
									)        
			trace_list.append(trace)
			return trace_list
			
		fig = make_subplots(rows=1, cols=1, shared_xaxes=True, specs=[  [dict(secondary_y=True)]  ]*1 )
		for trace in get_joint_states_traces(self.raw_data['/joint_states']['df']):
			fig.add_trace(trace, row=1, col=1, secondary_y=False)
		for trace in get_motion_seq_state_trace(self.raw_data['/motion_sequence_state']['df']):
			fig.add_trace(trace, row=1, col=1, secondary_y=True)
		fig.update_layout(
							title='Joint Positions + Motion Sequence',
							xaxis = dict(title='time [s]'),
							yaxis = dict(title='joint position [rad]/motion sequence state'),			    			
						)
		fig.show()

		if print_html:
			fig.write_html('plots/htmls/joint_position_and_motion_seq_state.html')
		if print_html:
			fig.write_image('plots/pngs/joint_position_and_motion_seq_state.png', width=2000, height=500)


	def create_2d_subplots_of_windowed_data(
												self,
												data	: dict,
												ncols	: int=3,
												nrows	: int=4,
												subplot_titles_prefix	: str='',
												title	: str='',
												x_title	: str='',
												y_title	: str='',
												print_html	: bool=False,
												print_img	: bool=False,
											):
		fig = make_subplots(
						rows=nrows, 
						cols=ncols,
						row_heights=[0.3]*nrows,
						column_widths=[0.3]*ncols,
						vertical_spacing=0.15, 
						horizontal_spacing=0.1,
						x_title=x_title,
						y_title=y_title,
						subplot_titles=[f'{subplot_titles_prefix} {i}' for i in data.keys()],
					)
		for i in range(1, len(data)+1):

			row = (i - 1) // ncols + 1
			col = (i - 1) % ncols + 1

			for trace in make_df_fig(data[i-1], return_trace_list=True, name_prefix='joint: '):
				fig.add_trace(trace, row=row, col=col)

		fig.update_layout(
					title=title, 
					showlegend=False, 
					height=1000, 
					width=1000,
					margin=dict(t=100)
				)
		
		fig.show()

		if print_html:
			fig.write_html(f'plots/htmls/{title}.html')
		if print_html:
			fig.write_image(f'plots/pngs/{title}.png', width=1000, height=1000)



	def create_3d_suplots_of_windowed_data(
											self,
											data	: dict,
											ncols	: int=3,
											nrows	: int=4,
											subplot_titles_prefix	: str='',
											title	: str='',
											x_title	: str='',
											y_title	: str='',
											print_html	: bool=False,
											print_img	: bool=False,
										):

		
		fig = make_subplots(
						rows=nrows, 
						cols=ncols,
						row_heights=[0.3]*nrows,
						column_widths=[0.3]*ncols,
						specs=[[{"type": "scatter3d"}]*ncols]*nrows,
						vertical_spacing=0.04, 
						horizontal_spacing=0.02,
						x_title=x_title,
						y_title=y_title,
						subplot_titles=[f'{subplot_titles_prefix} {i}' for i in data.keys()],
					)
		
		for i in range(1, len(data)+1):

			row = (i - 1) // ncols + 1
			col = (i - 1) % ncols + 1
			df = data[i-1]
			trace = go.Scatter3d(
						x=df[0],
						y=df[1],
						z=df[2],
						mode='lines+markers',
						marker=dict(
									size=4,
									color=df.index,
									colorscale='Viridis',
									opacity=0.8
								)
						)
			fig.add_trace(trace, row=row, col=col)

		fig.update_layout(
					title=title, 
					showlegend=False, 
					height=1000, 
					width=1000,
					margin=dict(t=100)
				)
		
		fig.show()

		if print_html:
			fig.write_html(f'plots/htmls/{title}.html')
		if print_html:
			fig.write_image(f'plots/pngs/{title}.png', width=1000, height=1000)



# plotter = Plotter(raw_data)



