import os
import numpy as np
import matplotlib.pyplot as plt
from robotics.estimation import ExtendedKalmanFilter

class BicycleModelEstimation:
	def __init__(self, length):
		self.length = length
		# Kalman Filter variables
		initial_state = np.zeros(6)
		initial_cov = np.eye(6)*100
		process_noise = np.diag([1, 1, 0.2, 0.2, 0.1, 0.1])

		# set up the filter
		self.kalman = ExtendedKalmanFilter(
			initial_state=initial_state,
			initial_cov=initial_cov,
			process_noise=process_noise,
		)
	
	def motion_model(self, init_state: np.ndarray, control: list):
		''' 
		Motion model (nonlinear) for the bicycle model
		'''
		x_dot, delta_dot = control
		length = self.length
		theta = init_state[2]
		x_next = np.array(
			[
				[x_dot*np.cos(theta)]            ,
				[x_dot*np.sin(theta)]            ,
				[x_dot*np.tan(delta_dot)/length] ,
				[delta_dot]                      ,
				# since the velocity and steering velocity are inputs we just make this zero
				[0]                              ,
				[0]
			]
		)
		return x_next
	
	def model_matrix(self, state:np.ndarray):
		''' 
		Calculate the Jacobian of the transition function x_next = f(x)
		'''
		x, y, theta, delta, x_dot, delta_dot = state
		length = self.length
		transition_model = np.array(
			[
				[0, 0, -x_dot*np.sin(theta), 0, np.cos(theta),            0],
				[0, 0, x_dot*np.cos(theta),  0, np.sin(theta),            0],
				[0, 0, 0,                    0, np.tan(delta_dot)/length, (x_dot*np.arccos(theta)**2)/length],
				[0, 0, 0,                    0, 0,                        1],
				[0, 0, 0,                    0, 0,                        0],
				[0, 0, 0,                    0, 0,                        0],
			]
		)
		return transition_model
	
	def predict(self):
		# predict the next simulated step 
		self.kalman.predict(nonlinear_motion_model=self.motion_model)

if __name__ == "__main__":
	bike = BicycleModelEstimation(0.5)