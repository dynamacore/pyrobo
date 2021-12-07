import robotics
import numpy as np
import matplotlib.pyplot as plt
import os
from robotics.estimation.kalman_filter import KalmanFilter

class Object3DTracking:
	def run(self):
		# define the transition matrices
		delta_t = 0.01
		newton_acc = 0.5*delta_t*delta_t
		control_F = np.array([
				[1, 0, 0, delta_t, 0, 0],
				[0, 1, 0, 0, delta_t, 0],
				[0, 0, 1, 0, 0, delta_t],
				[0, 0, 0, 1, 0, 0],
				[0, 0, 0, 0, 1, 0],
				[0, 0, 0, 0, 0, 1]
			]
			)
		control_G = np.array([
				[newton_acc, 0, 0],
				[0, newton_acc, 0],
				[0, 0, newton_acc],
				[delta_t, 0, 0],
				[0, delta_t, 0],
				[0, 0, delta_t]
			]
		)
		state_init = np.zeros(6)
		cov_init = np.eye(6)
		kalman = robotics.KalmanFilter(state_init, cov_init, control_F, control_G, np.eye(6), 0.5, delta_t)
		print(kalman.predict([0.1, 0, 0]))

		# measure the x, y, z coordinates
		measurement = np.array([0.5, 0.2, 0]).reshape(-1, 1)
		H = np.array([
			[1, 0, 0, 0, 0, 0],
			[0, 1, 0, 0, 0, 0],
			[0, 0, 1, 0, 0, 0]
		])

		R = np.eye(3)

		print(kalman.update(measurement, H, R))

class Vehicle2DEstimation:
	def __init__(self) -> None:
		delta_t = 1
		newton_acc = 0.5*delta_t*delta_t
		# no control inputs
		F = np.array([
			[1, delta_t, newton_acc, 0, 0, 0],
			[0, 1, delta_t, 0, 0, 0],
			[0, 0, 1, 0, 0, 0],
			[0, 0, 0, 1, delta_t, newton_acc], 
			[0, 0, 0, 0, 1, delta_t],
			[0, 0, 0, 0, 0, 1]
		])
		print(F)
		
		q_directions = np.array(
			[
				[delta_t**4/4, delta_t**3/2, delta_t**2/2],
				[delta_t**3/2, delta_t**2,   delta_t],
				[delta_t**2/2, delta_t,      1]
			]
		)
		Q = np.zeros((6, 6))
		Q[0:3, 0:3] = q_directions
		Q[3:, 3:] = q_directions
		# sigma for the acceleration
		sigma_a = 0.15
		print(Q)
		Q *= sigma_a*sigma_a
		init_state = np.zeros(6)
		# high estimate uncertainty gives a high weight to the measurement
		init_cov = np.eye(6)*500
		# observation matrix
		self.H = np.array(
			[
				[1, 0, 0, 0, 0, 0],
				[0, 0, 0, 1, 0, 0]
			]
		)
		self.R = np.eye(2)*9
		self.kalman = KalmanFilter(init_state, init_cov, F, Q, sigma_a, delta_t)
		self.kalman.state, self.kalman.P = self.kalman.predict()
		print("Initialized filter: \n", self.kalman.state, "\n", self.kalman.P)
		cur_path = os.path.dirname(os.path.abspath(__file__))
		data = np.load(cur_path + "/linear_kalman_vehicle_data.npz")
		self.xs = data['x']
		self.ys = data['y']


	

if __name__ == "__main__":
	vehicle = Vehicle2DEstimation()
	update = vehicle.kalman.update(
		np.array([-393.66, 300.4]).reshape(-1, 1),
		vehicle.H,
		vehicle.R
	)
	vehicle.kalman.state = update[0]
	vehicle.kalman.P = update[1]
	states = [vehicle.kalman.state.flatten()]
	prediction = vehicle.kalman.predict()
	vehicle.kalman.state = prediction[0]
	vehicle.kalman.P = prediction[1]

	for x in range(1, len(vehicle.xs)):
		point = np.array([vehicle.xs[x], vehicle.ys[x]]).reshape(-1, 1)
		update = vehicle.kalman.update(point, vehicle.H, vehicle.R)
		# update the state
		vehicle.kalman.state = update[0]
		vehicle.kalman.P = update[1]
		cur_state = vehicle.kalman.state
		states.append(cur_state.flatten())

		# prediction step
		prediction = vehicle.kalman.predict()
		vehicle.kalman.state = prediction[0]
		vehicle.kalman.P = prediction[1]

	
	states = np.array(states)
	plt.plot(vehicle.xs, vehicle.ys, 'b', label="Measurements")
	plt.plot(states[:, 0], states[:, 3], 'r', label="Filtered")
	plt.legend()
	plt.show()
