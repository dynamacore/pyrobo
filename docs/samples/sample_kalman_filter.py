import robotics
import numpy as np
import matplotlib.pyplot as plt
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
		self.xs = [-393.66,-375.93,-351.04,-328.96,-299.35,-273.36,-245.89,-222.58,-198.03,-174.17,-146.32,-123.72,-103.47,-78.23,-52.63,-23.34,	25.96,	49.72,	76.94,	95.38,	119.83,	144.01,	161.84,	180.56,	201.42,	222.62,	239.4,	252.51,	266.26,	271.75,	277.4,	294.12,	301.23,291.8,	299.89]
		self.ys = [300.4,	301.78,	295.1,	305.19,	301.06,	302.05,	300,	303.57,	296.33,	297.65,	297.41,	299.61,	299.6,	302.39,	295.04,	300.09,	294.72,	298.61,	294.64,	284.88,	272.82,	264.93,	251.46,	241.27,	222.98,	203.73,	184.1,	166.12,	138.71,	119.71,	100.41,	79.76,	50.62,	32.99,	2.14]
	

if __name__ == "__main__":
	vehicle = Vehicle2DEstimation()
	update = vehicle.kalman.update(
		np.array([-393.66, 300.4]).reshape(-1, 1),
		vehicle.H,
		vehicle.R
	)
	print("Iteration 1: \n", update[0], "\n", update[1])
	vehicle.kalman.state = update[0]
	vehicle.kalman.P = update[1]
	states = [vehicle.kalman.state.flatten()]
	prediction = vehicle.kalman.predict()
	print(prediction[0], "\n", prediction[1])
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
	print("Total", states[:, 0])
	print("Total", states[:, 3])
	plt.plot(vehicle.xs, vehicle.ys, 'b', label="Measurements")
	plt.plot(states[:, 0], states[:, 3], 'r', label="Filtered")
	plt.legend()
	plt.show()
