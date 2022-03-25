import os
import numpy as np
import matplotlib.pyplot as plt
from robotics.estimation import KalmanFilter

class Vehicle2DEstimation:
	def __init__(self) -> None:
		delta_t = 1
		newton_acc = 0.5*delta_t*delta_t
		# no control inputs
		# fmt: off
		F = np.array([
			[1 , delta_t , newton_acc , 0 , 0       , 0]          ,
			[0 , 1       , delta_t    , 0 , 0       , 0]          ,
			[0 , 0       , 1          , 0 , 0       , 0]          ,
			[0 , 0       , 0          , 1 , delta_t , newton_acc] ,
			[0 , 0       , 0          , 0 , 1       , delta_t]    ,
			[0 , 0       , 0          , 0 , 0       , 1]
		])
		print(F)
		
		q_directions = np.array(
			[
				[delta_t**4/4 , delta_t**3/2 , delta_t**2/2] ,
				[delta_t**3/2 , delta_t**2   , delta_t]      ,
				[delta_t**2/2 , delta_t      , 1]
			]
		)
		# fmt: on

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
		self.kalman = KalmanFilter(init_state, init_cov, F, Q)
		self.kalman.state, self.kalman.covariance = self.kalman.predict()
		print("Initialized filter: \n", self.kalman.state, "\n", self.kalman.covariance)
		cur_path = os.path.dirname(os.path.abspath(__file__))
		data = np.load(cur_path + "/data/linear_kalman_vehicle_data.npz")
		self.xs = data['x']
		self.ys = data['y']

if __name__ == "__main__":
	vehicle = Vehicle2DEstimation()
	vehicle.kalman.update(
		np.array([-393.66, 300.4]).reshape(-1, 1),
		vehicle.H,
		vehicle.R
	)
	states = [vehicle.kalman.state.flatten()]
	vehicle.kalman.predict()

	# perform simulation over the rest of the measurements
	for x in range(1, len(vehicle.xs)):
		point = np.array([vehicle.xs[x], vehicle.ys[x]]).reshape(-1, 1)
		vehicle.kalman.update(point, vehicle.H, vehicle.R)
		# update the state
		cur_state = vehicle.kalman.state
		states.append(cur_state.flatten())

		# prediction step
		vehicle.kalman.predict()
	
	states = np.array(states)
	print("Final Covariance\n", vehicle.kalman.covariance)
	plt.plot(vehicle.xs, vehicle.ys, 'b', label="Measurements")
	plt.plot(states[:, 0], states[:, 3], 'r', label="Filtered")
	plt.legend()
	plt.show()
