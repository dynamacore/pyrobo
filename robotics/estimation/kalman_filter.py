import numpy as np

class KalmanFilter:
	def __init__(
			self,
			initial_state: np.ndarray,
			initial_cov: np.ndarray,
			state_transition: np.ndarray,
			process_noise: np.ndarray,
			sigma_a: float,
			delta_t: float,
			input_transition: np.ndarray = None,
		):

		self.F = state_transition
		# control input matrix
		self.G = input_transition

		self.state = initial_state.reshape(-1, 1)
		self.P = initial_cov

		self.Q = process_noise
		self.R = np.eye(self.state.shape[0])
	
	def predict(self, input=None):
		# predict the state
		next_state = self.F @ self.state
		if input is not None:
			# add in the control inputs as well
			input_transition = self.G @ np.array(input).reshape(-1, 1)
			next_state += input_transition
		
		# predict the covariance
		next_cov = self.F @ self.P @ self.F.T + self.Q
		return next_state, next_cov

	def update(self, measurement, measurement_model, measurement_noise):
		H = measurement_model
		R = measurement_noise
		kalman_gain = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + R)

		update = kalman_gain @ (measurement - H @ self.state)
		updated_state = self.state + update
		kalman_factor = (np.eye(self.state.shape[0]) - kalman_gain@H)
		updated_cov = kalman_factor @ self.P @ kalman_factor.T + kalman_gain @ R @ kalman_gain.T
		return updated_state, updated_cov