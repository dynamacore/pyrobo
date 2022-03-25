from typing import Tuple
import numpy as np


class KalmanFilter:
    def __init__(
        self,
        initial_state: np.ndarray,
        initial_cov: np.ndarray,
        transition_matrix: np.ndarray,
        process_noise: np.ndarray,
        control_model: np.ndarray = None,
    ):

        self.transition_model = transition_matrix
        self.control_model = control_model

        self.state = initial_state.reshape(-1, 1)
        self.covariance = initial_cov

        self.process_noise_cov = process_noise

    def predict(self, control=None):
        """
        Perform the prediction step using member variables and update them
        """
        self.state, self.covariance = self.__predict(
            self.state,
            self.covariance,
            self.transition_model,
            self.process_noise_cov,
            self.control_model,
            control,
        )
        return self.state, self.covariance

    @staticmethod
    def __predict(
        initial_state: np.ndarray,
        initial_covariance: np.ndarray,
        transition_model: np.ndarray,
        process_noise_covariance: np.ndarray,
        control_model: np.ndarray = None,
        control: np.ndarray = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict the next state and covariance based on the previous state without modifying any member variables
        This is a static method, or a function without side effects
        """
        # assign the mathematical variables
        x_0 = initial_state
        P_0 = initial_covariance
        F = transition_model
        Q = process_noise_covariance
        G = control_model
        u = control
        # predict the state
        next_state = F @ x_0
        # if we have control inputs, pass these as well
        if u is not None:
            input_transition = G @ np.array(u).reshape(-1, 1)
            next_state += input_transition

        # predict the covariance
        next_cov = F @ P_0 @ F.T + Q
        return next_state, next_cov

    def update(self, measurement, measurement_model, measurement_noise):
        """
        Update the state using a new measurement, modifying member variables
        """
        self.state, self.covariance = self.__update(
            measurement,
            measurement_model,
            measurement_noise,
            self.state,
            self.covariance,
        )
        return self.state, self.covariance

    @staticmethod
    def __update(
        measurement: np.ndarray,
        measurement_model: np.ndarray,
        measurement_noise: np.ndarray,
        predicted_state: np.ndarray,
        predicted_cov: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update the state and covariance using a new measurement without updating any member variables
        This is a static method, or a function without side effects
        """
        # assign the mathematical symbols
        z = measurement
        H = measurement_model
        R = measurement_noise
        x_p = predicted_state
        P_p = predicted_cov
        # calculate the kalman gain
        K = P_p @ H.T @ np.linalg.inv(H @ P_p @ H.T + R)

        # update the state using the measurement
        updated_state = x_p + K @ (z - H @ x_p)

        # update the covariance using the kalman gain
        K_p = np.eye(x_p.shape[0]) - K @ H
        updated_cov = K_p @ P_p @ K_p.T + K @ R @ K.T
        return updated_state, updated_cov
