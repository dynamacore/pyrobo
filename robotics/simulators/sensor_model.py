import abc
import numpy as np
import matplotlib.pyplot as plt

# sensor object to inherit for different sensors
class Sensor(abc.ABC):
    def __init__(self, name, mean, cov, state):
        self.name_ = name
        # add the sensor noise characteristics
        self.mean_ = mean
        self.cov_ = cov
        self.H = None
        self.stateSize_ = state.shape[0]

    # simulate an update step
    def measure(self, state):
        # h(x) is the measurement model based off of the state
        measurement = self.model(state) + np.random.normal(self.mean_, self.cov_) 
        return measurement

    # the sensor model function producing prediction h(x)
    def model(self, state):
        # return the x value of the current state
        prediction = np.zeros_like(state)
        if self.H is not None:
            # perform the prediction based on the state
            prediction = self.H @ state
        else:
            # alert user if H is undefined
            print("Sensor jacobian H is not defined for {0}".format(self.name_))
        return prediction

    def innovation(self, predict, measurement):
        return measurement - predict


    # plot a function with the current sensor noise model
    def testPlot(self, function=lambda x: 2*x, start=0, end=1, step=100, plot=True):
        # create an array of x values to graph
        x = np.linspace(start, end, step)
        signal = function(x)

        # corrupt the sensor
        mu = np.random.normal(self.mean_, self.cov_, size=x.shape)
        
        # plot the sensor readings
        if plot:
            plt.plot(x, signal, "r", label="signal")
            plt.plot(x, signal + mu, "b", label="noise")
            plt.title("Noise Sample for Sensor {0}".format(self.name_))
            plt.legend()
            plt.show()

        return signal, signal + mu

class OdometrySensor(Sensor):
    def __init__(self, name, mean, cov, state):
        super().__init__(name, mean, cov, state)
        # the odometer senses (indirectly) the movement in the x direction
        self.H = np.zeros(self.stateSize_) 
        self.H[0] = 1

class GyroSensor(Sensor):
    def __init__(self, name, mean, cov, state):
        super().__init__(name, mean, cov, state)
        # the gyro is detecting the yaw
        self.H = np.zeros(self.stateSize)
        self.H[2] = 1


if __name__ == '__main__':
	sense = OdometrySensor("odom", 0, 0.2, np.zeros(3))
	sense.testPlot()