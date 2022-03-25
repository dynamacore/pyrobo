import pytest
import robotics
import numpy as np
import matplotlib.pyplot as plt


def run_pid(pid, startTime, endTime, dt, currentValue, desiredValue):
    times = np.arange(startTime, endTime, dt)
    values = []
    values.append(currentValue)
    for i in range(len(times)):
        time = times[i]

        value = pid.run(currentValue, desiredValue)
        currentValue = currentValue + value
        values.append(currentValue)

    return values


def test_pid_scalar():
    # test the pid controller for 10 seconds with the following gains
    pid = robotics.controllers.PID(0.1, 0.1, 0)

    values = run_pid(pid, 0, 10, 0.1, 0, 1)

    assert round(sum(values), 4) == 91.0002


def test_pid_multidimensional():
    pid = robotics.controllers.PID(
        np.diag([0.1, 1.1, 1.1]), np.diag([0, 0, 0]), np.diag([0.1, 0.0, 0.0])
    )

    currentValue = np.array([0, 0.3, 8])
    desiredValue = np.array([1, 4, 1])
    values = run_pid(
        pid, 0, 10, 0.1, currentValue=currentValue, desiredValue=desiredValue
    )

    assert np.array_equal(
        np.round(np.sum(values, axis=0), 2), np.array([100.99, 400.64, 107.36])
    )


if __name__ == "__main__":
    # pid = robotics.controllers.PID(1.5, 0.1, 0)
    pid = robotics.controllers.PID(
        np.diag([0.1, 1.1, 1.1]), np.diag([0, 0, 0]), np.diag([0.1, 0.0, 0.0])
    )

    currentValue = np.array([0, 0.3, 8])
    desiredValue = np.array([1, 4, 1])
    values = run_pid(
        pid, 0, 10, 0.1, currentValue=currentValue, desiredValue=desiredValue
    )

    print(sum(values))
    plt.plot(values)
    plt.show()
