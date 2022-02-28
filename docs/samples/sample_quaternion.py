import robotics
import numpy as np

if __name__ == '__main__':
	q = robotics.Quaternion(0.9659258, 0, 0, 0.258819)
	v = np.array([1.0, 0, 0.0])
	q2 = robotics.Quaternion(v)
	t = q*v
	print(f"Data of the quaternion [w, x, y, z]: {q.data()}")
	print(f"Initialize quaternion with array: {q2}")
	print(q.adj())
	print(f"Rotation of vector {v} equals: {t}")

	q.plot()