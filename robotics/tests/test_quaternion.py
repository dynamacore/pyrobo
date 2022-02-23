from robotics import Quaternion

def test_quaternion():
	q1 = Quaternion()

if __name__ == '__main__':
	q1 = Quaternion()
	q2 = Quaternion(1, 0, 0, 0)
	print(q1*2.0)
	print(2.0*q1)
	print(q2.inv())
	print(q1*q2)