from .transform import Transform
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class KinematicTree:
	""" 
	An ordered collection of transforms, most useful for creating a full robot
	"""
	def __init__(self):
		pass