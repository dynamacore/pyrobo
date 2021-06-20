import numpy as np

class PID:
	def __init__(self, prop, deriv, integral):
		# for the gains
		self.proportionalGain = prop
		self.derivativeGain = deriv
		self.integralGain = integral
		
		# for the integral and derivative gains
		self.errorLast = 0
		self.errorCum = 0

	def run(self, curValue, desValue):
		# calculate error
		error = desValue - curValue

		# assign member variables
		self.errorLast = error
		self.errorCum = self.errorCum + error
		self.eDiff = error - self.errorLast

		# return the pid

		# check if multidimensional
		if np.shape(self.proportionalGain) == ():
			pTerm = self.proportionalGain * error
			dTerm = self.derivativeGain * self.eDiff
			iTerm = self.integralGain * self.errorCum
		
		else:
			pTerm = self.proportionalGain @ error
			dTerm = self.derivativeGain @ self.eDiff
			iTerm = self.integralGain @ self.errorCum
			
		return pTerm + dTerm + iTerm


