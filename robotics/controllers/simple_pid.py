import numpy as np


class PID:
    def __init__(self, prop, deriv, integral):
        # for the gains
        self.prop_gain = prop
        self.deriv_gain = deriv
        self.int_gain = integral

        # for the integral and derivative gains
        self.err_last = 0
        self.err_cum = 0

    def run(self, cur_value, des_value):
        # calculate error
        error = des_value - cur_value

        # assign member variables
        self.err_last = error
        self.err_cum = self.err_cum + error
        self.eDiff = error - self.err_last

        # return the pid

        # check if multidimensional
        if np.shape(self.prop_gain) == ():
            pTerm = self.prop_gain * error
            dTerm = self.deriv_gain * self.eDiff
            iTerm = self.int_gain * self.err_cum

        else:
            pTerm = self.prop_gain @ error
            dTerm = self.deriv_gain @ self.eDiff
            iTerm = self.int_gain @ self.err_cum

        return pTerm + dTerm + iTerm
