from numpy import pi
from gpkit import VectorVariable, Variable, Model, units
from gpkit.tools import te_exp_minus1
import gpkit
import numpy as np
gpkit.settings['latex_modelname'] = False

class GasPoweredHALE(Model):
    def setup(self):
        constraints = []
        CD = Variable('C_D',1, '-', 'Drag coefficient')
    	CL = Variable('C_L',1, '-', 'Lift coefficient')

        W = VectorVariable(2,'W','lbf', 'Aircraft weight')
        V = VectorVariable(2,'V',[20, 15], 'm/s','cruise speed')
        rho = Variable(r"\rho", 1.2, "kg/m^3", "air density")
        S = Variable("S", 190, "ft^2", "wing area")

        constraints.extend([#0.5*rho*CL*S*np.power(V,2) == W,
        	0.5*rho*CL*S*V[1]**2 == W[1],
        	0.5*rho*CL*S*V[0]**2 == W[0],
        	W >= 5*units('lbf'),
            CL**1.5/CD <= 20])

        objective = W
        return objective, constraints

if __name__ == "__main__":
	M = GasPoweredHALE()
	M.solve()