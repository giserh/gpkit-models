from numpy import pi
from gpkit import Variable, Model, units
from gpkit.tools import te_exp_minus1
import gpkit
import numpy as np

CL = Variable("CL")
CD = Variable("CD")
V = Variable("V", "m/s", "cruise speed")
W = Variable("W", 200, "lbf", "weight")
rho = Variable(r"\rho", 1.2, "kg/m^3", "air density")
S = Variable("S", 190, "ft^2", "wing area")
m = Model(V*(W/(CL/CD)),
          [0.5*rho*CL*S*V**2 == W,
           CL**1.5/CD <= 20])
sol = m.solve()