""" d8fuselage.py """
from numpy import pi
import numpy as np
import matplotlib.pyplot as plt
from gpkit import VectorVariable, Variable, Model, units
from gpkit import LinkedConstraintSet
#from gpkit.tools import BoundedConstraintSet
from gpkit.tools import te_exp_minus1

class Fuselage(Model):
    def __init__(self, **kwargs):
    	constraints = []
    	# Will try to stick to Philippe's naming methods as closely as possible
    	# for cross-compatibility (to be able to switch models quickly)

    	# Fixed variables
    	SPR      = Variable('SPR', 8, '-', 'Number of seats per row')
    	npass    = Variable('n_{pass}', '-', 'Number of passengers')
    	nrows    = Variable('n_{rows}', '-', 'Number of rows')

    	# Cross sectional areas (free)

    	# Lengths (free)
    	lfuse    = Variable('l_{fuse}', 'm', 'Fuselage length')
        lnose    = Variable('l_{nose}', 'm', 'Nose length')
        lshell   = Variable('l_{shell}', 'm', 'Shell length')
    	Rfuse    = Variable('R_{fuse}', 'm', 'Fuselage radius')

    	# Lengths (fixed)

    	# Surface areas (free)
        Sbulk    = Variable('S_{bulk}', 'm^2', 'Bulkhead surface area')
        Sfloor   = Variable('S_{floor}', 'N', 'Maximum shear in floor beams')
        Snose    = Variable('S_{nose}', 'm^2', 'Nose surface area')

        # Volumes (free)
        Vcyl     = Variable('V_{cyl}', 'm^3', 'Cylinder skin volume')
        Vnose    = Variable('V_{nose}', 'm^3', 'Nose skin volume')
        Vbulk    = Variable('V_{bulk}', 'm^3', 'Bulkhead skin volume')
        Vcabin   = Variable('V_{cabin}', 'm^3', 'Cabin volume')


        # Weights (free)
        Wbuoy    = Variable('W_{buoy}', 'N', 'Buoyancy weight')
        Wapu     = Variable('W_{apu}', 'N', 'APU weight')

        # Weights (fixed)
        Wcargo   = Variable('W_{cargo}', 10000, 'N', 'Cargo weight')
        Wavgpass = Variable('W_{avg. pass}', 180, 'lbf', 'Average passenger weight')
        Wcarryon = Variable('W_{carry on}', 15, 'lbf', 'Ave. carry-on weight')
        Wchecked = Variable('W_{checked}', 40, 'lbf', 'Ave. checked bag weight')
    	Wfix     = Variable('W_{fix}', 3000, 'lbf',
                            'Fixed weights (pilots, cockpit seats, navcom)')

    	# Weight fractions (fixed)

    	ffadd    = Variable('f_{fadd}', '-',
                            'Fractional added weight of local reinforcements')
        fframe   = Variable('f_{frame}', '-', 'Fractional frame weight')

    	# Misc free variables
    	thetaDB = ('\\theta_{DB}','-','DB fuselage joining angle')



    	with SignomialsEnabled():
    		constraints = [

    		# Fuselage surface area relations
    		Snose >= (2*pi + 4*thetaDB)*Rfuse**2 *(1/3 + 2/3*(lnose/Rfuse)**(8/5))**(5/8),
    		Sbulk >= (2*pi + 4*thetaDB)*Rfuse**2,

    		# Fuselage volume relations
    		Vcyl == Askin*lshell,
            Vnose == Snose*tskin,
            Vbulk == Sbulk*tskin,

            # Fuselage weight relations
            Wskin >= rhoskin*g*(Vcyl + Vnose + Vbulk),
           	Wshell >= Wskin*(1 + fstring + fframe + ffadd)

    		]



    	Model.__init__(self, None, constraints, **kwargs)

# class Aircraft(Model):
#     """
#     Combined fuselage, tail, and landing gear model
#     """

#     def __init__(self):

#         # Free variables
#         W      = Variable('W', 'N', 'Total aircraft weight')
#         Wfuse  = Variable('W_{fuse}', 'N', 'Fuselage weight')
#         Wlg    = Variable('W_{lg}', 'N', 'Landing gear weight')
#         Wvt    = Variable('W_{vt}', 'N', 'Vertical tail weight')
#         xCG    = Variable('x_{CG}', 'm', 'x-location of CG')
#         xCGfu  = Variable('x_{CG_{fu}}', 'm', 'x-location of fuselage CG')
#         xCGlg  = Variable('x_{CG_{lg}}', 'm', 'x-location of landing gear CG')
#         xCGvt  = Variable('x_{CG_{vt}}', 'm', 'x-location of tail CG') 

#         # Fixed variables (pulled from Philippe's aircraft model)
#         Weng    = Variable('W_{eng}', 10000, 'N', 'Engine weight')
#         Wht     = Variable('W_{ht}', 5000, 'N', 'Horizontal tail weight')
#         Wwing   = Variable('W_{wing}', 30000, 'N', 'Wing weight')
#         xCGeng  = Variable('x_{CG_{eng}}', 15, 'm', 'x-location of engine CG')
#         xCGht   = Variable('x_{CG_{ht}}', 38, 'm', 'x-location of horizontal tail CG')
#         xCGwing = Variable('x_{CG_{wing}}', 15, 'm', 'x-location of wing CG')

#         fuselage = Fuselage()

#         self.submodels = [Fuselage]

#         constraints = [];

#         lc = LinkedConstraintSet([self.submodels, constraints],
#                                  include_only=INCLUDE)

#         objective = 1/W;

#         Model.__init__(self, objective, lc, **kwargs)

if __name__ == "__main__":
	M = Fuselage()
	sol = M.solve("mosek")