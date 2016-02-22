from numpy import pi
from gpkit import Variable, Model, units
from gpkit.tools import te_exp_minus1
"""
Changed varibles to get similar results to PenguinB aircraft
"""
class GasPoweredHALE(Model):
    def setup(self):
        constraints = []

        # Steady level flight relations
        CD = Variable('C_D', '-', 'Drag coefficient')
        CL = Variable('C_L', '-', 'Lift coefficient')
        P_shaft = Variable('P_{shaft}', 'W', 'Shaft power')
        T = Variable('Thrust','N','Cruise thrust')
        S = Variable('S', 'm^2', 'Wing reference area')
        V = Variable('V', 'm/s', 'Cruise velocity')
        W = Variable('W', 'N', 'Aircraft weight')

        eta_prop = Variable(r'\eta_{prop}',0.7,'-', 'Propulsive efficiency')
        rho = Variable(r'\rho', 'kg/m^3')

        constraints.extend([P_shaft >= V*W*CD/CL/eta_prop,   # eta*P = D*V
                            W == 0.5*rho*V**2*CL*S])

        # Aerodynamics model
        Cd0 = Variable('C_{d0}', 0.01, '-', "non-wing drag coefficient")
        CLmax = Variable('C_{L-max}', 1.3, '-', 'maximum lift coefficient')
        e = Variable('e', 0.9, '-', "spanwise efficiency")
        A = Variable('A', 13, '-', "aspect ratio")
        b = Variable('b', 'm', 'span')
        mu = Variable(r'\mu', 1.5e-5, 'N*s/m^2', "dynamic viscosity")
        Re = Variable("Re", '-', "Reynolds number")
        Cf = Variable("C_f", "-", "wing skin friction coefficient")
        Kwing = Variable("K_{wing}", 1.3, "-", "wing form factor")
        cl_16 = Variable("cl_{16}", 0.0001, "-", "profile stall coefficient")
        
        constraints.extend([CD >= Cd0 + 2*Cf*Kwing + CL**2/(pi*e*A) + cl_16*CL**16,
                            b**2 == S*A,
                            CL <= CLmax, 
                            Re == rho*V/mu*(S/A)**0.5,
                            Cf >= 0.074/Re**0.2])


        # Engine Weight Model
        W_eng = Variable('W_{eng}', 'N', 'Engine weight')
        W_engmin = Variable('W_{min}', 13, 'N', 'min engine weight')
        W_engmax = Variable('W_{max}', 1500, 'N', 'max engine weight')
        eta_t = Variable('\\eta_t', 0.5, '-', 'percent throttle')
        eng_cnst = Variable('eng_{cnst}', 0.0013, '-',
                            'engine constant based off of power weight model')
        constraints.extend([W_eng >= W_engmin,
                            W_eng <= W_engmax,
                            W_eng >= (P_shaft/eta_t)**1.1572*eng_cnst* units('N/watt^1.1572')])

        # Weight model
        W_airframe = Variable('W_{airframe}', 'N', 'Airframe weight')
        W_pay = Variable('W_{pay}', 5*9.81, 'N', 'Payload weight')
        W_fuel = Variable('W_{fuel}', 'N', 'Fuel weight')
        W_zfw = Variable('W_{zfw}', 'N', 'Zero fuel weight')
        W_avionics = Variable('W_{avionics}', 1*9.81, 'N', 'Avionics weight')
        wl = Variable('wl', 'N/ft^2', 'wing loading')
        
        f_airframe = Variable('f_{airframe}', 0.4, '-',
                              'Airframe weight fraction')
        g = Variable('g', 9.81, 'm/s^2', 'Gravitational acceleration')

        constraints.extend([W_airframe >= W*f_airframe,
                            wl == W/S,
                            W_zfw >= W_airframe + W_eng + W_pay + W_avionics,
                            W >= W_fuel + W_zfw])

        # Breguet Range
        z_bre = Variable("z_bre", "-", "breguet coefficient")
        h_fuel = Variable("h_{fuel}", 42e6, "J/kg", "heat of combustion")
        eta_0 = Variable("\\eta_0", 0.2, "-", "overall efficiency")
        BSFC = Variable('BSFC', 0.65, 'lbf/hr/hp', 'brake specific fuel consumption')
        t = Variable('t', 2, 'days', 'time on station')

        constraints.extend([z_bre >= V*t*BSFC*CD/CL/eta_prop,
                            W_fuel/W_zfw >= te_exp_minus1(z_bre, 3)])

        # Atmosphere model
        h = Variable("h", 1000, "ft", "Altitude")
        p_sl = Variable("p_{sl}", 101325, "Pa", "Pressure at sea level")
        T_sl = Variable("T_{sl}", 288.15, "K", "Temperature at sea level")
        L_atm = Variable("L_{atm}", 0.0065, "K/m", "Temperature lapse rate")
        T_atm = Variable("T_{atm}", "K", "air temperature")
        M_atm = Variable("M_{atm}", 0.0289644, "kg/mol",
                         "Molar mass of dry air")
        R_atm = Variable("R_{atm}", 8.31447, "J/mol/K")
        TH = (g*M_atm/R_atm/L_atm).value.magnitude  # dimensionless
        constraints.extend([h <= 20000*units.m,  # Model valid to top of troposphere
                            T_sl >= T_atm + L_atm*h,     # Temp decreases w/ altitude
                            rho <= p_sl*T_atm**(TH-1)*M_atm/R_atm/(T_sl**TH)])
            # http://en.wikipedia.org/wiki/Density_of_air#Altitude


        # wind speed model
        V_max = Variable('V_{max}', 22, 'm/s', 'max speed')
        constraints.extend([V >= V_max]),


        objective = W

        return objective, constraints

if __name__ == "__main__":
    M = GasPoweredHALE()
    M.solve()
