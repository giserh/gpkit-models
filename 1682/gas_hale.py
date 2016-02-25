from numpy import pi
from gpkit import Variable, Model, units
from gpkit.tools import te_exp_minus1

class GasPoweredHALE(Model):
    def setup(self):
        constraints = []

        # Steady level flight relations
        CD = Variable('C_D', '-', 'Drag coefficient')
        CL = Variable('C_L', '-', 'Lift coefficient')
        P_shaft = Variable('P_{shaft}', 'hp', 'Shaft power')
        T = Variable('Thrust','lbf','Cruise thrust')
        S = Variable('S', 'm^2', 'Wing reference area')
        V = Variable('V', 'm/s', 'Cruise velocity')
        V_stall = Variable('V_{stall}','m/s','Stall speed at sea level')
        W = Variable('W', 'lbf', 'Aircraft weight')
        rho = Variable(r'\rho', 'kg/m^3')

        # Propulsion metrics (for a 3 bladed propeller, activity factor 100, design CL = 0.5)

        constraints.extend([#P_shaft == V*W*CD/CL/eta_prop,   # eta*P = D*V
                            W == 0.5*rho*V**2*CL*S,
                            T >= 0.5*rho*V**2*CD*S])

        # Aerodynamics model
        Cd0 = Variable('C_{d0}', 0.02, '-', "non-wing drag coefficient")
        CLmax = Variable('C_{L-max}', 1.5, '-', 'maximum lift coefficient')
        e = Variable('e', 0.9, '-', "spanwise efficiency")
        AR = Variable('AR', '-', "aspect ratio")
        b = Variable('b', 'ft', 'span')
        mu = Variable(r'\mu', 1.5e-5, 'N*s/m^2', "dynamic viscosity")
        Re = Variable("Re", '-', "Reynolds number")
        Cf = Variable("C_f", "-", "wing skin friction coefficient")
        Kwing = Variable("K_{wing}", 1.3, "-", "wing form factor")
        cl_16 = Variable("cl_{16}", 0.0001, "-", "profile stall coefficient")
        
        constraints.extend([CD >= Cd0 + 2*Cf*Kwing + CL**2/(pi*e*AR) + cl_16*CL**16,
                            b**2 == S*AR,
                            CL <= CLmax, 
                            Re == rho*V/mu*(S/AR)**0.5,
                            Cf >= 0.074/Re**0.2])

        # Propellor Model
        JAdvance = Variable('J_{advance}',2,'-','Advance ratio')
        CPower = Variable('C_{Power}',0.25,'-','Power coefficient')
        CThrust = Variable('C_{Thrust}',0.5,'-','Thrust coefficient')
        CTorque = Variable('C_{Torque}','-','Torque coefficient')
        RPM = Variable('RPM','1/min','Propeller rotation speed')
        D_Prop = Variable('D_{Prop}',2,'ft','Propeller diameter')

        eta_prop = Variable(r'\eta_{prop}',0.85,'-', 'Propulsive efficiency')
        MTip = Variable('MTip','-','Propeller tip Mach number')

        constraints.extend([T == P_shaft*(CThrust/CPower)/(RPM*D_Prop),
                            #eta_prop <= T*V/P_shaft,
                            #eta_prop == 1/(2*pi)*(CThrust/CTorque)*JAdvance,
                            #JAdvance == V/(RPM*D_Prop),
                            #JAdvance >= 1, JAdvance <= 2.8,
                            #JAdvance == 1.8/0.23*CPower + 0.23,
                            #CPower == P_shaft/(rho*RPM**3*D_Prop**5),
                            #CThrust == T/(rho*RPM**2*D_Prop**4),
                            #P_shaft >= 2*pi*RPM*(CTorque*rho*RPM**2*D_Prop**5)])
                            ])

        #Structure parameters
        g = Variable('g', 9.81, 'm/s^2', 'Gravitational acceleration')
        rho_skin = Variable('rho_{skin}',200, 'g/m^2', 'Wing Skin Density')
        rho_cap = Variable('rho_{cap}',1.76, 'g/cm^3','Density of CF cap')
        E_cap = Variable('E_{cap}', 2e7, 'psi','Youngs modulus of CF cap')

        #Weights and masses
        m_cap = Variable('m_{cap}','kg','Cap mass')
        m_skin = Variable('m_{skin}','kg','Skin mass')
        sigma_cap = Variable('sigma_capma_{cap}',475e6,'Pa','Cap stress') 
        #http://www.performance-composites.com/carbonfibre/mechanicalproperties_2.asp

        W_cent = Variable('W_{cent}', 'lbf','Center aircraft weight')
        W_payload = Variable('W_{fix}',10,'lbf','Payload weight')
        W_fuel = Variable('W_{fuel}','lbf','Fuel weight')
        W_zfw = Variable('W_{zfw}', 'lbf', 'Zero fuel weight')
        W_avionics = Variable('W_{avionics}', 2, 'lbf', 'Avionics weight')
        #W_tank = Variable('W_{tank}','lbf','Fuel tank weight')
        W_fuse = Variable('W_{fuse}',20,'lbf','Fuselage weight')
        W_wing = Variable('W_{wing}','lbf','Total wing structural weight')
        W = Variable('W', 'lbf', 'Aircraft weight') 

        #Structural ratios
        toverc = Variable('toverc',0.1,'-','Airfoil thickness ratio') #find better number
        LoverA = Variable('LoverA', 'lbf/ft^2', 'Wing loading')
        gamma_c = Variable('gamma_c','-','Taper ratio')

        #Structural lengths
        #Spar and caps are created assuming an I-beam desigma_capn
        h_spar = Variable('h_{spar}','m','Spar height') 
        tovercap = Variable('t_{cap}',.028,'in','Spar cap thickness') #arbitrarily placed based on available cf
        w_cap = Variable('w_{cap}','in','Spar cap width')
        c = Variable('c','ft','Wing chord') #assumes straight, untapered wing

        #Structural areas
        A_capcent = Variable('A_{capcent}','m**2','Cap area at center')
        A_cap = Variable('A_{cap}','m**2','Cap area') #currently assumes constant area
            
        #Structural volumes
        Vol_cap = Variable('Vol_{cap}','m**3','Cap volume')
        Vol_fuel = Variable('Vol_{fuel}','m**3','Fuel Volume')

        #Structural evaluation parameters
        Mom = Variable('Mom', 'N*m','Center bending moment')
        F = Variable('F','N','Load on wings')
        #S_l = Variable('S_{load}','Pa','Shear load') #need to add constraint
        LF = Variable('LF',5,'-','Load factor') #load rating for max number of g's
        L_cap = Variable('L_{cap}', 'N', 'Cap load')
        delta_tip = Variable('\delta_{tip}','ft','Tip deflection') #need to add constraint

        # Engine Model
        W_eng = Variable('W_{eng}', 6, 'lbf', 'Engine weight')
        P_shaft_max_sl = Variable('P_{shaft max sl}',5.4,'hp','Maximum power at sea level')
        P_shaft_max =  Variable('P_{shaft max}','hp','Maximum power at altitude')
        W_engmin = Variable('W_{min}', 13, 'lbf', 'min engine weight')
        W_engmax = Variable('W_{max}', 1500, 'lbf', 'max engine weight')
        eta_t = Variable('\\eta_t', 0.5, '-', 'percent throttle')
        eng_cnst = Variable('eng_{cnst}', 0.0013, '-',
                            'engine constant based off of power weight model')
        W_engtot = Variable('W_{eng-tot}','lbf','Installed engine weight')

        constraints.extend([W_eng >= (P_shaft/eta_t)**1.1572*eng_cnst* units('lbf/watt^1.1572'),
                            W_engtot >= 2.572*W_eng**0.922*units('lbf')**0.078])

	# Fuel Volume 
        rho_fuel = Variable('\\rho_{fuel}', 6.01, 'lbf/gallon', 'density of 100LL')
        constraints.extend([Vol_fuel == W_fuel/rho_fuel])

        # Structural constraints

        constraints.extend([m_skin >= rho_skin*S*(1.977 + .52*toverc),
                            F == W_cent*LF,
                            c == S/b,
                            Mom == b*F/8,
                            L_cap >= Mom/h_spar,
                            A_capcent >= L_cap/sigma_cap,
                            Vol_cap >= A_capcent*b/3,
                            m_cap == rho_cap*Vol_cap,
                            h_spar <= toverc*c,
                            W_cent >= W_payload + W_fuel + W_engtot + W_fuse + W_avionics,
                            W_wing >= m_skin*g+m_cap*g,
                            W >= W_cent + W_wing*1.3, #1.2 factor to account for tail weight
                            W_zfw >= W_payload +W_engtot+W_fuse+m_skin*g + m_cap*g + W_avionics,
                            w_cap == A_capcent/tovercap,
                            LoverA ==W/S,
                            delta_tip == b**2*sigma_cap/(4*E_cap*h_spar),
                            delta_tip <= b/8])

        # Breguet Range
        z_bre = Variable("z_{bre}", "-", "breguet coefficient")
        h_fuel = Variable("h_{fuel}", 42e6, "J/kg", "heat of combustion")
        eta_0 = Variable('eta_{0}', "-", "overall efficiency")
        BSFC = Variable('BSFC', 0.6, 'lbf/hr/hp', 'brake specific fuel consumption')
        #5.27 best case
        t = Variable('t', 6, 'days', 'time on station')
        mdot_fuel = Variable('mdot_{fuel}','kg/hr','Fuel flow rate')

        constraints.extend([z_bre >= V*t*BSFC*CD/CL/eta_prop,
                            W_fuel/W_zfw >= te_exp_minus1(z_bre, 3)])

        # Atmosphere model
        h = Variable("h","ft", "Altitude")
        gamma = Variable('\gamma',1.4,'-','Heat capacity ratio of air')
        p_sl = Variable("p_{sl}", 101325, "Pa", "Pressure at sea level")
        T_sl = Variable("T_{sl}", 288.15, "K", "Temperature at sea level")
        L_atm = Variable("L_{atm}", 0.0065, "K/m", "Temperature lapse rate")
        T_atm = Variable("T_{atm}", "K", "Air temperature")
        a_atm = Variable("a_{atm}",'m/s','Speed of sound at altitude')
        R_spec = Variable('R_{spec}',287.058,'J/kg/K','Specific gas constant of air')
        TH = (g/R_spec/L_atm).value.magnitude  # dimensionless

        constraints.extend([h <= 20000*units.m,  # Model valid to top of troposphere
                            T_sl >= T_atm + L_atm*h,     # Temp decreases w/ altitude
                            rho <= p_sl*T_atm**(TH-1)/R_spec/(T_sl**TH),
                            a_atm == (gamma*R_spec*T_atm)**0.5,
                            MTip == pi*D_Prop*RPM/a_atm,
                            MTip <= 0.85
                            ])
            # http://en.wikipedia.org/wiki/Density_of_air#Altitude

        # station keeping requirement
        d_footprint = Variable("d_{footprint}", 100, 'km',
                             "station keeping d_footprint diameter")
        lu = Variable(r"\theta_{look-up}", 5, '-', "look up angle")
        R_earth = Variable("R_{earth}", 6371, "km", "Radius of earth")
        tan_lu = lu*pi/180. + (lu*pi/180.)**3/3.  # Taylor series expansion
        # approximate earth curvature penalty as distance^2/(2*Re)
        constraints.extend([
            h >= tan_lu*0.5*d_footprint + d_footprint**2/8./R_earth])

        # wind speed model
        V_wind = Variable('V_{wind}','m/s', 'wind speed')
        wd_cnst = Variable('wd_{cnst}', 0.0015, 'm/s/ft', 
                           'wind speed constant predicted by model')
                            #0.002 is worst case, 0.0015 is mean at 45d
        wd_ln = Variable('wd_{ln}', 8.845, 'm/s',
                         'linear wind speed variable')
                        #13.009 is worst case, 8.845 is mean at 45deg
        h_min = Variable('h_{min}', 11800, 'ft', 'minimum height')
        h_max = Variable('h_{max}', 20866, 'ft', 'maximum height')
        constraints.extend([V_wind >= wd_cnst*h + wd_ln, 
                            V >= V_wind,
                            h >= h_min,
                            h <= h_max])
                            # model predicting worse case scenario at 45 deg latitude
        objective = W

        return objective, constraints

if __name__ == "__main__":
    M = GasPoweredHALE()
    M.solve()
