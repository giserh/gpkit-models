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
        W = Variable('W', 'lbf', 'Aircraft weight')

        # Propulsion metrics (for a 3 bladed propeller, activity factor 100, design CL = 0.5)
        AdvRatio = Variable('J_{advance}',1.7,'-','Advance ratio')
        CPower = Variable('C_{Power}',0.2,'-','Power coefficient')
        CThrust = Variable('C_{Thrust}',0.5,'-','Thrust coefficient')
        CTorque = Variable('C_{Torque}','-','Torque coefficient')
        nRot = Variable('n_{Rot}','1/s','Propeller rotation speed')
        D_Prop = Variable('D_{Prop}',0.6,'m','Propeller diameter')

        eta_prop = Variable(r'\eta_{prop}',0.9,'-', 'Propulsive efficiency')
        rho = Variable(r'\rho', 'kg/m^3')

        constraints.extend([P_shaft == V*W*CD/CL/eta_prop,   # eta*P = D*V
                            W == 0.5*rho*V**2*CL*S,
                            T == 0.5*rho*V**2*CD*S])

        # Aerodynamics model
        Cd0 = Variable('C_{d0}', 0.01, '-', "non-wing drag coefficient")
        CLmax = Variable('C_{L-max}', 1.5, '-', 'maximum lift coefficient')
        e = Variable('e', 0.9, '-', "spanwise efficiency")
        A = Variable('A', '-', "aspect ratio")
        b = Variable('b', 'ft', 'span')
        mu = Variable(r'\mu', 1.5e-5, 'N*s/m^2', "dynamic viscosity")
        Re = Variable("Re", '-', "Reynolds number")
        Cf = Variable("C_f", "-", "wing skin friction coefficient")
        Kwing = Variable("K_{wing}", 1.3, "-", "wing form factor")
        cl_16 = Variable("cl_{16}", 0.0001, "-", "profile stall coefficient")
        
        constraints.extend([CD >= Cd0 + 2*Cf*Kwing + CL**2/(pi*e*A) + cl_16*CL**16,
                            #T == CD*1/2*rho*V**2*S,
                            #T <= P_shaft*(CThrust/CPower)/(nRot*D_Prop),
                            #eta_prop == T*V/P_shaft,
                            #AdvRatio == V/(nRot*D_Prop),
                            #AdvRatio >= 1, AdvRatio <= 2.8,
                            #AdvRatio == 1.8/0.23*CPower + 0.23,
                            #CPower == P_shaft/(rho*nRot**3*D_Prop**5),
                            #CThrust == T/(rho*nRot**2*D_Prop**4),
                            #P_shaft >= 2*pi*nRot*(CTorque*rho*nRot**2*D_Prop**5),
                            eta_prop == 1/(2*pi)*(CThrust/CTorque)*AdvRatio,
                            b**2 == S*A,
                            CL <= CLmax, 
                            Re == rho*V/mu*(S/A)**0.5,
                            Cf >= 0.074/Re**0.2])

        #Structure Variables
        rho_skin = Variable('rho_{skin}',200, 'g/m^2', 'Wing Skin Density')
        W_cent = Variable('W_{cent}', 'lbf','Center aircraft weight')
        W_fix = Variable('W_{fix}',10,'lbf','fixed weight')
        W_fuel = Variable('W_{fuel}','lbf','fuel weight')
        W_fuse = Variable('W_{fuse}',20,'lbf','fuselage weight') #probably needs structural model/better estimate
        A_capcent = Variable('A_{capcent}','m**2','cap area at center')
        A_cap = Variable('A_{cap}','m**2','cap area') #currently assumes constant area
        V_cap = Variable('V_{cap}','m**3','cap volume')
        M_cap = Variable('M_{cap}','kg','cap mass')
        M_skin = Variable('M_{skin}','kg','skin mass')
        E_cap = Variable('E_cap', 2e7, 'psi','Youngs modulus cf')

        M = Variable('M', 'N*m','center bending moment')
        d_tip = Variable('d','ft','Tip deflection') #need to add constraint
        h_spar = Variable('h_{spar}','m','Spar height') 
        sig = Variable('sig',475e6,'Pa','Cap stress') #http://www.performance-composites.com/carbonfibre/mechanicalproperties_2.asp
        F = Variable('F','N','load on wings')
        S_l = Variable('S_l','Pa','Shear load') #need to add constraint
        N = Variable('N',5,'-','Load factor') #load rating for max number of g's
        P = Variable('P', 'N', 'cap load')
        c = Variable('c','m','wing chord') #assumes straight, untapered wing
        rho_cap = Variable('rho_cap',1.76, 'g/cm^3','density of cf')
        t_cap = Variable('t_cap',.028,'in','spar cap thickness') #arbitrarily placed based on available cf
        w_cap = Variable('w_cap','in','spar cap width')
        W = Variable('W', 'lbf', 'Aircraft weight')
        d_tip_max = Variable('d_max','ft','max wing tip deflection')
        W_wing = Variable('W_wing','lbf','Total wing structural weight')
       
        t_c = Variable('t_c',0.1,'-','thickness ratio') #find better number

        # Engine Weight Model
        W_eng = Variable('W_{eng}', 26.2, 'lbf', 'Engine weight')
        #W_engmin = Variable('W_{min}', 13, 'lbf', 'min engine weight')
        #W_engmax = Variable('W_{max}', 1500, 'lbf', 'max engine weight')
        #eta_t = Variable('\\eta_t', 0.5, '-', 'percent throttle')
        #eng_cnst = Variable('eng_{cnst}', 0.0013, '-',
        #                    'engine constant based off of power weight model')
        #constraints.extend([W_eng >= W_engmin,
        #                    W_eng <= W_engmax,
        #                    W_eng >= (P_shaft/eta_t)**1.1572*eng_cnst* units('lbf/watt^1.1572')])
        #W_eng_installed = Variable('W_{eng-installed}','lbf','Installed engine weight')
        
        #constraints.extend([W_eng >= W_engmin,
         #                   W_eng <= W_engmax,
                            #W_eng_installed >= 2.572*W_eng**0.922*units('lbf')**0.078])

        # Weight model
        W_fuel = Variable('W_{fuel}', 'lbf', 'Fuel Weight')
        W_zfw = Variable('W_{zfw}', 'lbf', 'Zero fuel weight')
        W_avionics = Variable('W_{avionics}', 2, 'lbf', 'Avionics weight')
        wl = Variable('wl', 'lbf/ft^2', 'wing loading')

        g = Variable('g', 9.81, 'm/s^2', 'Gravitational acceleration')

        constraints.extend([M_skin >= rho_skin*S*(1.977 + .52*t_c),
                            F == W_cent*N,
                            c == S/b,
                            M == b*F/8,
                            P >= M/h_spar,
                            A_capcent >= P/sig,
                            V_cap >= A_capcent*b/3,
                            M_cap == rho_cap*V_cap,
                            h_spar <= t_c*c,
                            W_cent >= W_fix + W_fuel + W_eng + W_fuse,
                            W_wing >= M_skin*g+M_cap*g,
                            W >= W_cent + W_wing*1.2, #1.2 factor to account for tail weight
                            W_zfw >= W_fix +W_eng+W_fuse+M_skin*g + M_cap*g,
                            w_cap == A_capcent/t_cap,
                            wl ==W/S,
                            d_tip == b**2*sig/(4*E_cap*h_spar),
                            d_tip_max == b/8, # tip deflection less than 25% of half-span
                            d_tip <= d_tip_max])

        # constraints.extend([W_airframe >= W*f_airframe,
        #                     W_zfw >= W_airframe + W_eng + W_pay + W_avionics,
        #                     W >= W_fuel + W_zfw])

        # Breguet Range
        z_bre = Variable("z_bre", "-", "breguet coefficient")
        h_fuel = Variable("h_{fuel}", 42e6*0.4535, "J/lbf", "heat of combustion")
        eta_0 = Variable('eta_0', "-", "overall efficiency")
        BSFC = Variable('BSFC', 0.527, 'lbf/hr/hp', 'brake specific fuel consumption')
        t = Variable('t', 6, 'days', 'time on station')
        Wdot_fuel = Variable('mdot_{fuel}','lbf/hr','Fuel flow rate')

        constraints.extend([z_bre >= V*t*BSFC*CD/CL/eta_prop,
                            W_fuel/W_zfw >= te_exp_minus1(z_bre, 3),
                            Wdot_fuel == BSFC*P_shaft,
                            eta_0 == (T*V)/(h_fuel*Wdot_fuel),
                            ])

        # Atmosphere model
        h = Variable("h", "ft", "Altitude")
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

        # station keeping requirement
        footprint = Variable("d_{footprint}", 100, 'km',
                             "station keeping footprint diameter")
        lu = Variable(r"\theta_{look-up}", 5, '-', "look up angle")
        R_earth = Variable("R_{earth}", 6371, "km", "Radius of earth")
        tan_lu = lu*pi/180. + (lu*pi/180.)**3/3.  # Taylor series expansion
        # approximate earth curvature penalty as distance^2/(2*Re)
        constraints.extend([
            h >= tan_lu*0.5*footprint + footprint**2/8./R_earth])

        # wind speed model
        V_wind = Variable('V_{wind}', 'm/s', 'wind speed')
        wd_cnst = Variable('wd_{cnst}', 0.0015, 'm/s/ft', 
                           'wind speed constant predited by model')
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

        # fuel volume model
        #rho_fuel = Variable('\\rho_{fuel}', 719, 'kg/m^3', 'density of gasoline')
        #constraints.extend([S**1.5*A**-0.5*t_c >= W_fuel/g/rho_fuel])

        # Climb model
        # rho_sl = Variable('\\rho_{sl}',1.225,'kg/m**3','Density at sea level')
        # P_shaft_lapse = Variable('P_{shaft-lapse}',0.714/40000,'1/ft','Shaft power lapse rate')
        # P_shaft_to = Variable('P_{shaft-to}','hp','Shaft power on takeoff')

        # constraints.extend([
        # 	1 <= P_shaft_to/P_shaft + P_shaft_lapse*h,
        # 	P_shaft_to >= P_shaft
        # 	])

        objective = W

        return objective, constraints

if __name__ == "__main__":
    M = GasPoweredHALE()
    M.solve()
