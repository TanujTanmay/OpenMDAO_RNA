from math import pi, sqrt, sin, cos, tan, atan, log
import numpy as np
from openmdao.api import IndepVarComp, ExecComp, ExplicitComponent, Group, Problem, ScipyOptimizer, view_model
#from openmdao.drivers.pyoptsparse_driver import pyOptSparseDriver

from fixed_parameters import beautify_output, rho_Fes, rho_Fe, rho_Copper, rho_PM, C_Cu, C_Fe, C_Fes, C_PM, safety_factor



#############################################################################
##############################  PMSG Main Model #############################
#############################################################################        
class PMSG(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('r_s', desc = 'airgap radius r_s')
        self.add_input('l_s', desc = 'Stator core length l_s')
        self.add_input('h_s', desc = 'Yoke height h_s')
        self.add_input('tau_p', desc = 'Pole pitch self.tau_p')
        self.add_input('machine_rating', desc = 'Machine rating')
        self.add_input('n_nom', desc = 'rated speed')
        self.add_input('Torque', desc = 'Rated torque ')
        self.add_input('h_m', desc = 'magnet height')
        self.add_input('h_ys', desc = 'Yoke height')
        self.add_input('h_yr', desc = 'rotor yoke height')
        
        self.add_input('n_s', desc = 'number of stator arms n_s')
        self.add_input('b_st', desc = 'arm width b_r')
        self.add_input('d_s', desc = 'arm depth d_r')
        self.add_input('t_ws', desc = 'arm depth thickness self.t_wr')
        self.add_input('n_r', desc = 'number of arms n')
        self.add_input('b_r', desc = 'arm width b_r')
        self.add_input('d_r', desc = 'arm depth d_r')
        self.add_input('t_wr', desc = 'arm depth thickness self.t_wr')
        #self.add_input('t', desc = 'rotor back iron')
        #self.add_input('t_s', desc = 'stator back iron')
        self.add_input('R_o', desc = 'Shaft radius')
        
        self.add_input('main_shaft_cm', desc = 'Main Shaft CM', val=[0., 0., 0.])
        self.add_input('main_shaft_length', desc = 'main shaft length', val=0.)

        
        # outputs
        self.add_output('B_symax', desc = 'Peak Stator Yoke flux density B_ymax')
        self.add_output('B_tmax', desc = 'Peak Teeth flux density')
        self.add_output('B_rymax', desc = 'Peak Rotor yoke flux density')
        self.add_output('B_smax', desc = 'Peak Stator flux density')
        self.add_output('B_pm1', desc = 'Fundamental component of peak air gap flux density')
        self.add_output('B_g', desc = 'Peak air gap flux density B_g')
        
        self.add_output('N_s', desc = 'Number of turns in the stator winding')
        self.add_output('b_s', desc = 'slot width')
        self.add_output('b_t', desc = 'tooth width')
        self.add_output('A_Cuscalc', desc = 'Conductor cross-section mm^2')
        
        self.add_output('b_m', desc = 'magnet width')
        self.add_output('p', desc = 'No of pole pairs')
        
        self.add_output('E_p', desc = 'Stator phase voltage')
        self.add_output('f', desc = 'Generator output frequency')
        self.add_output('I_s', desc = 'Generator output phase current')
        self.add_output('R_s', desc = 'Stator resistance')
        self.add_output('L_s', desc = 'Stator synchronising inductance')
        self.add_output('A_1', desc = 'Electrical loading')
        self.add_output('J_s', desc = 'Current density')
        
        self.add_output('Mass', desc = 'Actual mass')
        self.add_output('K_rad', desc = 'K_rad')
        self.add_output('Losses', desc = 'Total loss')
        self.add_output('gen_eff', desc = 'Generator efficiency')
        
        self.add_output('u_Ar', desc = 'Rotor radial deflection')
        self.add_output('y_Ar', desc = 'Rotor axial deflection')
        self.add_output('z_A_r', desc = 'Rotor circumferential deflection')
        self.add_output('u_As', desc = 'Stator radial deflection')
        self.add_output('y_As', desc = 'Stator axial deflection')
        self.add_output('z_A_s', desc = 'Stator circumferential deflection')
        self.add_output('u_all_r', desc = 'Allowable radial rotor')
        self.add_output('u_all_s', desc = 'Allowable radial stator')
        self.add_output('y_all', desc = 'Allowable axial')
        self.add_output('z_all_s', desc = 'Allowable circum stator')
        self.add_output('z_all_r', desc = 'Allowable circum rotor')
        self.add_output('b_all_s', desc = 'Allowable arm')
        self.add_output('b_all_r', desc = 'Allowable arm dimensions')
        self.add_output('TC1', desc = 'Torque constraint')
        self.add_output('TC2', desc = 'Torque constraint-rotor')
        self.add_output('TC3', desc = 'Torque constraint-stator')
        
        self.add_output('R_out', desc = 'Outer radius')
        self.add_output('S', desc = 'Stator slots')
        self.add_output('Slot_aspect_ratio', desc = 'Slot aspect ratio')
        
        self.add_output('mass_PM', desc = 'Magnet mass')
        self.add_output('Copper', desc = 'Copper Mass')
        self.add_output('Iron', desc = 'Electrical Steel Mass')
        self.add_output('Structural_mass', desc = 'Structural Mass')
        self.add_output('cm', desc = 'COM [x,y,z]', shape=3)
        self.add_output('I', desc = 'Moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)
        self.add_output('Costs', desc = 'Total cost')

    def compute(self, inputs, outputs):
        # inputs    
        self.r_s = inputs['r_s']
        self.l_s = inputs['l_s']
        self.h_s = inputs['h_s']
        self.tau_p = inputs['tau_p']
        self.machine_rating = inputs['machine_rating']
        self.n_nom = inputs['n_nom']
        self.Torque = inputs['Torque']*safety_factor
        self.h_m = inputs['h_m']
        self.h_ys = inputs['h_ys']
        self.h_yr = inputs['h_yr']
        self.n_s = inputs['n_s']
        self.b_st = inputs['b_st']
        self.d_s = inputs['d_s']
        self.t_ws = inputs['t_ws']
        self.n_r = inputs['n_r']
        self.b_r = inputs['b_r']
        self.d_r = inputs['d_r']
        self.t_wr = inputs['t_wr']
        #self.t = inputs['t']
        #self.t_s = inputs['t_s']
        self.R_o = inputs['R_o']
        self.main_shaft_cm = inputs['main_shaft_cm']
        self.main_shaft_length = inputs['main_shaft_length']
        
        self.rho_Fe = rho_Fe
        self.rho_Fes = rho_Fes
        self.rho_Copper = rho_Copper
        self.rho_PM = rho_PM
        
        self.C_Cu = C_Cu
        self.C_Fe = C_Fe
        self.C_Fes = C_Fes
        self.C_PM = C_PM
        
        #Assign values to universal constants
        
        B_r    =1.2                 # Tesla remnant flux density
        g1     =9.81                # m/s^2 acceleration due to gravity
        E      =2e11                # N/m^2 young's modulus
        sigma  =40e3                # shear stress assumed
        ratio  =0.7                 # ratio of magnet width to pole pitch(bm/self.tau_p)
        mu_0   =pi*4e-7              # permeability of free space
        mu_r   =1.06                                # relative permeability 
        phi    =90*2*pi/360         # tilt angle (rotor tilt -90 degrees during transportation)
        cofi   =0.85                 # power factor
        
        #Assign values to design constants
        h_w    =0.005                                    # Slot wedge height
        h_i    =0.001                                 # coil insulation thickness
        y_tau_p=1                                         # Coil span to pole pitch
        m      =3                    # no of phases
        q1     =1                    # no of slots per pole per phase
        b_s_tau_s=0.45                              # slot width to slot pitch ratio
        k_sfil =0.65                                 # Slot fill factor
        P_Fe0h =4                           #specific hysteresis losses W/kg @ 1.5 T 
        P_Fe0e =1                           #specific hysteresis losses W/kg @ 1.5 T 
        rho_Cu=1.8*10**(-8)*1.4            # Copper resisitivty
        k_fes =0.9                                    # Stator iron fill factor per Grauers
        b_so            =  0.004                    # Slot opening
        alpha_p        =  pi/2*0.7

        # back iron thickness for rotor and stator
        self.t_s =self.h_ys    
        self.t =self.h_yr
        
        
        ###################################################### Electromagnetic design#############################################
        self.K_rad=self.l_s/(2*self.r_s)                            # Aspect ratio
        T =   self.Torque                                                            # rated torque
        l_u       =k_fes * self.l_s                   #useful iron stack length
        We                =self.tau_p
        l_b       = 2*self.tau_p                                          #end winding length
        l_e       =self.l_s+2*0.001*self.r_s     # equivalent core length
        self.b_m  =0.7*self.tau_p                                 # magnet width     
        
        
        # Calculating air gap length
        dia                =  2*self.r_s              # air gap diameter
        g         =  0.001*dia               # air gap length
        r_m         =  self.r_s+self.h_ys+self.h_s #magnet radius
        r_r                =  self.r_s-g             #rotor radius
        
        self.p        =  round(pi*dia/(2*self.tau_p))    # pole pairs
        self.f    =  self.n_nom*self.p/60                    # outout frequency
        self.S                = 2*self.p*q1*m                         # Stator slots
        N_conductors=self.S*2                    
        self.N_s=N_conductors/2/3                                    # Stator turns per phase
        tau_s=pi*dia/self.S                                                # Stator slot pitch
        self.b_s    =  b_s_tau_s*tau_s                        #slot width 
        self.b_t    =  tau_s-(self.b_s)                  #tooth width
        self.Slot_aspect_ratio=self.h_s/self.b_s
        
        # Calculating Carter factor for statorand effective air gap length
        gamma            =  4/pi*(b_so/2/(g+self.h_m/mu_r)*atan(b_so/2/(g+self.h_m/mu_r))-log(sqrt(1+(b_so/2/(g+self.h_m/mu_r))**2)))
        k_C                =  tau_s/(tau_s-gamma*(g+self.h_m/mu_r))   # carter coefficient
        g_eff            =  k_C*(g+self.h_m/mu_r)
        
        # angular frequency in radians
        om_m            =  2*pi*self.n_nom/60
        om_e            =  self.p*om_m/2

        
        # Calculating magnetic loading
        self.B_pm1             =  B_r*self.h_m/mu_r/(g_eff)
        self.B_g=  B_r*self.h_m/mu_r/(g_eff)*(4/pi)*sin(alpha_p)
        self.B_symax=self.B_g*self.b_m*l_e/(2*self.h_ys*l_u)
        self.B_rymax=self.B_g*self.b_m*l_e/(2*self.h_yr*self.l_s)
        self.B_tmax    =self.B_g*tau_s/self.b_t
        
        #Calculating winding factor
        k_wd            = sin(pi/6)/q1/sin(pi/6/q1)      
        
        L_t=self.l_s+2*self.tau_p
        l                    = L_t                          #length
        
        # Calculating no-load voltage induced in the stator
        self.E_p    = 2*(self.N_s)*L_t*self.r_s*k_wd*om_m*self.B_g/sqrt(2)
        
        # Stator winding length ,cross-section and resistance
        l_Cus            = 2*(self.N_s)*(2*self.tau_p+L_t)
        A_s                = self.b_s*(self.h_s-h_w)*q1*self.p
        A_scalc   = self.b_s*1000*(self.h_s*1000-h_w*1000)*q1*self.p
        A_Cus            = A_s*k_sfil/(self.N_s)
        self.A_Cuscalc = A_scalc *k_sfil/(self.N_s)
        self.R_s    = l_Cus*rho_Cu/A_Cus
        
        # Calculating leakage inductance in  stator
        L_m                = 2*m*k_wd**2*(self.N_s)**2*mu_0*self.tau_p*L_t/pi**2/g_eff/self.p
        L_ssigmas=2*mu_0*self.l_s*self.N_s**2/self.p/q1*((self.h_s-h_w)/(3*self.b_s)+h_w/b_so)  #slot leakage inductance
        L_ssigmaew=(2*mu_0*self.l_s*self.N_s**2/self.p/q1)*0.34*g*(l_e-0.64*self.tau_p*y_tau_p)/self.l_s                                #end winding leakage inductance
        L_ssigmag=2*mu_0*self.l_s*self.N_s**2/self.p/q1*(5*(g*k_C/b_so)/(5+4*(g*k_C/b_so))) # tooth tip leakage inductance#tooth tip leakage inductance
        L_ssigma    = (L_ssigmas+L_ssigmaew+L_ssigmag)
        
        self.L_s  = L_m+L_ssigma
        Z=(self.machine_rating/(m*self.E_p))
        
        G=(self.E_p**2-(om_e*self.L_s*Z)**2)
        
        # Calculating stator current and electrical loading
        self.I_s= sqrt(Z**2+(((self.E_p-G**0.5)/(om_e*self.L_s)**2)**2))
        self.J_s    = self.I_s/self.A_Cuscalc
        self.A_1 = 6*self.N_s*self.I_s/(pi*dia)
        I_snom        =(self.machine_rating/m/self.E_p/cofi) #rated current
        I_qnom        =self.machine_rating/(m*self.E_p)
        X_snom        =om_e*(L_m+L_ssigma)
        
        self.B_smax=sqrt(2)*self.I_s*mu_0/g_eff
        
        # Calculating Electromagnetically active mass
        
        V_Cus     =m*l_Cus*A_Cus     # copper volume
        V_Fest    =L_t*2*self.p*q1*m*self.b_t*self.h_s   # volume of iron in stator tooth
        V_Fesy    =L_t*pi*((self.r_s+self.h_s+self.h_ys)**2-(self.r_s+self.h_s)**2) # volume of iron in stator yoke
        V_Fery    =L_t*pi*((r_r-self.h_m)**2-(r_r-self.h_m-self.h_yr)**2)
        self.Copper        =V_Cus*self.rho_Copper
        M_Fest    =V_Fest*self.rho_Fe    # Mass of stator tooth
        M_Fesy    =V_Fesy*self.rho_Fe    # Mass of stator yoke
        M_Fery    =V_Fery*self.rho_Fe    # Mass of rotor yoke
        self.Iron        =M_Fest+M_Fesy+M_Fery
        
        # Calculating Losses
        ##1. Copper Losses
        
        K_R=1.2   # Skin effect correction co-efficient
        P_Cu        =m*I_snom**2*self.R_s*K_R

        # Iron Losses ( from Hysteresis and eddy currents) 
        P_Hyys    =M_Fesy*(self.B_symax/1.5)**2*(P_Fe0h*om_e/(2*pi*60)) # Hysteresis losses in stator yoke
        P_Ftys    =M_Fesy*((self.B_symax/1.5)**2)*(P_Fe0e*(om_e/(2*pi*60))**2) # Eddy losses in stator yoke
        P_Fesynom=P_Hyys+P_Ftys
        P_Hyd=M_Fest*(self.B_tmax/1.5)**2*(P_Fe0h*om_e/(2*pi*60))  # Hysteresis losses in stator teeth
        P_Ftd=M_Fest*(self.B_tmax/1.5)**2*(P_Fe0e*(om_e/(2*pi*60))**2) # Eddy losses in stator teeth
        P_Festnom=P_Hyd+P_Ftd
        
        # additional stray losses due to leakage flux
        P_ad=0.2*(P_Hyys + P_Ftys + P_Hyd + P_Ftd ) 
        pFtm =300 # specific magnet loss
        P_Ftm=pFtm*2*self.p*self.b_m*self.l_s
        
        self.Losses=P_Cu+P_Festnom+P_Fesynom+P_ad+P_Ftm
        self.gen_eff=self.machine_rating*100/(self.machine_rating+self.Losses)
        
        
        
        ################################################## Structural  Design ############################################################
        
        ##Deflection Calculations##
        
        #rotor structure calculations
        
        a_r                = (self.b_r*self.d_r)-((self.b_r-2*self.t_wr)*(self.d_r-2*self.t_wr))  # cross-sectional area of rotor arms
        A_r                = l*self.t                                                                                                                         # cross-sectional area of rotor cylinder 
        N_r                = round(self.n_r)                                                                                                             # rotor arms
        theta_r        =pi*1/N_r                                                                                              # half angle between spokes
        I_r                =l*self.t**3/12                                                                                     # second moment of area of rotor cylinder
        I_arm_axi_r    =((self.b_r*self.d_r**3)-((self.b_r-2*self.t_wr)*(self.d_r-2*self.t_wr)**3))/12  # second moment of area of rotor arm
        I_arm_tor_r    = ((self.d_r*self.b_r**3)-((self.d_r-2*self.t_wr)*(self.b_r-2*self.t_wr)**3))/12  # second moment of area of rotot arm w.r.t torsion
        R                    = self.r_s-g-self.h_m-0.5*self.t                                       # Rotor mean radius
        c                    =R/500
        self.u_all_r    =c/20                                                                     # allowable radial deflection
        R_1                = R-self.t*0.5                                                                #inner radius of rotor cylinder
        k_1                = sqrt(I_r/A_r)                               # radius of gyration
        m1                =(k_1/R)**2
        l_ir            =R                                                  # length of rotor arm beam at which rotor cylinder acts
        l_iir            =R_1 
        self.b_all_r        =2*pi*self.R_o/N_r                                            #allowable circumferential arm dimension for rotor
        
        q3                    = self.B_g**2/2/mu_0                                               # normal component of Maxwell stress
        
        self.mass_PM   =(2*pi*(R+0.5*self.t)*l*self.h_m*ratio*self.rho_PM)           # magnet mass
        
        
        # Calculating radial deflection of the rotor 
        
        Numer=R**3*((0.25*(sin(theta_r)-(theta_r*cos(theta_r)))/(sin(theta_r))**2)-(0.5/sin(theta_r))+(0.5/theta_r))
        Pov=((theta_r/(sin(theta_r))**2)+1/tan(theta_r))*((0.25*R/A_r)+(0.25*R**3/I_r))
        Qov=R**3/(2*I_r*theta_r*(m1+1))
        Lov=(R_1-self.R_o)/a_r
        Denom=I_r*(Pov-Qov+Lov) # radial deflection % rotor
        
        self.u_Ar                =(q3*R**2/E/self.t)*(1+Numer/Denom)
        
        # Calculating axial deflection of the rotor under its own weight
        w_r                    =self.rho_Fes*g1*sin(phi)*a_r*N_r                                                                         # uniformly distributed load of the weight of the rotor arm
        mass_st_lam=self.rho_Fe*2*pi*(R)*l*self.h_yr                                     # mass of rotor yoke steel
        W                =g1*sin(phi)*(mass_st_lam/N_r+(self.mass_PM)/N_r)                                               # weight of 1/nth of rotor cylinder
        
        y_a1=(W*l_ir**3/12/E/I_arm_axi_r)                                                # deflection from weight component of back iron
        y_a2=(w_r*l_iir**4/24/E/I_arm_axi_r)                                                                                         # deflection from weight component of yhe arms
        self.y_Ar       =y_a1+y_a2 # axial deflection
        self.y_all     =2*l/100    # allowable axial deflection
        
        # Calculating # circumferential deflection of the rotor
        self.z_all_r     =0.05*2*pi*R/360                                                                                                                           # allowable torsional deflection
        self.z_A_r       =(2*pi*(R-0.5*self.t)*l/N_r)*sigma*(l_ir-0.5*self.t)**3/3/E/I_arm_tor_r       # circumferential deflection
        
        val_str_rotor        = self.mass_PM+((mass_st_lam)+(N_r*(R_1-self.R_o)*a_r*self.rho_Fes))           #rotor mass
        

        

        
        #stator structure deflection calculation
        
        a_s       = (self.b_st*self.d_s)-((self.b_st-2*self.t_ws)*(self.d_s-2*self.t_ws)) # cross-sectional area of stator armms
        A_st      =l*self.t_s                                                                                                                            # cross-sectional area of stator cylinder 
        N_st            = round(self.n_s)                                                                                                                # stator arms
        theta_s        =pi*1/N_st                                                                                                                             # half angle between spokes
        I_st       =l*self.t_s**3/12                                                                                                  # second moment of area of stator cylinder
        k_2       = sqrt(I_st/A_st)                                                                                                             # radius of gyration
        
        I_arm_axi_s    =((self.b_st*self.d_s**3)-((self.b_st-2*self.t_ws)*(self.d_s-2*self.t_ws)**3))/12  # second moment of area of stator arm
        I_arm_tor_s    = ((self.d_s*self.b_st**3)-((self.d_s-2*self.t_ws)*(self.b_st-2*self.t_ws)**3))/12  # second moment of area of rotot arm w.r.t torsion
        R_st             =self.r_s+self.h_s+self.h_ys*0.5                                        # stator cylinder mean radius
        R_1s      = R_st-self.t_s*0.5                                                                                                            #inner radius of stator cylinder, m
        m2        =(k_2/R_st)**2
        d_se=dia+2*(self.h_ys+self.h_s+h_w)  # stator outer diameter
        
        # allowable radial deflection of stator
        c1        =R_st/500
        self.u_all_s    = c1/20
        
                        
        self.R_out=(R/0.995+self.h_s+self.h_ys)
        l_is      =R_st-self.R_o                                                                                                                    # distance at which the weight of the stator cylinder acts
        l_iis     =l_is                                                                                                                                        # distance at which the weight of the stator cylinder acts
        l_iiis    =l_is                                                                                                                                        # distance at which the weight of the stator cylinder acts
        
        
                
        mass_st_lam_s= M_Fest+pi*L_t*self.rho_Fe*((R_st+0.5*self.h_ys)**2-(R_st-0.5*self.h_ys)**2)
        W_is            =0.5*g1*sin(phi)*(self.rho_Fes*l*self.d_s**2)                          # length of stator arm beam at which self-weight acts
        W_iis     =g1*sin(phi)*(mass_st_lam_s+V_Cus*self.rho_Copper)/2/N_st                             # weight of stator cylinder and teeth
        w_s         =self.rho_Fes*g1*sin(phi)*a_s*N_st                                                                     # uniformly distributed load of the arms
        
        #print (M_Fest+self.Copper)*g1
        
        mass_stru_steel  =2*(N_st*(R_1s-self.R_o)*a_s*self.rho_Fes)                                            # Structural mass of stator arms
        
        # Calculating radial deflection of the stator
        
        Numers=R_st**3*((0.25*(sin(theta_s)-(theta_s*cos(theta_s)))/(sin(theta_s))**2)-(0.5/sin(theta_s))+(0.5/theta_s))
        Povs=((theta_s/(sin(theta_s))**2)+1/tan(theta_s))*((0.25*R_st/A_st)+(0.25*R_st**3/I_st))
        Qovs=R_st**3/(2*I_st*theta_s*(m2+1))
        Lovs=(R_1s-self.R_o)*0.5/a_s
        Denoms=I_st*(Povs-Qovs+Lovs)
        
        self.u_As                =(q3*R_st**2/E/self.t_s)*(1+Numers/Denoms)
        
        # Calculating axial deflection of the stator
        
        X_comp1 = (W_is*l_is**3/12/E/I_arm_axi_s)                                                                                # deflection component due to stator arm beam at which self-weight acts
        X_comp2 =(W_iis*l_iis**4/24/E/I_arm_axi_s)                                                                            # deflection component due to 1/nth of stator cylinder
        X_comp3 =w_s*l_iiis**4/24/E/I_arm_axi_s                                                                                    # deflection component due to weight of arms
        
        self.y_As       =X_comp1+X_comp2+X_comp3                                                                              # axial deflection
        
        # Calculating circumferential deflection of the stator
        self.z_A_s  =2*pi*(R_st+0.5*self.t_s)*l/(2*N_st)*sigma*(l_is+0.5*self.t_s)**3/3/E/I_arm_tor_s 
        self.z_all_s     =0.05*2*pi*R_st/360                                                                                      # allowable torsional deflection
        self.b_all_s        =2*pi*self.R_o/N_st                                                                                        # allowable circumferential arm dimension
        
        val_str_stator        = mass_stru_steel+mass_st_lam_s
        val_str_mass=val_str_rotor+val_str_stator
        
        self.TC1=T/(2*pi*sigma)     # Desired shear stress 
        self.TC2=R**2*l              # Evaluating Torque constraint for rotor
        self.TC3=R_st**2*l           # Evaluating Torque constraint for stator
        
        self.Structural_mass=mass_stru_steel+(N_r*(R_1-self.R_o)*a_r*self.rho_Fes)
        Stator=mass_st_lam_s+mass_stru_steel+self.Copper
        Rotor=((2*pi*self.t*L_t*(R)*self.rho_Fe)+(N_r*(R_1-self.R_o)*a_r*self.rho_Fes))+self.mass_PM
        self.Mass=Stator+Rotor
            
        # Calculating mass moments of inertia and center of mass
        self.I = np.zeros(3)
        self.cm = np.zeros(3)
        
        self.I[0]   = (0.5*self.Mass*self.R_out**2)
        self.I[1]   = (0.25*self.Mass*self.R_out**2+(1/12)*self.Mass*self.l_s**2) 
        self.I[2]   = self.I[1]
        self.cm[0]  = self.main_shaft_cm[0] + self.main_shaft_length/2. + self.l_s/2
        self.cm[1]  = self.main_shaft_cm[1]
        self.cm[2]  = self.main_shaft_cm[2]
        
        # Costs
        K_gen=self.Copper*self.C_Cu+self.Iron*self.C_Fe+self.C_PM*self.mass_PM
        Cost_str=self.C_Fes*self.Structural_mass
        self.Costs=K_gen+Cost_str
        
        
        
        # outputs
        outputs['B_symax'] = self.B_symax
        outputs['B_tmax'] = self.B_tmax
        outputs['B_rymax'] = self.B_rymax
        outputs['B_smax'] = self.B_smax
        outputs['B_pm1'] = self.B_pm1
        outputs['B_g'] = self.B_g
        outputs['N_s'] = self.N_s
        outputs['b_s'] = self.b_s
        outputs['b_t'] = self.b_t
        outputs['A_Cuscalc'] = self.A_Cuscalc
        outputs['b_m'] = self.b_m
        outputs['p'] = self.p
        outputs['E_p'] = self.E_p
        outputs['f'] = self.f
        outputs['I_s'] = self.I_s
        outputs['R_s'] = self.R_s
        outputs['L_s'] = self.L_s
        outputs['A_1'] = self.A_1
        outputs['J_s'] = self.J_s
        outputs['Mass'] = self.Mass
        outputs['K_rad'] = self.K_rad
        outputs['Losses'] = self.Losses
        outputs['gen_eff'] = self.gen_eff
        outputs['u_Ar'] = self.u_Ar
        outputs['y_Ar'] = self.y_Ar
        outputs['z_A_r'] = self.z_A_r
        outputs['u_As'] = self.u_As
        outputs['y_As'] = self.y_As
        outputs['z_A_s'] = self.z_A_s
        outputs['u_all_r'] = self.u_all_r
        outputs['u_all_s'] = self.u_all_s
        outputs['y_all'] = self.y_all
        outputs['z_all_s'] = self.z_all_s
        outputs['z_all_r'] = self.z_all_r
        outputs['b_all_s'] = self.b_all_s
        outputs['b_all_r'] = self.b_all_r
        outputs['TC1'] = self.TC1
        outputs['TC2'] = self.TC2
        outputs['TC3'] = self.TC3
        outputs['R_out'] = self.R_out
        outputs['S'] = self.S
        outputs['Slot_aspect_ratio'] = self.Slot_aspect_ratio
        outputs['mass_PM'] = self.mass_PM
        outputs['Copper'] = self.Copper
        outputs['Iron'] = self.Iron
        outputs['Structural_mass'] = self.Structural_mass
        outputs['cm'] = self.cm
        outputs['I'] = self.I
        outputs['Costs'] = self.Costs
        



#############################################################################
##############################  PMSG Optimizer ##############################
############################################################################# 
class PMSG_Optimizer(Group):
    def setup(self):
        i = self.add_subsystem('i', IndepVarComp(), promotes=['*'])
        i.add_output('r_s', desc = 'airgap radius r_s')
        i.add_output('l_s', desc = 'Stator core length l_s')
        i.add_output('h_s', desc = 'Yoke height h_s')
        i.add_output('tau_p', desc = 'Pole pitch self.tau_p')
        i.add_output('machine_rating', desc = 'Machine rating')
        i.add_output('n_nom', desc = 'rated speed')
        i.add_output('Torque', desc = 'Rated torque ')
        i.add_output('h_m', desc = 'magnet height')
        i.add_output('h_ys', desc = 'Yoke height')
        i.add_output('h_yr', desc = 'rotor yoke height')        
        i.add_output('n_s', desc = 'number of stator arms n_s')
        i.add_output('b_st', desc = 'arm width b_r')
        i.add_output('d_s', desc = 'arm depth d_r')
        i.add_output('t_ws', desc = 'arm depth thickness self.t_wr')
        i.add_output('n_r', desc = 'number of arms n')
        i.add_output('b_r', desc = 'arm width b_r')
        i.add_output('d_r', desc = 'arm depth d_r')
        i.add_output('t_wr', desc = 'arm depth thickness self.t_wr')
        i.add_output('R_o', desc = 'Shaft radius')        
        i.add_output('main_shaft_cm', desc = 'Main Shaft CM', val=np.zeros(3))
        i.add_output('main_shaft_length', desc = 'main shaft length')
        
        self.add_subsystem('pmsg', PMSG(), promotes_inputs=['*'])
        self.add_subsystem('obj', ExecComp('f = -1 * eff'))
        self.add_subsystem('c1', ExecComp('diff = B_smax - B_g'))
        self.add_subsystem('c2', ExecComp('diff = u_As - u_all_s'))
        self.add_subsystem('c3', ExecComp('diff = z_A_s - z_all_s'))
        self.add_subsystem('c4', ExecComp('diff = y_As - y_all'))
        self.add_subsystem('c5', ExecComp('diff = u_Ar - u_all_r'))
        self.add_subsystem('c6', ExecComp('diff = y_Ar - y_all'))
        self.add_subsystem('c7', ExecComp('diff = TC1 - TC2'))
        self.add_subsystem('c8', ExecComp('diff = TC1 - TC3'))
        self.add_subsystem('c9', ExecComp('diff = b_r - b_all_r'))
        self.add_subsystem('c10', ExecComp('diff = b_st - b_all_s'))
        self.add_subsystem('c11', ExecComp('diff = z_A_r - z_all_r'))


        self.connect('pmsg.gen_eff', 'obj.eff')
        self.connect('pmsg.B_smax', 'c1.B_smax')
        self.connect('pmsg.B_g', 'c1.B_g')
        self.connect('pmsg.u_As', 'c2.u_As')
        self.connect('pmsg.u_all_s', 'c2.u_all_s')
        self.connect('pmsg.z_A_s', 'c3.z_A_s')
        self.connect('pmsg.z_all_s', 'c3.z_all_s')
        self.connect('pmsg.y_As', 'c4.y_As')        
        self.connect('pmsg.y_all', 'c4.y_all')
        self.connect('pmsg.u_Ar', 'c5.u_Ar')
        self.connect('pmsg.u_all_r', 'c5.u_all_r')
        self.connect('pmsg.y_Ar', 'c6.y_Ar')
        self.connect('pmsg.y_all', 'c6.y_all')
        self.connect('pmsg.TC1', 'c7.TC1')
        self.connect('pmsg.TC2', 'c7.TC2')
        self.connect('pmsg.TC1', 'c8.TC1')
        self.connect('pmsg.TC3', 'c8.TC3')
        self.connect('b_r', 'c9.b_r')
        self.connect('pmsg.b_all_r', 'c9.b_all_r')
        self.connect('b_st', 'c10.b_st')
        self.connect('pmsg.b_all_s', 'c10.b_all_s')
        self.connect('pmsg.z_A_r', 'c11.z_A_r')
        self.connect('pmsg.z_all_r', 'c11.z_all_r')
        
    def optimizer(self):
        prob = Problem()
        prob.model = PMSG_Optimizer()
       
        # SLSQP, COBYLA
        prob.driver = ScipyOptimizer()
        prob.driver.options['optimizer'] = 'SLSQP'
        prob.driver.options['maxiter'] = 50
        prob.driver.options['tol'] = 1e-6
        
#         # CONMIN
#         prob.driver = pyOptSparseDriver()
#         prob.driver.options['optimizer'] = 'SLSQP'
#         prob.driver.options['gradient method'] = 'openmdao'
#         prob.driver.options['print_results'] = True
        
        # objective function
        #prob.model.add_objective('pmsg.Costs', ref0=0, ref=20000) 
        prob.model.add_objective('obj.f', ref0=0, ref=100.0) 
        
        # design variables
        prob.model.add_design_var('r_s', lower=0.5, upper=9, ref0=0.5, ref=9)
        prob.model.add_design_var('l_s', lower=0.5, upper=2.5, ref0=0.5, ref=2.5)
        prob.model.add_design_var('h_s', lower=0.04, upper=0.1, ref0=0.04, ref=0.1)
        prob.model.add_design_var('tau_p', lower=0.04, upper=0.1, ref0=0.04, ref=0.1)
        prob.model.add_design_var('h_m', lower=0.005, upper=0.1, ref0=0.005, ref=0.1)
        prob.model.add_design_var('n_r', lower=5, upper=15, ref0=5, ref=15)
        prob.model.add_design_var('h_yr', lower=0.045, upper=0.25, ref0=0.045, ref=0.25)
        prob.model.add_design_var('h_ys', lower=0.045, upper=0.25, ref0=0.045, ref=0.25)
        prob.model.add_design_var('b_r', lower=0.1, upper=1.5, ref0=0.1, ref=1.5)
        prob.model.add_design_var('d_r', lower=0.1, upper=1.5, ref0=0.1, ref=1.5)
        prob.model.add_design_var('t_wr', lower=0.001, upper=0.2, ref0=0.001, ref=0.2)
        prob.model.add_design_var('n_s', lower=5, upper=15, ref0=5, ref=15)
        prob.model.add_design_var('b_st', lower=0.1, upper=1.5, ref0=0.1, ref=1.5)
        prob.model.add_design_var('d_s', lower=0.1, upper=1.5, ref0=0.1, ref=1.5)
        prob.model.add_design_var('t_ws', lower=0.001, upper=0.2, ref0=0.001, ref=0.2)

        
        # constraints - denormalized
        prob.model.add_constraint('pmsg.B_symax', upper=2.0)                                          #1
        prob.model.add_constraint('pmsg.B_rymax', upper=2.0)                                          #2
        prob.model.add_constraint('pmsg.B_tmax', upper=2.0)                                          #3
        prob.model.add_constraint('c1.diff', upper=0.0)                                 #4
        prob.model.add_constraint('pmsg.B_g', lower=0.7, upper=1.2)                                              #5                
        prob.model.add_constraint('pmsg.E_p', lower=500.0, upper=5000.0)                                              #7
        prob.model.add_constraint('c2.diff', upper=0.0)                            #9
        prob.model.add_constraint('c3.diff', upper=0.0)                            #10
        prob.model.add_constraint('c4.diff', upper=0.0)                              #11
        prob.model.add_constraint('c5.diff', upper=0.0)                            #12
        prob.model.add_constraint('c6.diff', upper=0.0)                            #13
        prob.model.add_constraint('c7.diff', upper=0.0)                                 #14
        prob.model.add_constraint('c8.diff', upper=0.0)                                    #15
        prob.model.add_constraint('c9.diff', upper=0.0)                                    #16
        prob.model.add_constraint('c10.diff', upper=0.0)                                #17
        prob.model.add_constraint('c11.diff', upper=0.0)                            #18
        prob.model.add_constraint('pmsg.A_1', upper=60000.0)                                            #19
        prob.model.add_constraint('pmsg.J_s', upper=6.0)                                                 #20
        prob.model.add_constraint('pmsg.A_Cuscalc', lower=5.0)                                     #21
        prob.model.add_constraint('pmsg.K_rad', lower=0.2, upper=0.27)                                            #22
        prob.model.add_constraint('pmsg.Slot_aspect_ratio', lower=4, upper=10)                        #24
        prob.model.add_constraint('pmsg.gen_eff', lower=93.0)                                           #constraint 19        
        
        prob.setup(check=True, mode='fwd')
        #view_model(prob)
        
        return prob                


#############################################################################
#################################  Unit Test ################################
#############################################################################
def unit_test():
    inputs={'r_s' : 3.26, \
            'l_s' : 1.6, \
            'h_s' : 0.07, \
            'tau_p' : 0.08, \
            'machine_rating' : 5e6, \
            'n_nom' : 12.111*100, \
            'Torque' : 4.143289e6/100, \
            'h_m' : 0.009, \
            'h_ys' : 0.075, \
            'h_yr' : 0.075, \
            'n_s' : 5, \
            'b_st' : 0.480, \
            'd_s' : 0.35, \
            't_ws' : 0.06, \
            'n_r' : 5, \
            'b_r' : 0.53, \
            'd_r' : 0.7, \
            't_wr' : 0.06, \
            'R_o' : 0.43, \
            'main_shaft_cm' : np.zeros(3), \
            'main_shaft_length' : 0.0}
    outputs={}
    PMSG().compute(inputs, outputs)  
    beautify_output(outputs)   
    

def optimization():
    prob = PMSG_Optimizer().optimizer()
    
    prob['i.r_s'] = 3.26
    prob['i.l_s'] = 1.6
    prob['i.h_s'] = 0.07
    prob['i.tau_p'] = 0.08
    prob['i.machine_rating'] = 5000000.
    prob['i.n_nom'] = 12.111
    prob['i.Torque'] = 4143289.0
    prob['i.h_m'] = 0.009
    prob['i.h_ys'] = 0.075
    prob['i.h_yr'] = 0.075
    prob['i.n_s'] = 5
    prob['i.b_st'] = 0.48
    prob['i.d_s'] = 0.35
    prob['i.t_ws'] = 0.06
    prob['i.n_r'] = 5
    prob['i.b_r'] = 0.53
    prob['i.d_r'] = 0.7
    prob['i.t_wr'] = 0.06
    prob['i.R_o'] = 0.43
    
    prob.run_model()
    #prob.set_solver_print(level=2)
    prob.model.approx_totals('fd')
    prob.run_driver()
    
    print('minimum found at')
    print 'r_s', prob['pmsg.r_s'][0]
    print 'l_s', prob['pmsg.l_s'][0]
    print 'h_s', prob['pmsg.h_s'][0]
    print 'tau_p', prob['pmsg.tau_p'][0]
    print 'n_r', prob['pmsg.n_r'][0]
    print 'h_m', prob['pmsg.h_m'][0]
    print 'h_ys', prob['pmsg.h_ys'][0]
    print 'h_yr', prob['pmsg.h_yr'][0]
    print 'n_s', prob['pmsg.n_s'][0]
    print 'b_r', prob['pmsg.b_r'][0]
    print 'd_r', prob['pmsg.d_r'][0]
    print 't_wr', prob['pmsg.t_wr'][0]
    print 'b_st', prob['pmsg.b_st'][0]
    print 'd_s', prob['pmsg.d_s'][0]
    print 't_ws', prob['pmsg.t_ws'][0]
    print '-'*5
    
    print 'gen_eff', prob['pmsg.gen_eff']
    print 'Mass', prob['pmsg.Mass']
    print 'Costs', prob['pmsg.Costs']
    print 'Frequency', prob['pmsg.f']
    print '-'*5
    
    
    print 'B_symax<2', prob['pmsg.B_symax'][0]
    print 'B_rymax<2', prob['pmsg.B_rymax'][0]
    print 'B_tmax<2', prob['pmsg.B_tmax'][0]
    print 'B_smax<B_g', prob['c1.diff'][0]
    print '0.7<=B_g<=1.2', prob['pmsg.B_g'][0]
    print '500<=E_p<=5000', prob['pmsg.E_p'][0]
    print 'u_As<u_all_s', prob['c2.diff'][0]
    print 'z_A_s<z_all_s', prob['c3.diff'][0]
    print 'y_As<y_all', prob['c4.diff'][0]
    print 'u_Ar<u_all_r', prob['c5.diff'][0]
    print 'z_A_r<z_all_r', prob['c11.diff'][0]
    print 'y_Ar<y_all', prob['c6.diff'][0]
    print 'TC1<TC2', prob['c7.diff'][0]
    print 'TC1<TC3', prob['c8.diff'][0]
    print 'b_r<b_all_r', prob['c9.diff'][0]
    print 'b_st<b_all_s', prob['c10.diff'][0]
    print 'A_1<60000', prob['pmsg.A_1'][0]
    print 'J_s<=6', prob['pmsg.J_s'][0]
    print 'A_Cuscalc>=5', prob['pmsg.A_Cuscalc'][0]
    print '0.2<K_rad<=0.27', prob['pmsg.K_rad'][0]
    print '4<=Slot_aspect_ratio<=10', prob['pmsg.Slot_aspect_ratio'][0]
    print 'gen_eff>=93.0', prob['pmsg.gen_eff'][0]

   
    
if __name__ == "__main__":
    unit_test()
    #optimization()
    