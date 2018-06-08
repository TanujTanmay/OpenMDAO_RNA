from math import pi, sqrt, sin, cos
from openmdao.api import IndepVarComp, ExecComp, ExplicitComponent, Group, Problem, ScipyOptimizer, view_model
#from openmdao.drivers.pyoptsparse_driver import pyOptSparseDriver

from fixed_parameters import beautify_output, rho_Fe, rho_Copper, C_Cu, C_Fe, C_Fes



#############################################################################
##############################  DFIG Main Model #############################
#############################################################################        
class DFIG(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('r_s', desc = 'airgap radius r_s')
        self.add_input('l_s', desc = 'Stator core length l_s')
        self.add_input('h_s', desc = 'Yoke height h_s')
        self.add_input('h_r', desc = 'Stator slot height ')
        self.add_input('B_symax', desc = 'Peak Yoke flux density B_ymax')
        self.add_input('machine_rating', desc = 'Machine rating')
        self.add_input('n_nom', desc = 'rated speed ')
        self.add_input('highSpeedSide_cm', desc = ' high speed sidde COM [x, y, z]', val=[0.0, 0.0, 0.0])
        self.add_input('highSpeedSide_length', desc = 'high speed side length', val=0.0)
        self.add_input('Gearbox_efficiency', desc = 'Gearbox efficiency')
        self.add_input('S_Nmax', desc = 'Stator slot height ')
        self.add_input('I_0', desc = 'Rotor current at no-load')

        
        # outputs
        self.add_output('tau_p', desc = 'Pole pitch')
        self.add_output('p', desc = 'Pole pairs')
        self.add_output('B_g', desc = 'Peak air gap flux density B_g')
        self.add_output('q1', desc = 'Slots per pole per phase')
        self.add_output('h_ys', desc = ' stator yoke height')
        self.add_output('h_yr', desc = ' rotor yoke height')
        #self.add_output('B_g', desc = 'Peak air gap flux density B_g')
        self.add_output('B_g1', desc = 'air gap flux density fundamental ')
        self.add_output('B_rymax', desc = 'maximum flux density in rotor yoke')
        self.add_output('B_tsmax', desc = 'maximum tooth flux density in stator')
        self.add_output('B_trmax', desc = 'maximum tooth flux density in rotor')
        self.add_output('S', desc = 'Stator slots')
        self.add_output('Q_r', desc = 'Rotor slots')
        self.add_output('N_s', desc = 'Stator turns')
        self.add_output('N_r', desc = 'Rotor turns')
        self.add_output('M_actual', desc = 'Actual mass')
        #self.add_output('p', desc = 'No of pole pairs')
        self.add_output('f', desc = 'Output frequency')
        self.add_output('E_p', desc = 'Stator phase voltage')
        self.add_output('I_s', desc = 'Generator output phase current')
        self.add_output('b_s', desc = 'stator slot width')
        self.add_output('b_r', desc = 'rotor slot width')
        self.add_output('b_t', desc = 'stator tooth width')
        self.add_output('b_trmin', desc = 'minimum tooth width')
        self.add_output('b_tr', desc = 'rotor tooth width')
        #self.add_output('gen_eff', desc = 'Generator efficiency')
        self.add_output('Structural_mass', desc = 'Structural mass')
        self.add_output('Active', desc = 'Active Mass')
        self.add_output('TC1', desc = 'Torque constraint')
        self.add_output('TC2', desc = 'Torque constraint')
        self.add_output('A_1', desc = 'Specific current loading')
        self.add_output('J_s', desc = 'Stator winding current density')
        self.add_output('J_r', desc = 'Rotor winding current density')
        self.add_output('K_rad', desc = 'Stack length ratio')
        self.add_output('D_ratio', desc = 'Stator diameter ratio')
        self.add_output('A_Cuscalc', desc = 'Stator conductor cross-section')
        self.add_output('A_Curcalc', desc = 'Rotor conductor cross-section')
        self.add_output('Current_ratio', desc = 'Rotor current ratio')
        self.add_output('Slot_aspect_ratio1', desc = 'Slot apsect ratio')
        self.add_output('Slot_aspect_ratio2', desc = 'Slot apsect ratio')
        #self.add_output('K_rad', desc = 'Aspect ratio')
        self.add_output('gen_eff', desc = 'Generator efficiency')
        self.add_output('Overall_eff', desc = 'Overall drivetrain efficiency')
        self.add_output('R_s', desc = 'Stator resistance')
        self.add_output('L_sm', desc = 'mutual inductance')
        self.add_output('R_R', desc = 'Rotor resistance')
        self.add_output('L_r', desc = 'Rotor impedance')
        self.add_output('L_s', desc = 'Stator synchronising inductance')
        self.add_output('Copper', desc = 'Copper mass')
        self.add_output('Iron', desc = 'Iron mass')
        self.add_output('Losses', desc = 'Total power Loss')
        self.add_output('Mass', desc = 'Total mass')
        self.add_output('cm', desc = 'COM [x,y,z]', shape=3)
        self.add_output('I', desc = ' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=(3,1))
        self.add_output('Costs', desc = 'material cost estimate for a DFIG generator')
        
        
    def compute(self, inputs, outputs):
        # inputs
        self.r_s = inputs['r_s']
        self.l_s = inputs['l_s']
        self.h_s = inputs['h_s']
        self.h_r = inputs['h_r']
        self.B_symax = inputs['B_symax']
        self.machine_rating = inputs['machine_rating']
        self.n_nom = inputs['n_nom']
        self.highSpeedSide_cm = inputs['highSpeedSide_cm']
        self.highSpeedSide_length = inputs['highSpeedSide_length']
        self.Gearbox_efficiency = inputs['Gearbox_efficiency']
        self.S_Nmax = inputs['S_Nmax']
        self.I_0 = inputs['I_0'] 
        
        self.rho_Fe = rho_Fe
        self.rho_Copper = rho_Copper

        
        #Assign values to universal constants
        g1          = 9.81                  # m/s^2 acceleration due to gravity
        sigma       = 21.5e3                # shear stress
        mu_0        = pi*4e-7               # permeability of free space
        cofi        = 0.9                   # power factor
        h_w         = 0.005                 # wedge height
        m           = 3                     # Number of phases
        rho_Cu      = 1.8*10**(-8)*1.4      # copper resisitivity
        h_sy0       = 0
        
        #Assign values to design constants
        b_so = 0.004                                                 # Stator slot opening
        b_ro = 0.004                                                 # rotor slot openinng
        self.q1=5                                                   # Stator slots per pole per phase
        q2=self.q1-1                                             # Rotor slots per pole per phase
        k_sfil =0.65                                              # Stator Slot fill factor
        P_Fe0h =4                                        # specific hysteresis losses W/kg @ 1.5 T 
        P_Fe0e =1                                 # specific hysteresis losses W/kg @ 1.5 T 
        b_s_tau_s=0.45                                         # Stator slot width /slot pitch ratio
        b_r_tau_r=0.45                                         # Rotor slot width /slot pitch ratio
        y_tau_p=12./15                                         # Stator coil span to pole pitch    
        y_tau_pr=10./12                                         # Rotor coil span to pole pitch
        self.p=3                                                     # pole pairs
        freq=60                                                         # grid frequency in Hz
        k_fillr = 0.55                                         # Rotor Slot fill factor
        
        K_rs     =1/(-1*self.S_Nmax)             # Winding turns ratio between rotor and Staor        
        I_SN   = self.machine_rating/(sqrt(3)*3000) # Rated current
        I_SN_r =I_SN/K_rs                                     # Stator rated current reduced to rotor
                    
        # Calculating winding factor for stator and rotor    
        k_y1=sin(pi*0.5*y_tau_p)                                        # winding Chording factor
        k_q1=sin(pi/6)/(self.q1*sin(pi/(6*self.q1))) # winding zone factor
        k_y2=sin(pi*0.5*y_tau_pr)                                        # winding Chording factor
        k_q2=sin(pi/6)/(q2*sin(pi/(6*q2)))                    # winding zone factor
        k_wd1=k_y1*k_q1                                                            # Stator winding factor
        k_wd2=k_q2*k_y2                                                            # Rotor winding factor
        
        
        dia=2*self.r_s                                                              # air gap diameter
        g=(0.1+0.012*(self.machine_rating)**(1./3))*0.001  #air gap length in m
        self.K_rad=self.l_s/dia                                                         # Aspect ratio
        r_r=self.r_s-g                                                              #rotor radius
        self.tau_p=(pi*dia/(2*self.p))                                         #pole pitch
        

        self.S=2*self.p*self.q1*m                                                    # Stator Slots
        N_slots_pp=self.S/(m*self.p*2)                                        # Number of stator slots per pole per phase

        n  = self.S/2*self.p/self.q1                                    #no of slots per pole per phase
        tau_s=self.tau_p/(m*self.q1)                                    #slot pitch
        self.b_s=b_s_tau_s*tau_s;                                        #Stator slot width
        self.b_t=tau_s-self.b_s                                          #Stator tooth width


        self.Q_r=2*self.p*m*q2                                                        # Rotor Slots                                
        tau_r=pi*(dia-2*g)/self.Q_r                                                # Rotor Slot pitch
        self.b_r=b_r_tau_r*tau_r                                                    # Rotot Slot width
        self.b_tr=tau_r-self.b_r                                                    # Rotor tooth width
        
        # Calculating equivalent slot openings
        mu_rs                =0.005
        mu_rr                =0.005
        W_s                    =(self.b_s/mu_rs)*1e-3  #in m
        W_r                    =(self.b_r/mu_rr)*1e-3  #in m
        
        self.Slot_aspect_ratio1=self.h_s/self.b_s
        self.Slot_aspect_ratio2=self.h_r/self.b_r
        
        # Calculating Carter factor for stator,rotor and effective air gap length
        gamma_s    = (2*W_s/g)**2/(5+2*W_s/g)
        K_Cs=(tau_s)/(tau_s-g*gamma_s*0.5)  #page 3-13
        gamma_r        = (2*W_r/g)**2/(5+2*W_r/g)
        K_Cr=(tau_r)/(tau_r-g*gamma_r*0.5)  #page 3-13
        K_C=K_Cs*K_Cr
        g_eff=K_C*g
    
    
        om_m=2*pi*self.n_nom/60                                                            # mechanical frequency
        om_e=self.p*om_m                                                                        # Electrical frequency
        K_s=0.3                                                                                       # Saturation factor for Iron
        n_c1=2                                                                                        #number of conductors per coil
        a1 =2                                                                                            # number of parallel paths
        self.N_s=round(2*self.p*N_slots_pp*n_c1/a1)                    # Stator winding turns per phase
        
        self.N_r=round(self.N_s*k_wd1*K_rs/k_wd2)                        # Rotor winding turns per phase
        n_c2=self.N_r/(self.Q_r/m)                                                    # rotor turns per coil                                    
        
        # Calculating peak flux densities and back iron thickness
        self.B_g1=mu_0*3*self.N_r*self.I_0*2**0.5*k_y2*k_q2/(pi*self.p*g_eff*(1+K_s))
        self.B_g=self.B_g1*K_C
        self.h_ys = self.B_g*self.tau_p/(self.B_symax*pi)
        self.B_rymax = self.B_symax
        self.h_yr=self.h_ys
        self.B_tsmax=self.B_g*tau_s/(self.b_t) 
        
        d_se=dia+2*(self.h_ys+self.h_s+h_w)                                  # stator outer diameter
        self.D_ratio=d_se/dia                                                                # Diameter ratio
        self.f = self.n_nom*self.p/60
        
        # Stator slot fill factor
        if (2*self.r_s>2):
            K_fills=0.65
        else:
            K_fills=0.4
            
        # Stator winding calculation
   
        # End connection length  for stator winding coils
        l_fs=2*(0.015+y_tau_p*self.tau_p/2/cos(40))+pi*(self.h_s)
        
        l_Cus = 2*self.N_s*(l_fs+self.l_s)/a1             # Length of Stator winding 
        
        # Conductor cross-section
        A_s = self.b_s*(self.h_s-h_w)
        A_scalc=self.b_s*1000*(self.h_s*1000-h_w*1000)
        A_Cus = A_s*self.q1*self.p*K_fills/self.N_s
        self.A_Cuscalc = A_scalc*self.q1*self.p*K_fills/self.N_s
        
        # Stator winding resistance
        self.R_s=l_Cus*rho_Cu/A_Cus
        tau_r_min=pi*(dia-2*(g+self.h_r))/self.Q_r
        
        # Peak magnetic loading on the rotor tooth
        self.b_trmin=tau_r_min-b_r_tau_r*tau_r_min
        self.B_trmax = self.B_g*tau_r/self.b_trmin
        
        
        # Calculating leakage inductance in  stator
        
        K_01=1-0.033*(W_s**2/g/tau_s)
        sigma_ds=0.0042
        K_02=1-0.033*(W_r**2/g/tau_r)
        sigma_dr=0.0062
                        
        L_ssigmas=(2*mu_0*self.l_s*n_c1**2*self.S/m/a1**2)*((self.h_s-h_w)/(3*self.b_s)+h_w/b_so)  #slot leakage inductance
        L_ssigmaew=(2*mu_0*self.l_s*n_c1**2*self.S/m/a1**2)*0.34*self.q1*(l_fs-0.64*self.tau_p*y_tau_p)/self.l_s #end winding leakage inductance
        L_ssigmag=(2*mu_0*self.l_s*n_c1**2*self.S/m/a1**2)*(0.9*tau_s*self.q1*k_wd1*K_01*sigma_ds/g_eff) # tooth tip leakage inductance
        L_ssigma=(L_ssigmas+L_ssigmaew+L_ssigmag)                                                                                                              # stator leakage inductance
        self.L_sm =6*mu_0*self.l_s*self.tau_p*(k_wd1*self.N_s)**2/(pi**2*(self.p)*g_eff*(1+K_s))
        self.L_s=(L_ssigmas+L_ssigmaew+L_ssigmag)                                                                                                              # stator  inductance
        
        # Calculating leakage inductance in  rotor
        l_fr=(0.015+y_tau_pr*tau_r/2/cos(40*pi/180))+pi*(self.h_r)  # Rotor end connection length
        L_rsl=(mu_0*self.l_s*(2*n_c2)**2*self.Q_r/m)*((self.h_r-h_w)/(3*self.b_r)+h_w/b_ro)  #slot leakage inductance
        L_rel= (mu_0*self.l_s*(2*n_c2)**2*self.Q_r/m)*0.34*q2*(l_fr-0.64*tau_r*y_tau_pr)/self.l_s #end winding leakage inductance                  #end winding leakage inductance
        L_rtl=(mu_0*self.l_s*(2*n_c2)**2*self.Q_r/m)*(0.9*tau_s*q2*k_wd2*K_02*sigma_dr/g_eff) # tooth tip leakage inductance
        self.L_r=(L_rsl+L_rtl+L_rel)/K_rs**2  # rotor leakage inductance
        sigma1=1-(self.L_sm**2/self.L_s/self.L_r)
        
        #Rotor Field winding
            
        # conductor cross-section
        diff=self.h_r-h_w
        A_Cur=k_fillr*self.p*q2*self.b_r*diff/self.N_r
        self.A_Curcalc=A_Cur*1e6
        
        L_cur=2*self.N_r*(l_fr+self.l_s)                            # rotor winding length
        R_r=rho_Cu*L_cur/A_Cur                                                # Rotor resistance
        
        # Equivalent rotor resistance reduced to stator
        self.R_R=R_r/(K_rs**2)
        
        om_s=(self.n_nom)*2*pi/60                                    # synchronous speed in rad/s
        P_e=self.machine_rating/(1-self.S_Nmax)            #Air gap power
        
        # Calculating No-load voltage 
        self.E_p=om_s*self.N_s*k_wd1*self.r_s*self.l_s*self.B_g1*sqrt(2)
        
        I_r=P_e/m/self.E_p                                                 # rotor active current
        
        I_sm=self.E_p/(2*pi*freq*(self.L_s+self.L_sm)) # stator reactive current
        self.I_s=sqrt((I_r**2+I_sm**2))                                #Stator current
        I_srated=self.machine_rating/3/K_rs/self.E_p    #Rated current
        
        # Calculating winding current densities and specific current loading
        self.J_s=self.I_s/(self.A_Cuscalc)
        self.J_r=I_r/(self.A_Curcalc)
        self.A_1=2*m*self.N_s*self.I_s/pi/(2*self.r_s)
        
        self.Current_ratio=self.I_0/I_srated           # Ratio of magnetization current to rated current
        
        # Calculating masses of the electromagnetically active materials
        V_Cuss=m*l_Cus*A_Cus
        V_Cusr=m*L_cur*A_Cur
        V_Fest=(self.l_s*pi*((self.r_s+self.h_s)**2-self.r_s**2)-(2*m*self.q1*self.p*self.b_s*self.h_s*self.l_s))
        V_Fesy=self.l_s*pi*((self.r_s+self.h_s+self.h_ys)**2-(self.r_s+self.h_s)**2)
        V_Fert=pi*self.l_s*(r_r**2-(r_r-self.h_r)**2)-2*m*q2*self.p*self.b_r*self.h_r*self.l_s
        V_Fery=self.l_s*pi*((r_r-self.h_r)**2-(r_r-self.h_r-self.h_yr)**2)
        self.Copper=(V_Cuss+V_Cusr)*self.rho_Copper
        M_Fest=V_Fest*self.rho_Fe
        M_Fesy=V_Fesy*self.rho_Fe
        M_Fert=V_Fert*self.rho_Fe
        M_Fery=V_Fery*self.rho_Fe
        self.Iron=M_Fest+M_Fesy+M_Fert+M_Fery
        M_gen=(self.Copper)+(self.Iron)
        #K_gen=self.Cu*self.C_Cu+(self.Iron)*self.C_Fe #%M_pm*K_pm;
        L_tot=self.l_s
        self.Structural_mass=0.0002*M_gen**2+0.6457*M_gen+645.24
        self.Mass=M_gen+self.Structural_mass
        
        # Calculating Losses and efficiency 
        # 1. Copper losses
        
        K_R=1.2 # skin effect correction coefficient
        
        P_Cuss=m*self.I_s**2*self.R_s*K_R                         # Copper loss-stator
        P_Cusr=m*I_r**2*self.R_R                                                            # Copper loss-rotor
        P_Cusnom=P_Cuss+P_Cusr                                                                #  Copper loss-total
        
        # Iron Losses ( from Hysteresis and eddy currents)      
        P_Hyys=M_Fesy*(self.B_symax/1.5)**2*(P_Fe0h*om_e/(2*pi*60))                                        # Hysteresis losses in stator yoke
        P_Ftys=M_Fesy*(self.B_symax/1.5)**2*(P_Fe0e*(om_e/(2*pi*60))**2)                             # Eddy losses in stator yoke
        P_Hyd=M_Fest*(self.B_tsmax/1.5)**2*(P_Fe0h*om_e/(2*pi*60))                                        # Hysteresis losses in stator teeth
        P_Ftd=M_Fest*(self.B_tsmax/1.5)**2*(P_Fe0e*(om_e/(2*pi*60))**2)                                # Eddy losses in stator teeth
        P_Hyyr=M_Fery*(self.B_rymax/1.5)**2*(P_Fe0h*abs(self.S_Nmax)*om_e/(2*pi*60)) # Hysteresis losses in rotor yoke
        P_Ftyr=M_Fery*(self.B_rymax/1.5)**2*(P_Fe0e*(abs(self.S_Nmax)*om_e/(2*pi*60))**2) #Eddy losses in rotor yoke
        P_Hydr=M_Fert*(self.B_trmax/1.5)**2*(P_Fe0h*abs(self.S_Nmax)*om_e/(2*pi*60))    # Hysteresis losses in rotor teeth
        P_Ftdr=M_Fert*(self.B_trmax/1.5)**2*(P_Fe0e*(abs(self.S_Nmax)*om_e/(2*pi*60))**2) # Eddy losses in rotor teeth
        P_add=0.5*self.machine_rating/100                                                                                                    # additional losses
        P_Fesnom=P_Hyys+P_Ftys+P_Hyd+P_Ftd+P_Hyyr+P_Ftyr+P_Hydr+P_Ftdr                             # Total iron loss
        delta_v=1                                                                                                                                        # allowable brush voltage drop
        p_b=3*delta_v*I_r                                                                                                                        # Brush loss
        
        self.Losses=P_Cusnom+P_Fesnom+p_b+P_add
        self.gen_eff=(P_e-self.Losses)*100/P_e
        self.Overall_eff=self.gen_eff*self.Gearbox_efficiency
        
        # Calculating stator winding current density
        self.J_s=self.I_s/self.A_Cuscalc
        
        # Calculating  electromagnetic torque
        T_e=self.p *(self.machine_rating*1.01)/(2*pi*freq*(1-self.S_Nmax))
        
        # Calculating for tangential stress constraints
        self.TC1=T_e/(2*pi*sigma)
        self.TC2=self.r_s**2*self.l_s
    
        # Calculating mass moments of inertia and center of mass
        r_out=d_se*0.5
        self.I = [0.0] * 3
        self.I[0]   = (0.5*self.Mass*r_out**2)
        self.I[1]   = (0.25*self.Mass*r_out**2+(1/12)*self.Mass*self.l_s**2) 
        self.I[2]   = self.I[1]
        
        self.cm = [0.0] * 3
        self.cm[0]  = self.highSpeedSide_cm[0] + self.highSpeedSide_length/2. + self.l_s/2.
        self.cm[1]  = self.highSpeedSide_cm[1]
        self.cm[2]  = self.highSpeedSide_cm[2]
        
        # Material cost as a function of material mass and specific cost of material
        self.C_Cu = C_Cu
        self.C_Fe = C_Fe
        self.C_Fes = C_Fes
        
        K_gen=self.Copper*self.C_Cu+(self.Iron)*self.C_Fe #%M_pm*K_pm; #         
        Cost_str=self.C_Fes*self.Structural_mass
        self.Costs=K_gen+Cost_str  
        
        
        # outputs
        outputs['tau_p'] = self.tau_p
        outputs['p'] = self.p
        outputs['B_g'] = self.B_g
        outputs['q1'] = self.q1
        outputs['h_ys'] = self.h_ys
        outputs['h_yr'] = self.h_yr
        outputs['B_g'] = self.B_g
        outputs['B_g1'] = self.B_g1
        outputs['B_rymax'] = self.B_rymax
        outputs['B_tsmax'] = self.B_tsmax
        outputs['B_trmax'] = self.B_trmax
        outputs['S'] = self.S
        outputs['Q_r'] = self.Q_r
        outputs['N_s'] = self.N_s
        outputs['N_r'] = self.N_r
        #outputs['M_actual'] = self.M_actual
        outputs['p'] = self.p
        outputs['f'] = self.f
        outputs['E_p'] = self.E_p
        outputs['I_s'] = self.I_s
        outputs['b_s'] = self.b_s
        outputs['b_r'] = self.b_r
        outputs['b_t'] = self.b_t
        outputs['b_trmin'] = self.b_trmin
        outputs['b_tr'] = self.b_tr
        outputs['gen_eff'] = self.gen_eff
        outputs['Structural_mass'] = self.Structural_mass
        #outputs['Active'] = self.Active
        outputs['TC1'] = self.TC1
        outputs['TC2'] = self.TC2
        outputs['A_1'] = self.A_1
        outputs['J_s'] = self.J_s
        outputs['J_r'] = self.J_r
        outputs['K_rad'] = self.K_rad
        outputs['D_ratio'] = self.D_ratio
        outputs['A_Cuscalc'] = self.A_Cuscalc
        outputs['A_Curcalc'] = self.A_Curcalc
        outputs['Current_ratio'] = self.Current_ratio
        outputs['Slot_aspect_ratio1'] = self.Slot_aspect_ratio1
        outputs['Slot_aspect_ratio2'] = self.Slot_aspect_ratio2
        outputs['K_rad'] = self.K_rad
        outputs['gen_eff'] = self.gen_eff
        outputs['Overall_eff'] = self.Overall_eff
        outputs['R_s'] = self.R_s
        outputs['L_sm'] = self.L_sm
        outputs['R_R'] = self.R_R
        outputs['L_r'] = self.L_r
        outputs['L_s'] = self.L_s
        outputs['Copper'] = self.Copper
        outputs['Iron'] = self.Iron
        outputs['Losses'] = self.Losses
        outputs['Mass'] = self.Mass
        outputs['cm'] = self.cm
        outputs['I'] = self.I
        outputs['Costs'] =   self.Costs


#############################################################################
##############################  DFIG Optimizer ##############################
############################################################################# 
class DFIG_Optimizer(Group):
    def setup(self):
        i = self.add_subsystem('i', IndepVarComp(), promotes=['*'])
        i.add_output('r_s', desc = 'airgap radius r_s')
        i.add_output('l_s', desc = 'Stator core length l_s')
        i.add_output('h_s', desc = 'Yoke height h_s')
        i.add_output('h_r', desc = 'Stator slot height ')
        i.add_output('B_symax', desc = 'Peak Yoke flux density B_ymax')
        i.add_output('machine_rating', desc = 'Machine rating')
        i.add_output('n_nom', desc = 'rated speed ')
        i.add_output('highSpeedSide_cm', desc = ' high speed sidde COM [x, y, z]', val=[0.0, 0.0, 0.0])
        i.add_output('highSpeedSide_length', desc = 'high speed side length', val=0.0)
        i.add_output('Gearbox_efficiency', desc = 'Gearbox efficiency')
        i.add_output('S_Nmax', desc = 'Stator slot height ')
        i.add_output('I_0', desc = 'Rotor current at no-load')
         
        self.add_subsystem('dfig', DFIG(), promotes_inputs=['*'])
        self.add_subsystem('obj', ExecComp('f = -1 * eff'))
        self.add_subsystem('c1', ExecComp('tc1_tc2 = tc1 - tc2'))
        
        self.connect('dfig.Overall_eff', 'obj.eff')
        self.connect('dfig.TC1', 'c1.tc1')
        self.connect('dfig.TC2', 'c1.tc2')
    
        
    def optimizer(self):
        prob = Problem()
        prob.model = DFIG_Optimizer()
       
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
        #prob.model.add_objective('dfig.Costs', ref0=0, ref=20000) 
        prob.model.add_objective('obj.f', ref0=0, ref=100.0) 
        
        # design variables
        prob.model.add_design_var('r_s', lower=0.2, upper=1.0, ref0=0.2, ref=1.0)
        prob.model.add_design_var('l_s', lower=0.4, upper=2.0, ref0=0.4, ref=2.0)
        prob.model.add_design_var('h_s', lower=0.045, upper=0.1, ref0=0.045, ref=0.1)
        prob.model.add_design_var('h_r', lower=0.045, upper=0.1, ref0=0.045, ref=0.1)
        prob.model.add_design_var('B_symax', lower=1.0, upper=2.0, ref0=1.0, ref=2.0)
        prob.model.add_design_var('S_Nmax', lower=-0.3, upper=-0.1, ref0=-0.3, ref=-0.1)
        prob.model.add_design_var('I_0', lower=5.0, upper=100.0, ref0=5.0, ref=100.0)
        
        # constraints - denormalized
        prob.model.add_constraint('dfig.Overall_eff', lower=93.0) #, ref0=93.0, ref=100.0)                            #constraint 1
        prob.model.add_constraint('dfig.E_p', lower=500.0, upper=5000.0) #, ref0=500.0, ref=5000.0)                                                              #constraint 2
        prob.model.add_constraint('c1.tc1_tc2', upper=0.0) #, ref0=-1.0, ref=1.0)                                                            #constraint 4
        prob.model.add_constraint('dfig.B_g', lower=0.7, upper=1.2) #, ref0=0.7, ref=1.2)                                                                    #constraint 5
        prob.model.add_constraint('dfig.B_rymax', upper=2.0) #, ref0=0.0, ref=2.0)                                                                #constraint 7
        prob.model.add_constraint('dfig.B_trmax', upper=2.0) #, ref0=0.0, ref=2.0)                                                                #constraint 8
        prob.model.add_constraint('dfig.B_tsmax', upper=2.0) #, ref0=0.0, ref=2.0)                                                             #constraint 9
        prob.model.add_constraint('dfig.A_1', upper=60000.0) #, ref0=0.0, ref=60000.0)                                                              #constraint 10
        prob.model.add_constraint('dfig.J_s', upper=6.0) #, ref0=0.0, ref=6.0)                                                                #constraint 11
        prob.model.add_constraint('dfig.J_r', upper=6.0) #, ref0=0.0, ref=6.0)                                                                      #constraint 12
        prob.model.add_constraint('dfig.K_rad', lower=0.2, upper=1.5) #, ref0=0.2, ref=1.5)                                                        #constraint 13 #boldea Chapter 3
        prob.model.add_constraint('dfig.D_ratio', lower=1.37, upper=1.4) #, ref0=1.37, ref=1.4)                                                #constraint 15 #boldea Chapter 3
        prob.model.add_constraint('dfig.Current_ratio', lower=0.1, upper=0.3) #, ref0=0.1, ref=0.3)                                                #constraint 17
        prob.model.add_constraint('dfig.Slot_aspect_ratio1', lower=4, upper=10) #, ref0=4, ref=10)                                        #constraint 19
         
        
#         # constraints - normalized
#         prob.model.add_constraint('dfig.Overall_eff', lower=93.0, ref0=0.0, ref=100.0)                            #constraint 1
#         prob.model.add_constraint('dfig.E_p', lower=500.0, upper=5000.0, ref0=0.0, ref=5000.0)                                                              #constraint 2
#         prob.model.add_constraint('c4.tc1_tc2', upper=0.0, ref0=-1.0, ref=1.0)                                                            #constraint 4
#         prob.model.add_constraint('dfig.B_g', lower=0.7, upper=1.2, ref0=0, ref=2.0)                                                                    #constraint 5
#         prob.model.add_constraint('dfig.B_rymax', upper=2.0, ref0=0.0, ref=5.0)                                                                #constraint 7
#         prob.model.add_constraint('dfig.B_trmax', upper=2.0, ref0=0.0, ref=5.0)                                                                #constraint 8
#         prob.model.add_constraint('dfig.B_tsmax', upper=2.0, ref0=0.0, ref=5.0)                                                             #constraint 9
#         prob.model.add_constraint('dfig.A_1', upper=60000.0, ref0=0.0, ref=200000.0)                                                              #constraint 10
#         prob.model.add_constraint('dfig.J_s', upper=6.0, ref0=0.0, ref=100.0)                                                                #constraint 11
#         prob.model.add_constraint('dfig.J_r', upper=6.0, ref0=0.0, ref=200.0)                                                                      #constraint 12
#         prob.model.add_constraint('dfig.K_rad', lower=0.2, upper=1.5, ref0=0.0, ref=2.0)                                                        #constraint 13 #boldea Chapter 3
#         prob.model.add_constraint('dfig.D_ratio', lower=1.37, upper=1.4, ref0=0.0, ref=2.0)                                                #constraint 15 #boldea Chapter 3
#         prob.model.add_constraint('dfig.Current_ratio', lower=0.1, upper=0.3, ref0=0.0, ref=1.0)                                                #constraint 17
#         prob.model.add_constraint('dfig.Slot_aspect_ratio1', lower=4, upper=10, ref0=0.0, ref=10.0)                                        #constraint 19        
        
        prob.setup(check=True, mode='fwd')
        #view_model(prob)
        
        return prob                


#############################################################################
#################################  Unit Test ################################
#############################################################################
def unit_test():
    inputs={'r_s' : 0.61, \
            'l_s' : 0.49, \
            'h_s' : 0.08, \
            'h_r' : 0.1, \
            'B_symax' : 1.3, \
            'machine_rating' : 5000000.0, \
            'highSpeedSide_cm' : [0.]*3, \
            'highSpeedSide_length' : 0., \
            'n_nom' : 1200, \
            'Gearbox_efficiency' : 0.955, \
            'S_Nmax' : -0.2, \
            'I_0' : 40}
    outputs={}
    DFIG().compute(inputs, outputs)  
    beautify_output(outputs)   
    
    
def optimization():
    prob = DFIG_Optimizer().optimizer()
    
    prob['r_s'] = 0.61
    prob['l_s'] = 0.49
    prob['h_s'] = 0.08
    prob['h_r'] = 0.1
    prob['B_symax'] = 1.3
    prob['machine_rating'] = 5000000.0
    prob['n_nom'] = 1200.0
    prob['highSpeedSide_cm'] = [0.]*3
    prob['highSpeedSide_length'] = 0.0
    prob['Gearbox_efficiency'] = 0.955
    prob['S_Nmax'] = -0.2
    prob['I_0'] = 40
    
    prob.run_model()
    #prob.set_solver_print(level=2)
    prob.model.approx_totals('fd')
    prob.run_driver()
    
    print('minimum found at')
    print(prob['dfig.r_s'])
    print(prob['dfig.l_s'])
    print(prob['dfig.h_s'])
    print(prob['dfig.h_r'])
    print(prob['dfig.B_symax'])
    print(prob['dfig.machine_rating'])
    print(prob['dfig.n_nom'])
    print(prob['dfig.highSpeedSide_cm'])
    print(prob['dfig.highSpeedSide_length'])
    print(prob['dfig.Gearbox_efficiency'])
    print(prob['dfig.S_Nmax'])
    print(prob['dfig.I_0'])
    print '-'*5
    
    print 'gen_eff', prob['dfig.gen_eff']
    print 'Mass', prob['dfig.Mass']
    print 'Costs', prob['dfig.Costs']
    print 'Frequency', prob['dfig.f']
    print '-'*5
    
    print 'Eff>93', prob['dfig.Overall_eff']
    print '500<E_p<5000', prob['dfig.E_p']
    print 'TC1-TC2<0', prob['c1.tc1_tc2']
    print '0.7<B_g<1.2', prob['dfig.B_g']
    print 'B_rymax<2', prob['dfig.B_rymax']
    print 'B_trmax<2', prob['dfig.B_trmax']
    print 'B_tsmax<2', prob['dfig.B_tsmax']
    print 'A_1<60000', prob['dfig.A_1']
    print 'J_s<6', prob['dfig.J_s']
    print 'J_r<6', prob['dfig.J_r']
    print '0.2<K_rad<1.5', prob['dfig.K_rad']
    print '1.37<D_ratio<1.4', prob['dfig.D_ratio']
    print '0.1<Current_ratio<0.3', prob['dfig.Current_ratio']
    print '4<Slot_aspect_ratio1<10', prob['dfig.Slot_aspect_ratio1']






        
if __name__ == "__main__":
    #unit_test
    optimization()
