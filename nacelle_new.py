import numpy as np
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil
import algopy
import scipy as scp
import scipy.optimize as opt
from scipy import integrate
from time import time

from openmdao.api import ExplicitComponent, Group, IndepVarComp
from openmdao.solvers.nonlinear.nonlinear_block_gs import NonlinearBlockGS
from drivese_utils import resize_for_bearings, fx, gx
from fixed_parameters import bearing_types, g



class LowSpeedShaft(ExplicitComponent):
    ''' LowSpeedShaft class
          The LowSpeedShaft class is used to represent the low speed shaft component of a wind turbine drivetrain. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    def setup(self):
        # inputs
        self.add_input('machine_rating', units='kW', desc='machine_rating machine rating of the turbine')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')        
        self.add_input('rotor_bending_moment', units='N*m', desc='bending moment vector', shape=3)
        self.add_input('rotor_force', units='N', desc='force vector applied at hub center', shape=3)
        self.add_input('overhang', units='m', desc='Overhang distance')
        self.add_input('shaft_angle', units='rad', desc='Angle of the LSS inclindation with respect to the horizontal')
        self.add_input('rotor_mass', units='kg', desc='rotor mass')
        self.add_input('gearbox_mass', units='kg', desc='Gearbox mass')
        self.add_input('carrier_mass', units='kg', desc='Carrier mass')
        self.add_input('shrink_disc_mass', units='kg', desc='Mass of the shrink disc')
        self.add_input('gearbox_cm', units = 'm', desc = 'center of mass of gearbox', shape=3)
        self.add_input('gearbox_length', units='m', desc='gearbox length')
        self.add_input('mb1_type', desc ='main bearing type')
       
        # outputs
        self.add_output('length', units='m', desc='lss length')
        self.add_output('diameter1', units='m', desc='lss outer diameter at main bearing')
        self.add_output('diameter2', units='m', desc='lss outer diameter at second bearing')
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)
        self.add_output('FW_mb', units='m', desc='facewidth of main bearing')    
        self.add_output('bearing_mass1', units = 'kg', desc='main bearing mass')
        self.add_output('bearing_mass2', units = 'kg', desc='main bearing mass') #zero for 3-pt model
        self.add_output('bearing_location1', units = 'm', desc = 'main bearing 1 center of mass', shape=3)
        self.add_output('bearing_location2', units = 'm', desc = 'main bearing 2 center of mass', shape=3)

    def compute(self, inputs, outputs):
        machine_rating = inputs['machine_rating']
        rotor_diameter = inputs['rotor_diameter']
        rotor_bending_moment = inputs['rotor_bending_moment']
        rotor_force = inputs['rotor_force']
        overhang = inputs['overhang']
        shaft_angle = inputs['shaft_angle']
        rotor_mass = inputs['rotor_mass']
        gearbox_mass = inputs['gearbox_mass']
        carrier_mass = inputs['carrier_mass'] # http://www.ntnglobal.com/en/products/review/pdf/NTN_TR76_en_p113_120.pdf
        shrink_disc_mass = inputs['shrink_disc_mass'] # http://www.ringfeder.com/international/applications/energies/renewable/wind-turbine---shrink-discs-rfn-4051/wind-turbine---shrink-discs-rfn-4051/
        gearbox_cm = inputs['gearbox_cm']
        gearbox_length = inputs['gearbox_length']
        mb1Type = bearing_types[int(inputs['mb1_type'])]
        
        L_rb = inputs['L_rb']
        flange_length = inputs['flange_length']
        shaft_ratio = inputs['shaft_ratio']
        #mb1Type = 'SRB'
        mb2Type = 'TRB2'
        
        flange_length = 0.3*(rotor_diameter/100.0)**2.0 - 0.1 * (rotor_diameter / 100.0) + 0.4
        L_rb = 0.007835*rotor_diameter+0.9642
        
        rotor_bending_moment_x = rotor_bending_moment[0]
        rotor_bending_moment_y = rotor_bending_moment[1]
        rotor_bending_moment_z = rotor_bending_moment[2] if rotor_bending_moment[2] > 0 else 53.846*rotor_mass*L_rb
        rotor_force_x = rotor_force[0]
        rotor_force_y = rotor_force[1]
        rotor_force_z = rotor_force[2]
        
        density = 7850.0


        L_ms_new = 0.0
        L_ms_0=0.5 # main shaft length downwind of main bearing
        L_ms=L_ms_0
        tol=1e-4 
        check_limit = 1.0
        dL=0.05
        D_max = 1.0
        D_min = 0.2

        T=rotor_bending_moment_x/1000.0
        

        #Main bearing defelection check
        if mb1Type == 'TRB1' or 'TRB2':
            Bearing_Limit = 3.0/60.0/180.0*pi
        elif mb1Type == 'CRB':
            Bearing_Limit = 4.0/60.0/180.0*pi
        elif mb1Type == 'SRB' or 'RB':
            Bearing_Limit = 0.078
        elif mb1Type == 'RB':
            Bearing_Limit = 0.002
        elif mb1Type == 'CARB':
            Bearing_Limit = 0.5/180*pi
        else:
            Bearing_Limit = False
        
        n_safety_brg = 1.0
        n_safety=2.5
        Sy = 66000#*S_ut/700e6 #psi
        E=2.1e11  
        N_count=50    
          
        u_knm_inlb = 8850.745454036
        u_in_m = 0.0254000508001
        counter=0
        length_max = overhang - L_rb + (gearbox_cm[0] -gearbox_length/2.) #modified length limit 7/29

        while abs(check_limit) > tol and L_ms_new < length_max:
            counter =counter+1
            if L_ms_new > 0:
                 L_ms=L_ms_new
            else:
                  L_ms=L_ms_0

            #-----------------------
            # size_LSS_3pt
            #Distances
            L_bg = 6.11 *(machine_rating/5.0e3)         #distance from hub center to gearbox yokes
            L_as = L_ms/2.0     #distance from main bearing to shaft center
            H_gb = 1.0          #distance to gbx center from trunnions in z-dir     
            L_gp = 0.825        #distance from gbx coupling to gbx trunnions
            L_cu = L_ms + 0.5
            L_cd = L_cu + 0.5
            L_gb=0
            
            #Weight properties
            weightRotor=rotor_mass*g
            massLSS = pi/3*(D_max**2.0 + D_min**2.0 + D_max*D_min)*L_ms*density/4.0
            weightLSS = massLSS*g       #LSS weight
            weightShrinkDisc = shrink_disc_mass*g                #shrink disc weight
            weightGbx = gearbox_mass*g                              #gearbox weight
            weightCarrier = carrier_mass*g
            
            len_pts=101;
            x_ms = np.linspace(L_rb, L_ms+L_rb, len_pts)
            x_rb = np.linspace(0.0, L_rb, len_pts)
            y_gp = np.linspace(0, L_gp, len_pts)
            
            F_mb_x = -rotor_force_x - weightRotor*sin(shaft_angle)
            F_mb_y = rotor_bending_moment_z/L_bg - rotor_force_y*(L_bg + L_rb)/L_bg
            F_mb_z = (-rotor_bending_moment_y + weightRotor*(cos(shaft_angle)*(L_rb + L_bg)\
                        + sin(shaft_angle)*H_gb) + weightLSS*(L_bg - L_as)\
                        * cos(shaft_angle) + weightShrinkDisc*cos(shaft_angle)\
                        *(L_bg - L_ms) - weightGbx*cos(shaft_angle)*L_gb - rotor_force_z*cos(shaft_angle)*(L_bg + L_rb))/L_bg
            
            F_gb_x = -(weightLSS + weightShrinkDisc + weightGbx)*sin(shaft_angle)
            F_gb_y = -F_mb_y - rotor_force_y
            F_gb_z = -F_mb_z + (weightLSS + weightShrinkDisc + weightGbx + weightRotor)*cos(shaft_angle) - rotor_force_z
            
            #carrier bearing loads
            F_cu_z = (weightLSS*cos(shaft_angle) + weightShrinkDisc*cos(shaft_angle) + weightGbx*cos(shaft_angle)) - F_mb_z - rotor_force_z- \
            (-rotor_bending_moment_y - rotor_force_z*cos(shaft_angle)*L_rb + weightLSS*(L_bg - L_as)*cos(shaft_angle) - weightCarrier*cos(shaft_angle)*L_gb)/(1 - L_cu/L_cd)
            
            F_cd_z = (weightLSS*cos(shaft_angle) + weightShrinkDisc*cos(shaft_angle) + weightGbx*cos(shaft_angle)) - F_mb_z - rotor_force_z - F_cu_z 
            
            
            My_ms = np.zeros(2*len_pts)
            Mz_ms = np.zeros(2*len_pts)
            
            for k in range(len_pts):
                My_ms[k] = -rotor_bending_moment_y + weightRotor*cos(shaft_angle)*x_rb[k] + 0.5*weightLSS/L_ms*x_rb[k]**2 - rotor_force_z*x_rb[k]
                Mz_ms[k] = -rotor_bending_moment_z - rotor_force_y*x_rb[k]
            
            for j in range(len_pts):
                My_ms[j+len_pts] = -rotor_force_z*x_ms[j] - rotor_bending_moment_y + weightRotor*cos(shaft_angle)*x_ms[j] - F_mb_z*(x_ms[j]-L_rb) + 0.5*weightLSS/L_ms*x_ms[j]**2
                Mz_ms[j+len_pts] = -rotor_bending_moment_z - F_mb_y*(x_ms[j]-L_rb) - rotor_force_y*x_ms[j]
            
            x_shaft = np.concatenate([x_rb, x_ms])
            
            MM_max=np.amax((My_ms**2 + Mz_ms**2)**0.5/1000.0)
            Index=np.argmax((My_ms**2 + Mz_ms**2)**0.5/1000.0)                
            
            MM_min = ((My_ms[-1]**2+Mz_ms[-1]**2)**0.5/1000.0)
            
            MM=MM_max
            D_max=(16.0*n_safety/pi/Sy*(4.0*(MM*u_knm_inlb)**2 + 3.0*(rotor_bending_moment_x/1000.0*u_knm_inlb)**2)**0.5)**(1.0/3.0)*u_in_m
            
            #OD at end
            MM=MM_min
            D_min=(16.0*n_safety/pi/Sy*(4.0*(MM*u_knm_inlb)**2 + 3.0*(rotor_bending_moment_x/1000.0*u_knm_inlb)**2)**0.5)**(1.0/3.0)*u_in_m
            
            #Estimate ID
            D_in=shaft_ratio*D_max
            D_max=(D_in**4.0 + D_max**4.0)**0.25
            D_min=(D_in**4.0 + D_min**4.0)**0.25            
            
            weightLSS_new = (density*pi/12.0*L_ms*(D_max**2.0 + D_min**2.0 + D_max*D_min) - density*pi/4.0*D_in**2.0*L_ms + \
                              density*pi/4.0*D_max**2*L_rb)*g
            massLSS_new = weightLSS_new/g
            
            D1 = fx(rotor_force_z,weightRotor,shaft_angle,rotor_bending_moment_y,F_mb_z,L_rb,weightLSS_new,L_ms,L_rb+L_ms)
            D2 = fx(rotor_force_z,weightRotor,shaft_angle,rotor_bending_moment_y,F_mb_z,L_rb,weightLSS_new,L_ms,L_rb)
            C1 = -(D1-D2)/L_ms;
            C2 = -D2-C1*(L_rb);
            
            I_2=pi/64.0*(D_max**4 - D_in**4)
            theta_y = np.zeros(len_pts)
            d_y = np.zeros(len_pts)
            
            for kk in range(len_pts):
                theta_y[kk]=gx(rotor_force_z,weightRotor,shaft_angle,rotor_bending_moment_y,F_mb_z,L_rb,weightLSS_new,L_ms,C1,x_ms[kk])/E/I_2
                d_y[kk]=(fx(rotor_force_z,weightRotor,shaft_angle,rotor_bending_moment_y,F_mb_z,L_rb,weightLSS_new,L_ms,x_ms[kk])+C1*x_ms[kk]+C2)/E/I_2

  
            #-----------------------

            check_limit = abs(abs(theta_y[-1])-Bearing_Limit/n_safety_brg)
            L_ms_new = L_ms + dL        

        [D_max_a,FW_max,bearingmass] = resize_for_bearings(D_max,  mb1Type,False)
        [D_min_a,FW_min,trash] = resize_for_bearings(D_min,  mb2Type,False) #mb2 is a representation of the gearbox connection
        
        #D_in = 1.68
            
        lss_mass_new=(pi/3)*(D_max_a**2+D_min_a**2+D_max_a*D_min_a)*(L_ms-(FW_max+FW_min)/2)*density/4+ \
                         (pi/4)*(D_max_a**2-D_in**2)*density*FW_max+\
                         (pi/4)*(D_min_a**2-D_in**2)*density*FW_min-\
                         (pi/4)*(D_in**2)*density*(L_ms+(FW_max+FW_min)/2)               
        lss_mass_new *= 1.35 # add flange and shrink disk mass
        length=L_ms_new + (FW_max+FW_min)/2 + flange_length
        outputs['length'] = length
        #print ("L_ms: {0}").format(L_ms)
        #print ("LSS length, m: {0}").format(length)
        D_outer=D_max
        #print ("Upwind MB OD, m: {0}").format(D_max_a)
        #print ("CB OD, m: {0}").format(D_min_a)
        #print ("D_min: {0}").format(D_min)
        #D_in=D_in
        mass=lss_mass_new
        outputs['diameter1']= D_max_a
        outputs['diameter2']= D_min_a 
        #length=L_ms
        #print length
        D_outer=D_max_a
        diameter=D_max_a

         # calculate mass properties
        downwind_location = np.array([gearbox_cm[0]-gearbox_length/2. , gearbox_cm[1] , gearbox_cm[2] ])

        bearing_location1 = np.array([0.,0.,0.]) #upwind
        bearing_location1[0] = downwind_location[0] - L_ms*cos(shaft_angle)
        bearing_location1[1] = downwind_location[1]
        bearing_location1[2] = downwind_location[2] + L_ms*sin(shaft_angle)
        outputs['bearing_location1'] = bearing_location1

        outputs['bearing_location2'] = np.array([0.,0.,0.]) #downwind does not exist

        cm = np.array([0.0,0.0,0.0])
        cm[0] = downwind_location[0] - 0.65*length*cos(shaft_angle) #From solid models, center of mass with flange (not including shrink disk) very nearly .65*total_length
        cm[1] = downwind_location[1]
        cm[2] = downwind_location[2] + 0.65*length*sin(shaft_angle)

        #including shrink disk mass
        cm[0] = (cm[0]*mass + downwind_location[0]*shrink_disc_mass) / (mass+shrink_disc_mass) 
        cm[1] = cm[1]
        cm[2] = (cm[2]*mass + downwind_location[2]*shrink_disc_mass) / (mass+shrink_disc_mass)
        outputs['cm'] = cm
        # print 'shaft before shrink disk:', mass
        mass+=shrink_disc_mass
        outputs['mass'] = mass

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = mass * (D_in ** 2.0 + D_outer ** 2.0) / 8.0
        I[1]  = mass * (D_in ** 2.0 + D_outer ** 2.0 + (4.0 / 3.0) * (length ** 2.0)) / 16.0
        I[2]  = I[1]
        outputs['I'] = I

        # print 'L_rb %8.f' %(L_rb) #*(machine_rating/5.0e3)   #distance from hub center to main bearing scaled off NREL 5MW
        # print 'L_bg %8.f' %(L_bg) #*(machine_rating/5.0e3)         #distance from hub center to gearbox yokes
        # print 'L_as %8.f' %(L_as) #distance from main bearing to shaft center
      
        outputs['FW_mb']=FW_max
        outputs['bearing_mass1'] = bearingmass
        outputs['bearing_mass2'] = 0.




class MainBearing_drive(ExplicitComponent): 
    ''' MainBearings class          
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    def setup(self):
        # variables
        self.add_input('bearing_type', desc='Main bearing type: CARB, TRB1 or SRB')
        self.add_input('bearing_mass', units = 'kg', desc = 'bearing mass from LSS model')
        self.add_input('lss_diameter', units='m', desc='lss outer diameter at main bearing')
        self.add_input('lss_design_torque', units='N*m', desc='lss design torque')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('location', units = 'm', desc = 'x,y,z location from shaft model', shape=3)
    
        # returns
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=(3,1))
 
    def compute(self, inputs, outputs):
        
        bearing_type = bearing_types[int(inputs['bearing_type'])]
        bearing_mass = inputs['bearing_mass']
        lss_diameter = inputs['lss_diameter']
        lss_design_torque = inputs['lss_design_torque']
        rotor_diameter = inputs['rotor_diameter']
        location = inputs['location']

        #super(MainBearing_drive, self).compute(inputs, outputs)
        mass = bearing_mass
        mass += mass*(8000.0/2700.0) #add housing weight
        
        # calculate mass properties
        depth = lss_diameter * 1.5

        if location[0] != 0.0:
            cm = location

        else:
            cmMB = np.array([0.0,0.0,0.0])
            cmMB = ([- (0.035 * rotor_diameter), 0.0, 0.025 * rotor_diameter])
            cm = cmMB
            
        outputs['cm'] = cm    
        outputs['mass'] = mass
        b1I0 = (mass * lss_diameter ** 2 ) / 4.0 
        outputs['I'] = np.array([b1I0, b1I0 / 2.0, b1I0 / 2.0])
        
        
        
class SecondBearing_drive(ExplicitComponent): 
    ''' MainBearings class          
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    def setup(self):
        # variables
        #self.add_input('bearing_type', desc='Main bearing type: CARB, TRB1 or SRB')
        self.add_input('bearing_mass', units = 'kg', desc = 'bearing mass from LSS model')
        self.add_input('lss_diameter', units='m', desc='lss outer diameter at main bearing')
        self.add_input('lss_design_torque', units='N*m', desc='lss design torque')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('location', units = 'm', desc = 'x,y,z location from shaft model', shape=3)
    
        # returns
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=(3,1))
 
    
    def compute(self, inputs, outputs):

        bearing_type = 'TRB2' #inputs['bearing_type']
        bearing_mass = inputs['bearing_mass']
        lss_diameter = inputs['lss_diameter']
        lss_design_torque = inputs['lss_design_torque']
        rotor_diameter = inputs['rotor_diameter']
        location = inputs['location']
        
        mass = bearing_mass
        mass += mass*(8000.0/2700.0) #add housing weight

        # calculate mass properties
        depth = (lss_diameter * 1.5)

        if mass > 0 and location[0] != 0.0:
            cm = location
        else:
            cm = np.array([0,0,0])
            mass = 0.
            
        outputs['mass'] = mass
        outputs['cm'] = cm       

        b2I0  = (mass * lss_diameter ** 2 ) / 4.0 
        outputs['I'] = ([b2I0, b2I0 / 2.0, b2I0 / 2.0])      
        
        
        
        
class AboveYawMassAdder_drive(ExplicitComponent):

    def setup(self):
        # variables
        self.add_input('machine_rating', units='kW', desc='machine rating')
        self.add_input('lss_mass', units='kg', desc='component mass')
        self.add_input('main_bearing_mass', units='kg', desc='component mass')
        self.add_input('second_bearing_mass', units='kg', desc='component mass')
        self.add_input('gearbox_mass', units='kg', desc='component mass')
        self.add_input('hss_mass', units='kg', desc='component mass')
        self.add_input('generator_mass', units='kg', desc='component mass')
        self.add_input('bedplate_mass', units='kg', desc='component mass')
        self.add_input('bedplate_length', units='m', desc='component length')
        self.add_input('bedplate_width', units='m', desc='component width')
        #self.add_input('transformer_mass', units='kg', desc='component mass')
    
        # parameters
        self.add_input('crane', desc='flag for presence of crane', val=1)
    
        # returns
        self.add_output('electrical_mass', units='kg', desc='component mass')
        self.add_output('vs_electronics_mass', units='kg', desc='component mass')
        self.add_output('hvac_mass', units='kg', desc='component mass')
        self.add_output('controls_mass', units='kg', desc='component mass')
        self.add_output('platforms_mass', units='kg', desc='component mass')
        self.add_output('crane_mass', units='kg', desc='component mass')
        self.add_output('mainframe_mass', units='kg', desc='component mass')
        self.add_output('cover_mass', units='kg', desc='component mass')
        self.add_output('above_yaw_mass', units='kg', desc='total mass above yaw system')
        self.add_output('length', units='m', desc='component length')
        self.add_output('width', units='m', desc='component width')
        self.add_output('height', units='m', desc='component height')

    def compute(self, inputs, outputs):

        machine_rating = inputs['machine_rating']
        lss_mass = inputs['lss_mass']
        main_bearing_mass = inputs['main_bearing_mass']
        second_bearing_mass = inputs['second_bearing_mass']
        gearbox_mass = inputs['gearbox_mass']
        hss_mass = inputs['hss_mass']
        generator_mass = inputs['generator_mass']
        bedplate_mass = inputs['bedplate_mass']
        bedplate_length = inputs['bedplate_length']
        bedplate_width = inputs['bedplate_width']
        transformer_mass = 0 #inputs['transformer_mass']
        crane = inputs['crane']

        # electronic systems, hydraulics and controls
        outputs['electrical_mass'] = 0.0        
        outputs['vs_electronics_mass'] = 0 #2.4445*self.machine_rating + 1599.0 accounted for in transformer calcs
        outputs['hvac_mass'] = 0.08 * machine_rating
        outputs['controls_mass']     = 0.0
        
        # mainframe system including bedplate, platforms, crane and miscellaneous hardware
        outputs['platforms_mass'] = 0.125 * bedplate_mass
        
        if (crane):
            outputs['crane_mass'] =  3000.0
        else:
            outputs['crane_mass'] = 0.0
        
        outputs['mainframe_mass']  = bedplate_mass + outputs['crane_mass'] + outputs['platforms_mass']
        
        nacelleCovArea      = 2 * (bedplate_length ** 2)              # this calculation is based on Sunderland
        outputs['cover_mass'] = (84.1 * nacelleCovArea) / 2          # this calculation is based on Sunderland - divided by 2 in order to approach CSM
        
        # yaw system weight calculations based on total system mass above yaw system
        outputs['above_yaw_mass'] =  lss_mass + \
                    main_bearing_mass + second_bearing_mass + \
                    gearbox_mass + \
                    hss_mass + \
                    generator_mass + \
                    outputs['mainframe_mass'] + \
                    transformer_mass + \
                    outputs['electrical_mass'] + \
                    outputs['vs_electronics_mass'] + \
                    outputs['hvac_mass'] + \
                    outputs['controls_mass'] + \
                    outputs['cover_mass']
        
        outputs['length']      = bedplate_length                              # nacelle length [m] based on bedplate length
        outputs['width']       = bedplate_width                        # nacelle width [m] based on bedplate width
        outputs['height']      = (2.0 / 3.0) * outputs['length']                         # nacelle height [m] calculated based on cladding area
        
        


class NacelleSystemAdder_drive(ExplicitComponent): #added to drive to include transformer
    ''' NacelleSystem class
          The Nacelle class is used to represent the overall nacelle of a wind turbine.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    
    def setup(self):
        # variables
        self.add_input('above_yaw_mass', units='kg', desc='mass above yaw system')
        self.add_input('yawMass', units='kg', desc='mass of yaw system')
        self.add_input('lss_mass', units='kg', desc='component mass')
        self.add_input('main_bearing_mass', units='kg', desc='component mass')
        self.add_input('second_bearing_mass', units='kg', desc='component mass')
        self.add_input('gearbox_mass', units='kg', desc='component mass')
        self.add_input('hss_mass', units='kg', desc='component mass')
        self.add_input('generator_mass', units='kg', desc='component mass')
        self.add_input('bedplate_mass', units='kg', desc='component mass')
        self.add_input('mainframe_mass', units='kg', desc='component mass')
        self.add_input('lss_cm', units='m', desc='component CM', shape=3)
        self.add_input('main_bearing_cm', units='m', desc='component CM', shape=3)
        self.add_input('second_bearing_cm', units='m', desc='component CM', shape=3)
        self.add_input('gearbox_cm', units='m', desc='component CM', shape=3)
        self.add_input('hss_cm', units='m', desc='component CM', shape=3)
        self.add_input('generator_cm', units='m', desc='component CM', shape=3)
        self.add_input('bedplate_cm', units='m', desc='component CM', shape=3)
        self.add_input('lss_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('main_bearing_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('second_bearing_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('gearbox_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('hss_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('generator_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('bedplate_I', units='kg*m**2', desc='component I', shape=3)
        #self.add_input('transformer_mass', units='kg', desc='component mass')
        #self.add_input('transformer_cm', units='kg', desc='component CM', val=np.array([0.0,0.0,0.0]))
        #self.add_input('transformer_I', units='kg', desc='component I', val=np.array([0.0,0.0,0.0]))
    
        # returns
        self.add_output('nacelle_mass', units='kg', desc='overall component mass')
        self.add_output('nacelle_cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('nacelle_I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=6)

    
    
    
    def compute(self, inputs, outputs):
        
        above_yaw_mass = inputs['above_yaw_mass']
        yawMass = inputs['yawMass']
        lss_mass = inputs['lss_mass']
        main_bearing_mass = inputs['main_bearing_mass']
        second_bearing_mass = inputs['second_bearing_mass']
        gearbox_mass = inputs['gearbox_mass']
        hss_mass = inputs['hss_mass']
        generator_mass = inputs['generator_mass']
        bedplate_mass = inputs['bedplate_mass']
        mainframe_mass = inputs['mainframe_mass']
        lss_cm = inputs['lss_cm']
        main_bearing_cm = inputs['main_bearing_cm']
        second_bearing_cm = inputs['second_bearing_cm']
        gearbox_cm = inputs['gearbox_cm']
        hss_cm = inputs['hss_cm']
        generator_cm = inputs['generator_cm']
        bedplate_cm = inputs['bedplate_cm']
        lss_I = inputs['lss_I']
        main_bearing_I = inputs['main_bearing_I']
        second_bearing_I = inputs['second_bearing_I']
        gearbox_I = inputs['gearbox_I']
        hss_I = inputs['hss_I']
        generator_I = inputs['generator_I']
        bedplate_I = inputs['bedplate_I']
        transformer_mass = 0.0 #inputs['transformer_mass']
        transformer_cm = np.array([0.0,0.0,0.0]) #inputs['transformer_cm']
        transformer_I = np.array([0.0,0.0,0.0]) #inputs['transformer_I']

        outputs['nacelle_mass'] = (above_yaw_mass + yawMass)
        
        # calculation of mass center and moments of inertia
        cm = np.array([0.0,0.0,0.0])
        for i in (range(0,3)):
            # calculate center of mass (use mainframe_mass in place of bedplate_mass - assume lumped around bedplate_cm)
            cm[i] = (lss_mass * lss_cm[i] + transformer_cm[i] * transformer_mass + \
                    main_bearing_mass * main_bearing_cm[i] + second_bearing_mass * second_bearing_cm[i] + \
                    gearbox_mass * gearbox_cm[i] + hss_mass * hss_cm[i] + \
                    generator_mass * generator_cm[i] + mainframe_mass * bedplate_cm[i] ) / \
                    (lss_mass + main_bearing_mass + second_bearing_mass + \
                    gearbox_mass + hss_mass + generator_mass + mainframe_mass)
        outputs['nacelle_cm'] = cm
        
        I = np.zeros(6)
        for i in (range(0,3)):                        # calculating MOI, at nacelle center of gravity with origin at tower top center / yaw mass center, ignoring masses of non-drivetrain components / auxiliary systems
            # calculate moments around CM
            # sum moments around each components CM (adjust for mass of mainframe) # TODO: add yaw MMI
            I[i]  =  lss_I[i] + main_bearing_I[i] + second_bearing_I[i] + gearbox_I[i] + transformer_I[i] +\
                          hss_I[i] + generator_I[i] + bedplate_I[i] * (mainframe_mass / bedplate_mass)
            # translate to nacelle CM using parallel axis theorem (use mass of mainframe en lieu of bedplate to account for auxiliary equipment)
            for j in (range(0,3)):
                if i != j:
                    I[i] +=  lss_mass * (lss_cm[j] - cm[j]) ** 2 + \
                                  main_bearing_mass * (main_bearing_cm[j] - cm[j]) ** 2 + \
                                  second_bearing_mass * (second_bearing_cm[j] - cm[j]) ** 2 + \
                                  gearbox_mass * (gearbox_cm[j] - cm[j]) ** 2 + \
                                  transformer_mass * (transformer_cm[j] - cm[j]) ** 2 + \
                                  hss_mass * (hss_cm[j] - cm[j]) ** 2 + \
                                  generator_mass * (generator_cm[j] - cm[j]) ** 2 + \
                                  mainframe_mass * (bedplate_cm[j] - cm[j]) ** 2
        outputs['nacelle_I'] = I













#-------------------------------------------------------------------------------



class MainBearing_drive(ExplicitComponent): 
    ''' MainBearings class          
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    def setup(self):
        # variables
        #self.add_input('bearing_type', desc='Main bearing type: CARB, TRB1 or SRB')
        self.add_input('bearing_mass', units = 'kg', desc = 'bearing mass from LSS model')
        self.add_input('lss_diameter', units='m', desc='lss outer diameter at main bearing')
        self.add_input('lss_design_torque', units='N*m', desc='lss design torque')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('location', units = 'm', desc = 'x,y,z location from shaft model', shape=3)
    
        # returns
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=(3,1))
 
    def compute(self, inputs, outputs):
        
        bearing_type = 'SRB' #inputs['bearing_type']
        bearing_mass = inputs['bearing_mass']
        lss_diameter = inputs['lss_diameter']
        lss_design_torque = inputs['lss_design_torque']
        rotor_diameter = inputs['rotor_diameter']
        location = inputs['location']

        #super(MainBearing_drive, self).compute(inputs, outputs)
        mass = bearing_mass
        mass += mass*(8000.0/2700.0) #add housing weight
        
        # calculate mass properties
        depth = (lss_diameter * 1.5)

        if location[0] != 0.0:
            cm = location

        else:
            cmMB = np.array([0.0,0.0,0.0])
            cmMB = ([- (0.035 * rotor_diameter), 0.0, 0.025 * rotor_diameter])
            cm = cmMB
            
        outputs['cm'] = cm    
        outputs['mass'] = mass
        b1I0 = (mass * lss_diameter ** 2 ) / 4.0 
        outputs['I'] = np.array([b1I0, b1I0 / 2.0, b1I0 / 2.0])
        
        
        
class SecondBearing_drive(ExplicitComponent): 
    ''' MainBearings class          
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    def setup(self):
        # variables
        #self.add_input('bearing_type', desc='Main bearing type: CARB, TRB1 or SRB')
        self.add_input('bearing_mass', units = 'kg', desc = 'bearing mass from LSS model')
        self.add_input('lss_diameter', units='m', desc='lss outer diameter at main bearing')
        self.add_input('lss_design_torque', units='N*m', desc='lss design torque')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('location', units = 'm', desc = 'x,y,z location from shaft model', shape=3)
    
        # returns
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=(3,1))
 
    
    def compute(self, inputs, outputs):

        bearing_type = 'TRB2' #inputs['bearing_type']
        bearing_mass = inputs['bearing_mass']
        lss_diameter = inputs['lss_diameter']
        lss_design_torque = inputs['lss_design_torque']
        rotor_diameter = inputs['rotor_diameter']
        location = inputs['location']
        
        mass = bearing_mass
        mass += mass*(8000.0/2700.0) #add housing weight

        # calculate mass properties
        depth = (lss_diameter * 1.5)

        if mass > 0 and location[0] != 0.0:
            cm = location
        else:
            cm = np.array([0,0,0])
            mass = 0.
            
        outputs['mass'] = mass
        outputs['cm'] = cm       

        b2I0  = (mass * lss_diameter ** 2 ) / 4.0 
        outputs['I'] = ([b2I0, b2I0 / 2.0, b2I0 / 2.0])        




class Gearbox_drive(ExplicitComponent):
    ''' Gearbox class
          The Gearbox class is used to represent the gearbox component of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    def setup(self):
        #variables
        
        self.add_input('gear_ratio', desc='overall gearbox speedup ratio')
        self.add_input('Np', desc='number of planets in each stage', shape=3)
        self.add_input('rotor_speed', units='rpm', desc='rotor rpm at rated power')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_torque', units='N*m', desc='rotor torque at rated power')
        self.add_input('gearbox_cm_x', units='m', desc ='gearbox position along x-axis')
    
        #parameters
        #self.add_input('gear_configuration', desc='string that represents the configuration of the gearbox (stage number and types)')
        #self.add_input('ratio_type', desc='optimal or empirical stage ratios')
        #self.add_input('shaft_type', desc = 'normal or short shaft length')
    
        # outputs
        self.add_output('stage_masses', units='kg', desc='individual gearbox stage masses', shape=(3,1))
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=(3,1))    
        self.add_output('length', units='m', desc='gearbox length')
        self.add_output('height', units='m', desc='gearbox height')
        self.add_output('diameter', units='m', desc='gearbox diameter')


    def compute(self, inputs, outputs):

        gear_ratio = inputs['gear_ratio']
        Np = inputs['Np']
        rotor_speed = inputs['rotor_speed']
        rotor_diameter = inputs['rotor_diameter']
        rotor_torque = inputs['rotor_torque']
        gearbox_cm_x = inputs['gearbox_cm_x']
        #gear_configuration = inputs['gear_configuration']
        #ratio_type = inputs['ratio_type']
        #shaft_type = inputs['shaft_type']
        
        gear_configuration='epp'
        ratio_type='optimal' # 'empirical'
        shaft_type='normal'

        stageRatio=np.zeros([3,1])
        stageTorque = np.zeros([len(stageRatio),1]) #filled in when ebxWeightEst is called
        stageMass = np.zeros([len(stageRatio),1]) #filled in when ebxWeightEst is called
        stageType=stageTypeCalc(gear_configuration)
        stageRatio=stageRatioCalc(gear_ratio,Np,ratio_type,gear_configuration)
        
        [m, stageMass] =gbxWeightEst(gear_configuration,gear_ratio,Np,ratio_type,shaft_type, \
                       rotor_torque, stageRatio, stageTorque, stageMass, stageType)
        #print(m)
        mass = float(m)
        outputs['mass'] = mass 
        outputs['stage_masses']= stageMass
        # calculate mass properties

        length = (0.012 * rotor_diameter)
        height = (0.015 * rotor_diameter)
        diameter = (0.75 * height)

        cm0   = gearbox_cm_x
        cm1   = 0.0
        cm2   = 0.4*height #TODO validate or adjust factor. origin is modified to be above bedplate top
        outputs['length'] = length
        outputs['height'] = height
        outputs['diameter'] = diameter
        outputs['cm'] = np.array([cm0, cm1, cm2]) #np.ndarray(shape=(3,), buffer=np.array([cm0, cm1, cm2])) #np.ndarray([cm0, cm1, cm2])
#         outputs['cm'][0] = cm0
#         outputs['cm'][1] = cm1
#         outputs['cm'][2] = cm2
        #outputs['cm'] = np.reshape(cm, (3,1))

        I0 = mass * (diameter ** 2 ) / 8 + (mass / 2) * (height ** 2) / 8
        I1 = mass * (0.5 * (diameter ** 2) + (2 / 3) * (length ** 2) + 0.25 * (height ** 2)) / 8
        I2 = I1
        outputs['I'] = np.array([I0, I1, I2])





class Bedplate_drive(ExplicitComponent):
    ''' Bedplate class
          The Bedplate class is used to represent the bedplate of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    def setup(self):
        #variables
        self.add_input('gbx_length', units = 'm', desc = 'gearbox length')
        self.add_input('gbx_location', units = 'm', desc = 'gearbox CM location')
        self.add_input('gbx_mass', units = 'kg', desc = 'gearbox mass')
        self.add_input('hss_location', units = 'm', desc='HSS CM location')
        self.add_input('hss_mass', units = 'kg', desc='HSS mass')
        self.add_input('generator_location', units = 'm', desc='generator CM location')
        self.add_input('generator_mass', units = 'kg', desc='generator mass')
        self.add_input('lss_location', units = 'm', desc='LSS CM location')
        self.add_input('lss_mass', units = 'kg', desc='LSS mass')
        self.add_input('lss_length', units = 'm', desc = 'LSS length')
        self.add_input('mb1_location', units = 'm', desc='Upwind main bearing CM location')
        self.add_input('FW_mb1', units = 'm', desc = 'Upwind main bearing facewidth')
        self.add_input('mb1_mass', units = 'kg', desc='Upwind main bearing mass')
        self.add_input('mb2_location', units = 'm', desc='Downwind main bearing CM location')
        self.add_input('mb2_mass', units = 'kg', desc='Downwind main bearing mass')
        #self.add_input('transformer_mass', units = 'kg', desc='Transformer mass')
        #self.add_input('transformer_location', units = 'm', desc = 'transformer CM location')
        self.add_input('tower_top_diameter', units = 'm', desc='diameter of the top tower section at the yaw gear')
        self.add_input('rotor_diameter', units = 'm', desc='rotor diameter')
        self.add_input('machine_rating', units='kW', desc='machine_rating machine rating of the turbine')
        self.add_input('rotor_mass', units='kg', desc='rotor mass')
        self.add_input('rotor_bending_moment_y', units='N*m', desc='The bending moment about the y axis')
        self.add_input('rotor_force_z', units='N', desc='The force along the z axis applied at hub center')
        self.add_input('flange_length', units='m', desc='flange length')
        self.add_input('L_rb', units = 'm', desc = 'length between rotor center and upwind main bearing')
        self.add_input('overhang', units='m', desc='Overhang distance')
    
        #parameters
        #self.add_input('uptower_transformer', desc = 'Boolean stating if transformer is uptower', val=1)
    
        #outputs
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)    
        self.add_output('length', units='m', desc='length of bedplate')
        self.add_output('height', units='m', desc='max height of bedplate')
        self.add_output('width', units='m', desc='width of bedplate')

    def compute(self, inputs, outputs):
        #Model bedplate as 2 parallel I-beams with a rear steel frame and a front cast frame
        #Deflection constraints applied at each bedplate end
        #Stress constraint checked at root of front and rear bedplate sections
        
        gbx_length = inputs['gbx_length']
        gbx_location = inputs['gbx_location']
        gbx_mass = inputs['gbx_mass']
        hss_location = inputs['hss_location']
        hss_mass = inputs['hss_mass']
        generator_location = inputs['generator_location']
        generator_mass = inputs['generator_mass']
        lss_location = inputs['lss_location']
        lss_mass = inputs['lss_mass']
        lss_length = inputs['lss_length']
        mb1_location = inputs['mb1_location']
        FW_mb1 = inputs['FW_mb1']
        mb1_mass = inputs['mb1_mass']
        mb2_location = inputs['mb2_location']
        mb2_mass = inputs['mb2_mass']
        transformer_mass = 0.0 #inputs['transformer_mass']
        transformer_location = [0., 0., 0.] #inputs['transformer_location']
        tower_top_diameter = inputs['tower_top_diameter']
        rotor_diameter = inputs['rotor_diameter']
        machine_rating = inputs['machine_rating']
        rotor_mass = inputs['rotor_mass']
        rotor_bending_moment_y = inputs['rotor_bending_moment_y']
        rotor_force_z = inputs['rotor_force_z']
        flange_length = inputs['flange_length']
        L_rb = inputs['L_rb']
        overhang = inputs['overhang']
        uptower_transformer = 0 #inputs['uptower_transformer']
        
        
        [tf, tw, h0, b0, density, g, E, \
         gbx_location, gbx_mass, rotorLoc, rotorFz, rotorMy, \
         rearTotalLength, frontTotalLength, convLoc, transLoc, convMass, \
         rootStress, totalTipDefl, stressTol, deflTol, \
         defl_denom, stress_mult, stressMax, deflMax] = setup_Bedplate(L_rb, rotor_diameter, \
                                        transformer_mass, transformer_location, \
                                        machine_rating, generator_location, \
                                        mb1_location, mb2_location, lss_location, tower_top_diameter, FW_mb1, \
                                        rotor_force_z, rotor_bending_moment_y, rotor_mass, gbx_location, gbx_mass )
            
        # transLoc, rearTotalLength
        counter = 0
        while rootStress*stress_mult - stressMax >  stressTol or totalTipDefl - deflMax >  deflTol:
    
          counter += 1
          #print counter
    
          [totalTipDefl, rootStress, totalSteelMass] = characterize_Bedplate_Rear(tf, tw, h0, b0, density, g, E, rearTotalLength, \
                                     hss_location, hss_mass, generator_location, generator_mass, \
                                     convLoc, convMass, transLoc, transformer_mass, gbx_location, gbx_mass)
    
          tf += 0.002 
          tw += 0.002
          b0 += 0.006
          h0 += 0.006
          rearCounter = counter
    
        rearHeight = h0
    
        #Front cast section:
        [rootStress, totalTipDefl, deflMax, stressMax, \
         gbx_location, gbx_mass, \
         tf, tw, h0, b0, castDensity, E] = setup_Bedplate_Front(gbx_location, gbx_mass, frontTotalLength, defl_denom)
    
        counter = 0
    
        while rootStress*stress_mult - stressMax >  stressTol or totalTipDefl - deflMax >  deflTol:
          counter += 1
          #print counter
          [totalTipDefl, rootStress, totalCastMass] = characterize_Bedplate_Front(tf, tw, h0, b0, castDensity, g, E, frontTotalLength, \
                                      gbx_location, gbx_mass, mb1_location, mb1_mass, \
                                      mb2_location, mb2_mass, lss_location, lss_mass, \
                                      rotorLoc, rotor_mass, rotorFz, rotorMy)
          tf += 0.002 
          tw += 0.002
          b0 += 0.006
          h0 += 0.006
    
          frontCounter=counter
        
        frontHeight = h0
          
        [mass, cm, I, length, width, height] = size_Bedplate(b0, rotor_diameter, tower_top_diameter, frontHeight, rearHeight, \
                                                            frontTotalLength, rearTotalLength, totalCastMass, totalSteelMass)

        outputs['mass'] = mass
        outputs['cm'] = cm
        outputs['I'] = I
        outputs['length'] = length
        outputs['height'] = height
        outputs['width'] = width    






class HighSpeedSide_drive(ExplicitComponent):
    '''
    HighSpeedShaft class
          The HighSpeedShaft class is used to represent the high speed shaft and mechanical brake components of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    
    def setup(self):
        # variables
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_torque', units='N*m', desc='rotor torque at rated power')
        self.add_input('gear_ratio', desc='overall gearbox ratio')
        self.add_input('lss_diameter', units='m', desc='low speed shaft outer diameter')
        self.add_input('gearbox_length', units = 'm', desc='gearbox length')
        self.add_input('gearbox_height', units = 'm', desc = 'gearbox height')
        self.add_input('gearbox_cm', units = 'm', desc = 'gearbox cm [x,y,z]', shape=3)
        self.add_input('hss_length', units = 'm', desc = 'high speed shaft length determined by user. Default 0.5m')
    
        # returns
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)
        self.add_output('length', units='m', desc='length of high speed shaft')



    def compute(self, inputs, outputs):
        
        rotor_diameter = inputs['rotor_diameter']
        rotor_torque = inputs['rotor_torque']
        gear_ratio = inputs['gear_ratio']
        lss_diameter = inputs['lss_diameter']
        gearbox_length = inputs['gearbox_length']
        gearbox_height = inputs['gearbox_height']
        gearbox_cm = inputs['gearbox_cm']
        hss_length = inputs['hss_length']

        # compute masses, dimensions and cost
        design_torque = rotor_torque / gear_ratio               # design torque [Nm] based on rotor torque and Gearbox ratio
        massFact = 0.025                                 # mass matching factor default value
        highSpeedShaftMass = (massFact * design_torque)
        
        mechBrakeMass = (0.5 * highSpeedShaftMass)      # relationship derived from HSS multiplier for University of Sunderland model compared to NREL CSM for 750 kW and 1.5 MW turbines
        
        mass = (mechBrakeMass + highSpeedShaftMass)
        outputs['mass'] = mass
        
        diameter = (1.5 * lss_diameter)                     # based on WindPACT relationships for full HSS / mechanical brake assembly
        if hss_length == 0:
            length = 0.5+rotor_diameter/127.
        else:
            length = hss_length
        outputs['length'] = length
        
        matlDensity = 7850. # material density kg/m^3
        
        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0]   = gearbox_cm[0]+gearbox_length/2+length/2
        cm[1]   = gearbox_cm[1]
        cm[2]   = gearbox_cm[2]+gearbox_height*0.2
        outputs['cm'] = cm
        
        I = np.array([0.0, 0.0, 0.0])
        I[0]    = 0.25 * length * 3.14159 * matlDensity * (diameter ** 2) * (gear_ratio**2) * (diameter ** 2) / 8.
        I[1]    = mass * ((3/4.) * (diameter ** 2) + (length ** 2)) / 12.
        I[2]    = I[1]
        outputs['I'] = I
        
        
        




class Generator_drive(ExplicitComponent):
    '''Generator class
          The Generator class is used to represent the generator of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    
    def setup(self):
        # variables
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('machine_rating', units='kW', desc='machine rating of generator')
        self.add_input('gear_ratio', desc='overall gearbox ratio')
        self.add_input('highSpeedSide_length', units = 'm', desc='length of high speed shaft and brake')
        self.add_input('highSpeedSide_cm', units = 'm', desc='cm of high speed shaft and brake', shape=3)
        self.add_input('rotor_speed', units='rpm', desc='Speed of rotor at rated power')
    
        # parameters
        #drivetrain_design = Enum('geared', ('geared', 'single_stage', 'multi_drive', 'pm_direct_drive'), iotype='in')
        
    
        # returns
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)

    def compute(self, inputs, outputs):
        
        rotor_diameter = inputs['rotor_diameter']
        machine_rating = inputs['machine_rating']
        gear_ratio = inputs['gear_ratio']
        highSpeedSide_length = inputs['highSpeedSide_length']
        highSpeedSide_cm = inputs['highSpeedSide_cm']
        rotor_speed = inputs['rotor_speed']
        #drivetrain_design = inputs['drivetrain_design']     
        drivetrain_design =  'geared'  
        
        massCoeff = [None, 6.4737, 10.51 ,  5.34  , 37.68  ]
        massExp   = [None, 0.9223, 0.9223,  0.9223, 1      ]
        
        if rotor_speed !=0:
          CalcRPM = rotor_speed
        else:
          CalcRPM    = 80 / (rotor_diameter*0.5*pi/30)
        CalcTorque = (machine_rating*1.1) / (CalcRPM * pi/30)
        
        if drivetrain_design == 'geared':
            drivetrain_design = 1
        elif drivetrain_design == 'single_stage':
            drivetrain_design = 2
        elif drivetrain_design == 'multi_drive':
            drivetrain_design = 3
        elif drivetrain_design == 'pm_direct_drive':
            drivetrain_design = 4
        
        if (drivetrain_design < 4):
            mass = (massCoeff[drivetrain_design] * machine_rating ** massExp[drivetrain_design])
        else:  # direct drive
            mass = (massCoeff[drivetrain_design] * CalcTorque ** massExp[drivetrain_design])
            
        outputs['mass'] = mass    
        
        # calculate mass properties
        length = (1.8 * 0.015 * rotor_diameter)
        d_length_d_rotor_diameter = 1.8*.015
        
        depth = (0.015 * rotor_diameter)
        d_depth_d_rotor_diameter = 0.015
        
        width = (0.5 * depth)
        d_width_d_depth = 0.5
        
        # print highSpeedSide_cm
        cm = np.array([0.0,0.0,0.0])
        cm[0]  = highSpeedSide_cm[0] + highSpeedSide_length/2. + length/2.
        cm[1]  = highSpeedSide_cm[1]
        cm[2]  = highSpeedSide_cm[2]
        outputs['cm'] = cm
        
        I = np.array([0.0, 0.0, 0.0])
        I[0]   = ((4.86 * (10. ** (-5))) * (rotor_diameter ** 5.333)) + (((2./3.) * mass) * (depth ** 2 + width ** 2) / 8.)
        I[1]   = (I[0] / 2.) / (gear_ratio ** 2) + ((1./3.) * mass * (length ** 2) / 12.) + (((2. / 3.) * mass) * \
                   (depth ** 2. + width ** 2. + (4./3.) * (length ** 2.)) / 16. )
        I[2]   = I[1]
        outputs['I'] = I        
        
        
        
        
        
class YawSystem_drive(ExplicitComponent):
    ''' YawSystem class
          The YawSystem class is used to represent the yaw system of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    
    def setup(self):
        #variables
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_thrust', units='N', desc='maximum rotor thrust')
        self.add_input('tower_top_diameter', units='m', desc='tower top diameter')
        self.add_input('above_yaw_mass', units='kg', desc='above yaw mass')
        self.add_input('bedplate_height', units = 'm', desc = 'bedplate height')
    
        #parameters
        #self.add_input('yaw_motors_number', desc='number of yaw motors', val=0)
    
        #outputs
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)    


    def compute(self, inputs, outputs):
        
        rotor_diameter = inputs['rotor_diameter']
        rotor_thrust = inputs['rotor_thrust']
        tower_top_diameter = inputs['tower_top_diameter']
        above_yaw_mass = inputs['above_yaw_mass']
        bedplate_height = inputs['bedplate_height']
        yaw_motors_number = 0 #inputs['yaw_motors_number']

        if yaw_motors_number == 0 :
          if rotor_diameter < 90.0 :
            yaw_motors_number = 4
          elif rotor_diameter < 120.0 :
            yaw_motors_number = 6
          else:
            yaw_motors_number = 8
        
        #Assume friction plate surface width is 1/10 the diameter
        #Assume friction plate thickness scales with rotor diameter
        frictionPlateVol=pi*tower_top_diameter*(tower_top_diameter*0.10)*(rotor_diameter/1000.0)
        steelDensity=8000.0
        frictionPlateMass=frictionPlateVol*steelDensity
        
        #Assume same yaw motors as Vestas V80 for now: Bonfiglioli 709T2M
        yawMotorMass=190.0
        
        totalYawMass=frictionPlateMass + (yaw_motors_number*yawMotorMass)
        outputs['mass']= totalYawMass
        
        # calculate mass properties
        # yaw system assumed to be collocated to tower top center
        cm = np.array([0.0,0.0,0.0])
        cm[2] = -bedplate_height
        outputs['cm'] = cm
        
        # assuming 0 MOI for yaw system (ie mass is nonrotating)
        I = np.array([0.0, 0.0, 0.0])
        outputs['I'] = I       
        
        

class Transformer_drive(ExplicitComponent):
    ''' Transformer class
            The transformer class is used to represent the transformer of a wind turbine drivetrain.
            It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
            It contains an update method to determine the mass, mass properties, and dimensions of the component if it is in fact uptower'''

    def setup(self):
        #inputs
        self.add_input('machine_rating', units='kW', desc='machine rating of the turbine')
        self.add_input('uptower_transformer', desc = 'uptower or downtower transformer')
        self.add_input('tower_top_diameter', units = 'm', desc = 'tower top diameter for comparision of nacelle CM')
        self.add_input('rotor_mass', units='kg', desc='rotor mass')
        self.add_input('overhang', units='m', desc='rotor overhang distance')
        self.add_input('generator_cm', units='m', desc='center of mass of the generator in [x,y,z]', shape=3)
        self.add_input('rotor_diameter', units='m', desc='rotor diameter of turbine')
        self.add_input('RNA_mass', units='kg', desc='mass of total RNA')
        self.add_input('RNA_cm', units='m', desc='RNA CM along x-axis', shape=3)
    
        #outputs
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)    

    def compute(self, inputs, outputs):

        machine_rating = inputs['machine_rating']
        uptower_transformer = inputs['uptower_transformer']
        tower_top_diameter = inputs['tower_top_diameter']
        rotor_mass = inputs['rotor_mass']
        overhang = inputs['overhang']
        generator_cm = inputs['generator_cm']
        rotor_diameter = inputs['rotor_diameter']
        RNA_mass = inputs['RNA_mass']
        RNA_cm = inputs['RNA_cm']

        def combine_CM(mass1,CM1,mass2,CM2):
            return (mass1*CM1+mass2*CM2)/(mass1+mass2)

        if uptower_transformer == 1:
            #function places transformer where tower top CM is within tower bottom OD to reduce tower moments
            if rotor_mass:
                rotor_mass = rotor_mass
            else:
                [rotor_mass] = get_rotor_mass(machine_rating,False)
        
            bottom_OD = tower_top_diameter*1.7 #approximate average from industry data
            # print bottom_OD
        
            mass = 2.4445*(machine_rating) + 1599.0
        
            if RNA_cm <= -(bottom_OD)/2: #upwind of acceptable. Most likely
                transformer_x = (bottom_OD/2.*(RNA_mass+mass) - (RNA_mass*RNA_cm))/(mass)
                if transformer_x > generator_cm[0]*3:
                    print '\n ---------transformer location manipulation not suitable for overall Nacelle CM changes: rear distance excessively large------- \n'
                    transformer_x = generator_cm[0] + (1.6 * 0.015 * rotor_diameter) #assuming generator and transformer approximately same length
            else:
                transformer_x = generator_cm[0] + (1.8 * 0.015 * rotor_diameter) #assuming generator and transformer approximately same length
        
            cm = np.array([0.,0.,0.])
            cm[0] = transformer_x
            cm[1] = generator_cm[1]
            cm[2] = generator_cm[2]/.75*.5 #same height as gearbox CM
        
            width = tower_top_diameter+.5
            height = 0.016*rotor_diameter #similar to gearbox
            length = .012*rotor_diameter #similar to gearbox
        
            def get_I(d1,d2,mass):
                return mass*(d1**2 + d2**2)/12.
        
            I = np.array([0.,0.,0.])
            I[0] = get_I(height,width,mass)
            I[1] = get_I(length, height, mass)
            I[2] = get_I(length, width, mass)
        
        else:
            cm = np.array([0.,0.,0.])
            I = cm.copy()
            mass = 0.

        outputs['mass'] = mass
        outputs['cm'] = cm
        outputs['I'] = I        
        

class RNASystemAdder_drive(ExplicitComponent):
    ''' RNASystem class
          This analysis is only to be used in placing the transformer of the drivetrain.
          The Rotor-Nacelle-Assembly class is used to represent the RNA of the turbine without the transformer and bedplate (to resolve circular dependency issues).
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component. 
    '''
    
    def setup(self):
        #inputs
        self.add_input('yawMass', units='kg', desc='mass of yaw system')
        self.add_input('lss_mass', units='kg', desc='component mass')
        self.add_input('main_bearing_mass', units='kg', desc='component mass')
        self.add_input('second_bearing_mass', units='kg', desc='component mass')
        self.add_input('gearbox_mass', units='kg', desc='component mass')
        self.add_input('hss_mass', units='kg', desc='component mass')
        self.add_input('generator_mass', units='kg', desc='component mass')
        self.add_input('lss_cm', units='m', desc='component CM', shape=3)
        self.add_input('main_bearing_cm', units='m', desc='component CM', shape=3)
        self.add_input('second_bearing_cm', units='m', desc='component CM', shape=3)
        self.add_input('gearbox_cm', units='m', desc='component CM', shape=3)
        self.add_input('hss_cm', units='m', desc='component CM', shape=3)
        self.add_input('generator_cm', units='m', desc='component CM', shape=3)
        self.add_input('overhang', units='m', desc='nacelle overhang')
        self.add_input('rotor_mass', units='kg', desc='component mass')
        self.add_input('machine_rating', units = 'kW', desc = 'machine rating ')
    
        #returns
        self.add_output('RNA_mass', units='kg', desc='mass of total RNA')
        self.add_output('RNA_cm', units='m', desc='RNA CM along x-axis', shape=3)

    def compute(self, inputs, outputs):
        
        yawMass = inputs['yawMass']
        lss_mass = inputs['lss_mass']
        main_bearing_mass = inputs['main_bearing_mass']
        second_bearing_mass = inputs['second_bearing_mass']
        gearbox_mass = inputs['gearbox_mass']
        hss_mass = inputs['hss_mass']
        generator_mass = inputs['generator_mass']
        lss_cm = inputs['lss_cm']
        main_bearing_cm = inputs['main_bearing_cm']
        second_bearing_cm = inputs['second_bearing_cm']
        gearbox_cm = inputs['gearbox_cm']
        hss_cm = inputs['hss_cm']
        generator_cm = inputs['generator_cm']
        overhang = inputs['overhang']
        rotor_mass = inputs['rotor_mass']
        machine_rating = inputs['machine_rating']

        if rotor_mass>0:
            rotor_mass = rotor_mass
        else:
            [rotor_mass] = get_rotor_mass(machine_rating,False)
        
        masses = np.array([rotor_mass, yawMass, lss_mass, main_bearing_mass,second_bearing_mass,gearbox_mass,hss_mass,generator_mass])
        cms = np.array([(-overhang), 0.0, lss_cm[0], main_bearing_cm[0], second_bearing_cm[0], gearbox_cm[0], hss_cm[0], generator_cm[0]])
        
        outputs['RNA_mass'] = np.sum(masses)
        outputs['RNA_cm'] = np.sum(masses*cms)/np.sum(masses)





# class NacelleGS(Group):
#     def setup(self):
#         
#         self.add_subsystem('gearbox', Gearbox_drive(), promotes_outputs=[('mass', 'gearbox_mass')])
#         self.add_subsystem('lowSpeedShaft', LowSpeedShaft_drive3pt(), promotes_outputs=[('mass', 'low_speed_shaft_mass')])
#         self.add_subsystem('mainBearing', MainBearing_drive(), promotes_outputs=[('mass', 'main_bearing_mass'), ('cm', 'MB1_location')])
#         self.add_subsystem('secondBearing',SecondBearing_drive(), promotes_outputs=[('mass', 'second_bearing_mass')])
#         self.add_subsystem('highSpeedSide', HighSpeedSide_drive(), promotes_outputs=[('mass', 'high_speed_side_mass')])
#         self.add_subsystem('generator', Generator_drive(), promotes_outputs=[('mass', 'generator_mass')])
#         self.add_subsystem('bedplate', Bedplate_drive(), promotes_outputs=[('mass', 'bedplate_mass')])
#         self.add_subsystem('above_yaw_massAdder', AboveYawMassAdder_drive())
#         self.add_subsystem('yawSystem', YawSystem_drive(), promotes_outputs=[('mass', 'yaw_system_mass')])
#         self.add_subsystem('rna', RNASystemAdder_drive())
#         self.add_subsystem('nacelleSystem', NacelleSystemAdder_drive(), promotes_outputs=['nacelle_mass', 'nacelle_cm', 'nacelle_I'])
#         
#         self.connect('gearbox.mass', ['lowSpeedShaft.gearbox_mass', 'bedplate.gbx_mass', 'above_yaw_massAdder.gearbox_mass', 'rna.gearbox_mass', 'nacelleSystem.gearbox_mass'])
#         self.connect('gearbox.cm', ['lowSpeedShaft.gearbox_cm', 'highSpeedSide.gearbox_cm', 'bedplate.gbx_location', 'rna.gearbox_cm', 'nacelleSystem.gearbox_cm'])
#         self.connect('gearbox.length', ['lowSpeedShaft.gearbox_length', 'highSpeedSide.gearbox_length', 'bedplate.gbx_length'])
#         self.connect('gearbox.height', ['highSpeedSide.gearbox_height'])
#         self.connect('gearbox.I', ['nacelleSystem.gearbox_I'])
#         
#         self.connect('lowSpeedShaft.mass', ['bedplate.lss_mass', 'above_yaw_massAdder.lss_mass', 'rna.lss_mass', 'nacelleSystem.lss_mass'])
#         self.connect('lowSpeedShaft.cm', ['bedplate.lss_location', 'rna.lss_cm', 'nacelleSystem.lss_cm'])
#         self.connect('lowSpeedShaft.length', ['bedplate.lss_length'])
#         self.connect('lowSpeedShaft.FW_mb', ['bedplate.FW_mb1'])
#         self.connect('lowSpeedShaft.I', ['nacelleSystem.lss_I'])
#         self.connect('lowSpeedShaft.bearing_mass1', 'mainBearing.bearing_mass')
#         self.connect('lowSpeedShaft.bearing_mass2', 'secondBearing.bearing_mass')
#         self.connect('lowSpeedShaft.bearing_location1', 'mainBearing.location')
#         self.connect('lowSpeedShaft.bearing_location2', 'secondBearing.location')
#         self.connect('lowSpeedShaft.diameter1', ['mainBearing.lss_diameter', 'highSpeedSide.lss_diameter'])
#         self.connect('lowSpeedShaft.diameter2', 'secondBearing.lss_diameter')
#         
#         self.connect('mainBearing.bearing_mass', ['bedplate.mb1_mass', 'above_yaw_massAdder.main_bearing_mass', 'rna.main_bearing_mass', 'nacelleSystem.main_bearing_mass'])
#         #self.connect('lowSpeedShaft.design_torque', ['mainBearing.lss_design_torque', 'secondBearing.lss_design_torque'])
#         self.connect('lowSpeedShaft.bearing_location1', ['mainBearing.location', 'bedplate.mb1_location', 'rna.main_bearing_cm', 'nacelleSystem.main_bearing_cm'])
#         self.connect('mainBearing.I', ['nacelleSystem.main_bearing_I'])
#         
#         self.connect('lowSpeedShaft.bearing_mass2', ['secondBearing.bearing_mass', 'bedplate.mb2_mass', 'above_yaw_massAdder.second_bearing_mass', 'rna.second_bearing_mass', 'nacelleSystem.second_bearing_mass'])
#         self.connect('lowSpeedShaft.diameter2', 'secondBearing.lss_diameter')
#         self.connect('lowSpeedShaft.bearing_location2', ['secondBearing.location', 'bedplate.mb2_location', 'rna.second_bearing_cm', 'nacelleSystem.second_bearing_cm'])
#         self.connect('secondBearing.I', ['nacelleSystem.second_bearing_I'])       
# 
#         
#         self.connect('highSpeedSide.mass', ['bedplate.hss_mass', 'above_yaw_massAdder.hss_mass', 'rna.hss_mass', 'nacelleSystem.hss_mass'])
#         self.connect('highSpeedSide.length', ['generator.highSpeedSide_length'])
#         self.connect('highSpeedSide.cm', ['generator.highSpeedSide_cm', 'bedplate.hss_location', 'rna.hss_cm', 'nacelleSystem.hss_cm'])
#         self.connect('highSpeedSide.I', ['nacelleSystem.hss_I'])
#         
#         self.connect('generator.cm', ['bedplate.generator_location', 'rna.generator_cm', 'nacelleSystem.generator_cm'])
#         self.connect('generator.mass', ['bedplate.generator_mass', 'above_yaw_massAdder.generator_mass', 'rna.generator_mass', 'nacelleSystem.generator_mass'])
#         self.connect('generator.I', ['nacelleSystem.generator_I'])
#         
#         self.connect('bedplate.mass', ['above_yaw_massAdder.bedplate_mass', 'nacelleSystem.bedplate_mass'])
#         self.connect('bedplate.length', ['above_yaw_massAdder.bedplate_length'])
#         self.connect('bedplate.width', ['above_yaw_massAdder.bedplate_width'])
#         self.connect('bedplate.height', ['yawSystem.bedplate_height'])
#         self.connect('bedplate.cm', ['nacelleSystem.bedplate_cm'])
#         self.connect('bedplate.I', ['nacelleSystem.bedplate_I'])
#         
#         self.connect('above_yaw_massAdder.above_yaw_mass', ['yawSystem.above_yaw_mass', 'nacelleSystem.above_yaw_mass'])
#         self.connect('above_yaw_massAdder.mainframe_mass', ['nacelleSystem.mainframe_mass'])
#         
#         self.connect('yawSystem.mass', ['rna.yawMass', 'nacelleSystem.yawMass'])
#         
#         
#         
#         self.nonlinear_solver = NonlinearBlockGS()
        






class NacelleSE(Group):
    def setup(self):
        
        
        
        '''
          DriveSE class
          The DriveSE3pt class is used to represent the nacelle system of a wind turbine with a single main bearing
        '''
    
        torque = 1.5 * (5000.0 * 1000 / 0.95) / (12.1 * (pi / 30))
        sd_mass = 333.3*5000.0/1000.0
        # variables
        i = IndepVarComp()
        i.add_output('rotor_diameter', units='m', desc='rotor diameter', val=126.0)
        i.add_output('rotor_speed', units='rpm', desc='rotor speed at rated', val=12.1)
        i.add_output('machine_rating', units='kW', desc='machine rating of generator', val=5000.0)
        i.add_output('rotor_torque', units='N*m', desc='rotor torque at rated power', val=torque)
        i.add_output('rotor_thrust', units='N', desc='maximum rotor thrust', val=599610.0)
        i.add_output('rotor_mass', units='kg', desc='rotor mass')
        i.add_output('rotor_bending_moment_x', units='N*m', desc='The bending moment about the x axis', val=330770.0)
        i.add_output('rotor_bending_moment_y', units='N*m', desc='The bending moment about the y axis', val=-16665000.0)
        i.add_output('rotor_bending_moment_z', units='N*m', desc='The bending moment about the z axis', val=2896300.0)
        i.add_output('rotor_force_x', units='N', desc='The force along the x axis applied at hub center', val=599610.0)
        i.add_output('rotor_force_y', units='N', desc='The force along the y axis applied at hub center', val=186780.0)
        i.add_output('rotor_force_z', units='N', desc='The force along the z axis applied at hub center', val=-842710.0)
        i.add_output('gear_ratio', desc='overall gearbox ratio', val=96.76)
        i.add_output('crane', desc='flag for presence of crane', val=1)
        i.add_output('shaft_angle', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal', val=5.0)
        i.add_output('shaft_ratio', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS', val=0.10)
        i.add_output('Np', desc='number of planets in each stage', val=np.array([3.0,3.0,1.0,]))
        i.add_output('shrink_disc_mass', units='kg', desc='Mass of the shrink disc', val=sd_mass)
        i.add_output('carrier_mass', units='kg', desc='Carrier mass', val=8000.0)
        i.add_output('flange_length', units='m', desc='flange length', val=0.5)
        i.add_output('overhang', units='m', desc='Overhang distance', val=5.0)
        i.add_output('L_rb', units='m', desc='distance between hub center and upwind main bearing', val=1.912)
        i.add_output('gearbox_cm_x', units = 'm', desc = 'distance from tower-top center to gearbox cm--negative for upwind', val=0.0)
        i.add_output('tower_top_diameter', units='m', desc='diameter of tower top', val=3.78)
        i.add_output('hss_length', units = 'm', desc = 'optional high speed shaft length determined by user')
        
        #i.add_output('rotor_bending_moment', units='N*m', desc='maximum aerodynamic bending moment', val=-16665000.0)
        #i.add_output('bevel', desc='Flag for the presence of a bevel stage - 1 if present, 0 if not', val=1)
        #drivetrain_design = Enum('geared', ('geared', 'single_stage', 'multi_drive', 'pm_direct_drive'), iotype='in')        
        #gear_configuration = Str(iotype='in', desc='tring that represents the configuration of the gearbox (stage number and types)')
        #i.add_output('blade_root_diameter', units='m', desc='blade root diameter')
        #i.add_output('ratio_type', desc='optimal or empirical stage ratios')
        #i.add_output('shaft_type', desc = 'normal or short shaft length')
        #i.add_output('uptower_transformer', desc = 'Boolean stating if transformer is uptower', val=1)
        #mb1Type = Enum('SRB',('CARB','TRB1','TRB2','SRB','CRB','RB'),iotype='in',desc='Main bearing type')
        #mb2Type = Enum('SRB',('CARB','TRB1','TRB2','SRB','CRB','RB'),iotype='in',desc='Second bearing type')



        
        # select components
        self.add_subsystem('dof', i)  
        self.add_subsystem('gearbox', Gearbox_drive())
        self.add_subsystem('lowSpeedShaft', LowSpeedShaft_drive3pt())
        self.add_subsystem('mainBearing', MainBearing_drive())
        self.add_subsystem('secondBearing',SecondBearing_drive())
        self.add_subsystem('highSpeedSide', HighSpeedSide_drive())
        self.add_subsystem('generator', Generator_drive())
        self.add_subsystem('bedplate', Bedplate_drive())
        self.add_subsystem('above_yaw_massAdder', AboveYawMassAdder_drive())
        self.add_subsystem('yawSystem', YawSystem_drive())
        self.add_subsystem('rna', RNASystemAdder_drive())
        self.add_subsystem('nacelleSystem', NacelleSystemAdder_drive())
        
#         self.add_subsystem('dof', i)  
#         self.add_subsystem('gearbox', Gearbox_drive(), promotes_outputs=[('mass', 'gearbox_mass')])
#         self.add_subsystem('lowSpeedShaft', LowSpeedShaft_drive3pt(), promotes_outputs=[('mass', 'low_speed_shaft_mass')])
#         self.add_subsystem('mainBearing', MainBearing_drive(), promotes_outputs=[('mass', 'main_bearing_mass'), ('cm', 'MB1_location')])
#         self.add_subsystem('secondBearing',SecondBearing_drive(), promotes_outputs=[('mass', 'second_bearing_mass')])
#         self.add_subsystem('highSpeedSide', HighSpeedSide_drive(), promotes_outputs=[('mass', 'high_speed_side_mass')])
#         self.add_subsystem('generator', Generator_drive(), promotes_outputs=[('mass', 'generator_mass')])
#         self.add_subsystem('bedplate', Bedplate_drive(), promotes_outputs=[('mass', 'bedplate_mass')])
#         self.add_subsystem('above_yaw_massAdder', AboveYawMassAdder_drive())
#         self.add_subsystem('yawSystem', YawSystem_drive(), promotes_outputs=[('mass', 'yaw_system_mass')])
#         self.add_subsystem('rna', RNASystemAdder_drive())
#         self.add_subsystem('nacelleSystem', NacelleSystemAdder_drive(), promotes_outputs=['nacelle_mass', 'nacelle_cm', 'nacelle_I'])
        
        #self.add_subsystem('cycle', NacelleGS())
        

        # connect inputs
#         self.connect('dof.rotor_diameter', ['cycle.gearbox.rotor_diameter', 'cycle.lowSpeedShaft.rotor_diameter', \
#                                             'cycle.mainBearing.rotor_diameter', 'cycle.secondBearing.rotor_diameter', 'cycle.highSpeedSide.rotor_diameter', \
#                                             'cycle.generator.rotor_diameter', 'cycle.bedplate.rotor_diameter', 'cycle.yawSystem.rotor_diameter'])
#         self.connect('dof.rotor_speed', ['cycle.gearbox.rotor_speed','cycle.generator.rotor_speed'])
#         self.connect('dof.machine_rating', ['cycle.lowSpeedShaft.machine_rating', 'cycle.generator.machine_rating', 'cycle.bedplate.machine_rating', 'cycle.above_yaw_massAdder.machine_rating','cycle.rna.machine_rating'])
#         self.connect('dof.rotor_torque', ['cycle.gearbox.rotor_torque', 'cycle.highSpeedSide.rotor_torque', 'cycle.mainBearing.lss_design_torque', 'cycle.secondBearing.lss_design_torque']) # Need to address internal torque calculations...
#         self.connect('dof.rotor_thrust', 'cycle.yawSystem.rotor_thrust')
#         self.connect('dof.rotor_mass', ['cycle.bedplate.rotor_mass','cycle.lowSpeedShaft.rotor_mass','cycle.rna.rotor_mass'])
#         self.connect('dof.rotor_bending_moment_x', ['cycle.lowSpeedShaft.rotor_bending_moment_x'])
#         self.connect('dof.rotor_bending_moment_y', ['cycle.bedplate.rotor_bending_moment_y','cycle.lowSpeedShaft.rotor_bending_moment_y'])
#         self.connect('dof.rotor_bending_moment_z', 'cycle.lowSpeedShaft.rotor_bending_moment_z')
#         self.connect('dof.rotor_force_x', 'cycle.lowSpeedShaft.rotor_force_x')
#         self.connect('dof.rotor_force_y', 'cycle.lowSpeedShaft.rotor_force_y')
#         self.connect('dof.rotor_force_z', ['cycle.bedplate.rotor_force_z','cycle.lowSpeedShaft.rotor_force_z'])
#         self.connect('dof.gear_ratio', ['cycle.gearbox.gear_ratio', 'cycle.generator.gear_ratio', 'cycle.highSpeedSide.gear_ratio'])
#         self.connect('dof.crane', 'cycle.above_yaw_massAdder.crane')
#         self.connect('dof.shaft_angle', 'cycle.lowSpeedShaft.shaft_angle')
#         self.connect('dof.shaft_ratio', 'cycle.lowSpeedShaft.shaft_ratio')
#         self.connect('dof.Np', 'cycle.gearbox.Np')
#         self.connect('dof.shrink_disc_mass', 'cycle.lowSpeedShaft.shrink_disc_mass')
#         self.connect('dof.carrier_mass', 'cycle.lowSpeedShaft.carrier_mass')
#         self.connect('dof.flange_length', ['cycle.bedplate.flange_length','cycle.lowSpeedShaft.flange_length'])
#         self.connect('dof.overhang',['cycle.lowSpeedShaft.overhang','cycle.bedplate.overhang', 'cycle.rna.overhang'])
#         self.connect('dof.L_rb', ['cycle.lowSpeedShaft.L_rb','cycle.bedplate.L_rb'])
#         self.connect('dof.gearbox_cm', 'cycle.gearbox.gearbox_cm')
#         self.connect('dof.tower_top_diameter', ['cycle.bedplate.tower_top_diameter', 'cycle.yawSystem.tower_top_diameter'])
#         self.connect('dof.hss_length', 'cycle.highSpeedSide.hss_length')
        
        
        self.connect('dof.rotor_diameter', ['gearbox.rotor_diameter', 'lowSpeedShaft.rotor_diameter', \
                                            'mainBearing.rotor_diameter', 'secondBearing.rotor_diameter', 'highSpeedSide.rotor_diameter', \
                                            'generator.rotor_diameter', 'bedplate.rotor_diameter', 'yawSystem.rotor_diameter'])
        self.connect('dof.rotor_speed', ['gearbox.rotor_speed','generator.rotor_speed'])
        self.connect('dof.machine_rating', ['lowSpeedShaft.machine_rating', 'generator.machine_rating', 'bedplate.machine_rating', 'above_yaw_massAdder.machine_rating','rna.machine_rating'])
        self.connect('dof.rotor_torque', ['gearbox.rotor_torque', 'highSpeedSide.rotor_torque']) # Need to address internal torque calculations...
        self.connect('dof.rotor_torque', ['mainBearing.lss_design_torque', 'secondBearing.lss_design_torque'])
        self.connect('dof.rotor_thrust', 'yawSystem.rotor_thrust')
        self.connect('dof.rotor_mass', ['bedplate.rotor_mass','lowSpeedShaft.rotor_mass','rna.rotor_mass'])
        self.connect('dof.rotor_bending_moment_x', ['lowSpeedShaft.rotor_bending_moment_x'])
        self.connect('dof.rotor_bending_moment_y', ['bedplate.rotor_bending_moment_y','lowSpeedShaft.rotor_bending_moment_y'])
        self.connect('dof.rotor_bending_moment_z', 'lowSpeedShaft.rotor_bending_moment_z')
        self.connect('dof.rotor_force_x', 'lowSpeedShaft.rotor_force_x')
        self.connect('dof.rotor_force_y', 'lowSpeedShaft.rotor_force_y')
        self.connect('dof.rotor_force_z', ['bedplate.rotor_force_z','lowSpeedShaft.rotor_force_z'])
        self.connect('dof.gear_ratio', ['gearbox.gear_ratio', 'generator.gear_ratio', 'highSpeedSide.gear_ratio'])
        self.connect('dof.crane', 'above_yaw_massAdder.crane')
        self.connect('dof.shaft_angle', 'lowSpeedShaft.shaft_angle')
        self.connect('dof.shaft_ratio', 'lowSpeedShaft.shaft_ratio')
        self.connect('dof.Np', 'gearbox.Np')
        self.connect('dof.shrink_disc_mass', 'lowSpeedShaft.shrink_disc_mass')
        self.connect('dof.carrier_mass', 'lowSpeedShaft.carrier_mass')
        self.connect('dof.flange_length', ['bedplate.flange_length','lowSpeedShaft.flange_length'])
        self.connect('dof.overhang',['lowSpeedShaft.overhang','bedplate.overhang', 'rna.overhang'])
        self.connect('dof.L_rb', ['lowSpeedShaft.L_rb','bedplate.L_rb'])
        self.connect('dof.gearbox_cm_x', 'gearbox.gearbox_cm_x')
        self.connect('dof.tower_top_diameter', ['bedplate.tower_top_diameter', 'yawSystem.tower_top_diameter'])
        self.connect('dof.hss_length', 'highSpeedSide.hss_length')
        
        


        self.connect('gearbox.mass', ['lowSpeedShaft.gearbox_mass', 'bedplate.gbx_mass', 'above_yaw_massAdder.gearbox_mass', 'rna.gearbox_mass', 'nacelleSystem.gearbox_mass'])
        self.connect('gearbox.cm', ['lowSpeedShaft.gearbox_cm', 'highSpeedSide.gearbox_cm', 'rna.gearbox_cm', 'nacelleSystem.gearbox_cm'])
        self.connect('gearbox.length', ['lowSpeedShaft.gearbox_length', 'highSpeedSide.gearbox_length', 'bedplate.gbx_length'])
        self.connect('gearbox.height', ['highSpeedSide.gearbox_height'])
        self.connect('gearbox.I', ['nacelleSystem.gearbox_I'])
        
        self.connect('lowSpeedShaft.mass', ['bedplate.lss_mass', 'above_yaw_massAdder.lss_mass', 'rna.lss_mass', 'nacelleSystem.lss_mass'])
        self.connect('lowSpeedShaft.cm', ['rna.lss_cm', 'nacelleSystem.lss_cm'])
        self.connect('lowSpeedShaft.length', ['bedplate.lss_length'])
        self.connect('lowSpeedShaft.FW_mb', ['bedplate.FW_mb1'])
        self.connect('lowSpeedShaft.I', ['nacelleSystem.lss_I'])
        self.connect('lowSpeedShaft.bearing_mass1', 'mainBearing.bearing_mass')
        self.connect('lowSpeedShaft.bearing_mass2', 'secondBearing.bearing_mass')
        self.connect('lowSpeedShaft.bearing_location1', 'mainBearing.location')
        self.connect('lowSpeedShaft.bearing_location2', 'secondBearing.location')
        self.connect('lowSpeedShaft.diameter1', ['mainBearing.lss_diameter', 'highSpeedSide.lss_diameter'])
        self.connect('lowSpeedShaft.diameter2', 'secondBearing.lss_diameter')
        #self.connect('lowSpeedShaft.design_torque', ['mainBearing.lss_design_torque', 'secondBearing.lss_design_torque'])
        
        self.connect('mainBearing.mass', ['bedplate.mb1_mass', 'above_yaw_massAdder.main_bearing_mass', 'rna.main_bearing_mass', 'nacelleSystem.main_bearing_mass'])
        self.connect('mainBearing.cm', ['rna.main_bearing_cm', 'nacelleSystem.main_bearing_cm'])
        self.connect('mainBearing.I', ['nacelleSystem.main_bearing_I'])
        
        self.connect('secondBearing.mass', ['bedplate.mb2_mass', 'above_yaw_massAdder.second_bearing_mass', 'rna.second_bearing_mass', 'nacelleSystem.second_bearing_mass'])
        self.connect('secondBearing.cm', ['rna.second_bearing_cm', 'nacelleSystem.second_bearing_cm'])
        self.connect('secondBearing.I', ['nacelleSystem.second_bearing_I'])       

        
        self.connect('highSpeedSide.mass', ['bedplate.hss_mass', 'above_yaw_massAdder.hss_mass', 'rna.hss_mass', 'nacelleSystem.hss_mass'])
        self.connect('highSpeedSide.length', ['generator.highSpeedSide_length'])
        self.connect('highSpeedSide.cm', ['generator.highSpeedSide_cm', 'rna.hss_cm', 'nacelleSystem.hss_cm'])
        self.connect('highSpeedSide.I', ['nacelleSystem.hss_I'])
        
        self.connect('generator.cm', ['rna.generator_cm', 'nacelleSystem.generator_cm'])
        self.connect('generator.mass', ['bedplate.generator_mass', 'above_yaw_massAdder.generator_mass', 'rna.generator_mass', 'nacelleSystem.generator_mass'])
        self.connect('generator.I', ['nacelleSystem.generator_I'])
        
        self.connect('bedplate.mass', ['above_yaw_massAdder.bedplate_mass', 'nacelleSystem.bedplate_mass'])
        self.connect('bedplate.length', ['above_yaw_massAdder.bedplate_length'])
        self.connect('bedplate.width', ['above_yaw_massAdder.bedplate_width'])
        self.connect('bedplate.height', ['yawSystem.bedplate_height'])
        self.connect('bedplate.cm', ['nacelleSystem.bedplate_cm'])
        self.connect('bedplate.I', ['nacelleSystem.bedplate_I'])
        
        self.connect('gearbox.cm', 'bedplate.gbx_location', src_indices=[0])
        self.connect('highSpeedSide.cm', 'bedplate.hss_location', src_indices=[0])
        self.connect('generator.cm', 'bedplate.generator_location', src_indices=[0])
        self.connect('lowSpeedShaft.cm', 'bedplate.lss_location', src_indices=[0])
        self.connect('mainBearing.cm', 'bedplate.mb1_location', src_indices=[0])
        self.connect('secondBearing.cm', 'bedplate.mb2_location', src_indices=[0])
        
        self.connect('above_yaw_massAdder.above_yaw_mass', ['yawSystem.above_yaw_mass', 'nacelleSystem.above_yaw_mass'])
        self.connect('above_yaw_massAdder.mainframe_mass', ['nacelleSystem.mainframe_mass'])
        
        self.connect('yawSystem.mass', ['rna.yawMass', 'nacelleSystem.yawMass'])
        
        
        









        
if __name__ == "__main__":
    
    from openmdao.api import Problem, view_model
    start = time()
    
    
    # get and set values of the variables using Problem
    prob = Problem(NacelleSE())
    prob.setup()
    prob['dof.rotor_diameter'] = 126.
    prob['dof.rotor_speed'] = 12.1
    prob['dof.machine_rating'] = 5000.
    prob['dof.rotor_torque'] = 6230511.04
    prob['dof.rotor_thrust'] = 599610.
    prob['dof.rotor_mass'] = 0.
    prob['dof.rotor_bending_moment_x'] = 330770.
    prob['dof.rotor_bending_moment_y'] = -16665000.
    prob['dof.rotor_bending_moment_z'] = 2896300.
    prob['dof.rotor_force_x'] = 599610.
    prob['dof.rotor_force_y'] = 186780.
    prob['dof.rotor_force_z'] = -842710.
    prob['dof.gear_ratio'] = 96.76
    prob['dof.crane'] = 1
    prob['dof.shaft_angle'] = 5.
    prob['dof.shaft_ratio'] = 0.1
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.shrink_disc_mass'] = 1666.5
    prob['dof.carrier_mass'] = 8000.
    prob['dof.flange_length'] = 0.5
    prob['dof.overhang'] = 5.
    prob['dof.L_rb'] = 1.912
    prob['dof.gearbox_cm_x'] = 0.0
    prob['dof.tower_top_diameter'] = 3.78
    prob['dof.hss_length'] = 0.
    
    
    view_model(prob, outfile='nacelle.html')
    prob.run_model()
    
    print prob['nacelleSystem.nacelle_mass'] 
    print(time() - start, "seconds", clock())
    #print prob['nacelle_I'] 
               
        
        
        