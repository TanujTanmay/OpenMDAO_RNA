import numpy as np
from time import time
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil

from openmdao.api import ExplicitComponent, Group, IndepVarComp
from fixed_parameters import rho_iron


class HubNose(ExplicitComponent):
    ''' Hub class    
          The Hub class is used to represent the hub component of a wind turbine. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.            
    '''
    def setup(self):
        # inputs
        self.add_input('blade_root_diameter', units='m', desc='blade root diameter')
        self.add_input('blade_number', desc='number of turbine blades')
    
        # outputs
        self.add_output('diameter', units='m', desc='hub diameter')
        self.add_output('thickness', units='m',desc='hub thickness')
        self.add_output('length', units='m',desc='hub length, distance between hub center and main bearing')
        self.add_output('mass', units='kg', desc='overall component mass')
    

    def compute(self, inputs, outputs):
        
        blade_root_diameter = inputs['blade_root_diameter']
        blade_number = inputs['blade_number']

        # model hub as a cyclinder with holes for blade root and nacelle flange.
        r = 1.1*blade_root_diameter/2.0
        h = 2.8*blade_root_diameter/2.0
        t = r/10.0
        volume_cylinder = 2*pi*r*t*h
        volume_opening  = pi*(blade_root_diameter/2.0)**2*t

        # assume nacelle flange opening is similar to blade root opening
        volume_hub = volume_cylinder - (1.0 + blade_number)*volume_opening
        mass_hub = volume_hub * rho_iron

        # calculate mass properties
        outputs['diameter']=2*r
        outputs['thickness']=t  
        outputs['length']=h
        outputs['mass'] = mass_hub

class Hub_System_Adder_drive(ExplicitComponent):
    ''' Get_hub_cm class
          The Get_hub_cm class is used to pass the hub cm data to upper level models.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
    '''
 
    def setup(self):
        # variables
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('L_rb', units = 'm', desc = 'distance between hub center and upwind main bearing')
        self.add_input('shaft_angle', units = 'deg', desc = 'shaft angle')
        self.add_input('MB1_location', units = 'm', desc = 'center of mass of main bearing in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_input('hub_mass', units='kg',desc='mass of Hub')
        self.add_input('hub_diameter', units='m', desc='hub diameter', val=3.0)
        self.add_input('hub_thickness', units='m', desc='hub thickness')
        self.add_input('pitch_system_mass', units='kg',desc='mass of Pitch System')
        self.add_input('spinner_mass', units='kg',desc='mass of spinner')
     
        # outputs
        self.add_output('hub_system_cm', units='m',desc='center of mass of the hub relative to tower to in yaw-aligned c.s.', shape=3)
        self.add_output('hub_system_I', desc='mass moments of Inertia of hub [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] around its center of mass in yaw-aligned c.s.', shape=6)
        self.add_output('hub_system_mass', units='kg',desc='mass of hub system')
 
 
    def compute(self, inputs, outputs):
         
        rotor_diameter = inputs['rotor_diameter']
        L_rb = inputs['L_rb']
        shaft_angle = inputs['shaft_angle']
        MB1_location = inputs['MB1_location']
        hub_mass = inputs['hub_mass']
        hub_diameter = inputs['hub_diameter']
        hub_thickness = inputs['hub_thickness']
        pitch_system_mass = inputs['pitch_system_mass']
        spinner_mass = inputs['spinner_mass']
         
         
        if L_rb>0:
            L_rb = L_rb
        else:
            L_rb = get_L_rb(rotor_diameter)
 
        cm = np.array([0.0,0.0,0.0])
        cm[0]     = MB1_location[0] - L_rb[0]
        cm[1]     = 0.0
        cm[2]     = MB1_location[2] + L_rb[0]*sin(radians(shaft_angle))
        outputs['hub_system_cm'] = (cm)
 
        outputs['hub_system_mass'] = hub_mass + pitch_system_mass + spinner_mass
 
 
        #add I definitions here
        hub_I = np.array([0.0, 0.0, 0.0])
        hub_I[0] = 0.4 * (hub_mass) * ((hub_diameter / 2) ** 5 - (hub_diameter / 2 - hub_thickness) ** 5) / \
               ((hub_diameter / 2) ** 3 - (hub_diameter / 2 - hub_thickness) ** 3)
        hub_I[1] = hub_I[0]
        hub_I[2] = hub_I[1]
 
        pitch_system_I = np.array([0.0, 0.0, 0.0])
        pitch_system_I[0] = pitch_system_mass * (hub_diameter ** 2) / 4
        pitch_system_I[1] = pitch_system_I[0]
        pitch_system_I[2] = pitch_system_I[1]
 
 
        if hub_diameter == 0:
            spinner_diameter =(3.30)
        else:
            spinner_diameter =(hub_diameter)
        spinner_thickness = spinner_diameter * (0.055 / 3.30)         # 0.055 for 1.5 MW outer diameter of 3.3 - using proportional constant
 
        spinner_I = np.array([0.0, 0.0, 0.0])
        spinner_I[0] = 0.4 * (spinner_mass) * ((spinner_diameter / 2) ** 5 - (spinner_diameter / 2 - spinner_thickness) ** 5) / \
               ((spinner_diameter / 2) ** 3 - (spinner_diameter / 2 - spinner_thickness) ** 3)
        spinner_I[1] = spinner_I[0]
        spinner_I[2] = spinner_I[1]
 
 
        #add moments of inertia
        I = np.zeros(6)
        for i in (range(0,3)):                        # calculating MOI, at nacelle center of gravity with origin at tower top center / yaw mass center, ignoring masses of non-drivetrain components / auxiliary systems
            # calculate moments around CM
            # sum moments around each components CM
            I[i]  =  hub_I[i] + pitch_system_I[i] + spinner_I[i]
            # translate to hub system CM using parallel axis theorem- because cm is assumed shared, unneeded
            # for j in (range(0,3)):
            #     if i != j:
            #         I[i] +=  (hub_mass * (hub_cm[j] - hub_system_cm[j]) ** 2) + \
            #                       (pitch_system_mass * (pitch_system_cm[j] - hub_system_cm[j]) ** 2) + \
            #                       (spinner_mass * (spinner_cm[j] - hub_system_cm[j]) ** 2)
        outputs['hub_system_I'] = I
        






        
        
        
class PitchSystem_drive(ExplicitComponent):
    '''
     PitchSystem class
      The PitchSystem class is used to represent the pitch system of a wind turbine.
      It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
      It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    
    def setup(self):
        # variables
        self.add_input('blade_mass', units='kg', desc='mass of one blade')
        self.add_input('rotor_bending_moment', units='N*m', desc='flapwise bending moment at blade root')
    
        # parameters
        self.add_input('blade_number', desc='number of turbine blades', val=3)
    
        # outputs
        self.add_output('mass', units='kg', desc='overall component mass', val=0.0)



    def compute(self, inputs, outputs):
        
        blade_mass = inputs['blade_mass']
        rotor_bending_moment = inputs['rotor_bending_moment']
        blade_number = inputs['blade_number']

        # Sunderland method for calculating pitch system masses
        pitchmatldensity = 7860.0                             # density of pitch system material (kg / m^3) - assuming BS1503-622 (same material as LSS)
        pitchmatlstress  = 371000000.0                              # allowable stress of hub material (N / m^2)

        # Root moment required as input, could be undone
        '''if rotor_bending_moment == 0.0:
            rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (RotorDiam ** 3)) / BladeNum
                                                            # simplified equation for blade root moment (Sunderland model) if one is not provided'''

        hubpitchFact      = 1.0                                 # default factor is 1.0 (0.54 for modern designs)
        #self.mass =hubpitchFact * (0.22 * self.blade_mass * self.blade_number + 12.6 * self.blade_number * self.rotor_bending_moment * (pitchmatldensity / pitchmatlstress))
                                                            # mass of pitch system based on Sunderland model
        outputs['mass'] =hubpitchFact * (0.22 * blade_mass * blade_number + 12.6 * rotor_bending_moment * (pitchmatldensity / pitchmatlstress))
                                                            # mass of pitch system based on Sunderland model


#-------------------------------------------------------------------------------

class Spinner_drive(ExplicitComponent):
    '''
       Spinner class
          The SpinnerClass is used to represent the spinner of a wind turbine.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    def setup(self):
        # variables
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
    
        # outputs
        self.add_output('mass', units='kg', desc='overall component mass', val=0.0)

    def compute(self, inputs, outputs):
        outputs['mass'] =18.5 * inputs['rotor_diameter'] + (-520.5)   # spinner mass comes from cost and scaling model 
        
        
        
        
        
        
        
        
        
        
class HubSE(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('blade_number', desc='number of turbine blades')
        i.add_output('blade_root_diameter', units='m', desc='blade root diameter')
        i.add_output('machine_rating', units = 'kW', desc = 'machine rating of turbine')
        i.add_output('blade_mass', units='kg', desc='mass of one blade')
        i.add_output('rotor_bending_moment', units='N*m', desc='flapwise bending moment at blade root')
        i.add_output('L_rb', units='m', desc='distance between hub center and upwind main bearing')
        i.add_output('shaft_angle', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
        i.add_output('MB1_location', units = 'm', desc = 'x,y,z location from shaft model', shape=3)
        
        
        
        
        # parameters
        self.add_subsystem('dof', i)        
        self.add_subsystem('hub', Hub_drive())
        self.add_subsystem('pitchSystem', PitchSystem_drive())
        self.add_subsystem('spinner', Spinner_drive())
        self.add_subsystem('adder', Hub_System_Adder_drive())
        
        
        self.connect('dof.blade_mass', ['pitchSystem.blade_mass'])
        self.connect('dof.rotor_bending_moment', ['pitchSystem.rotor_bending_moment'])
        self.connect('dof.blade_number', ['hub.blade_number', 'pitchSystem.blade_number'])
        self.connect('dof.rotor_diameter', ['spinner.rotor_diameter', 'adder.rotor_diameter'])
        self.connect('dof.blade_root_diameter', 'hub.blade_root_diameter')
        self.connect('dof.machine_rating','hub.machine_rating')
        self.connect('dof.L_rb','adder.L_rb')
        self.connect('dof.shaft_angle','adder.shaft_angle')
        self.connect('dof.MB1_location','adder.MB1_location')
        
        self.connect('hub.mass', 'adder.hub_mass')
        self.connect('hub.thickness', 'adder.hub_thickness')
        self.connect('hub.diameter', 'adder.hub_diameter')
        self.connect('pitchSystem.mass', 'adder.pitch_system_mass')
        self.connect('spinner.mass', 'adder.spinner_mass')
        
        
        
        
if __name__ == "__main__":
    
    from openmdao.api import Problem, view_model
    
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(HubSE())
    prob.setup()
    view_model(prob, outfile='hub.html')
    
    AirDensity= 1.225 # kg/(m^3)
    Solidity  = 0.0517
    RatedWindSpeed = 11.05 # m/s
    
    prob['dof.rotor_diameter'] = 126.0
    prob['dof.blade_number'] = 3
    prob['dof.blade_root_diameter'] = 3.542
    prob['dof.machine_rating'] = 5000.0
    prob['dof.blade_mass'] = 14583.0
    prob['dof.rotor_bending_moment'] = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (prob['dof.rotor_diameter'] ** 3)) / prob['dof.blade_number']

    prob['dof.L_rb'] = 1.91 #inputs['L_rb']
    prob['dof.shaft_angle']     = -6.0 #inputs['shaft_angle']
    prob['dof.MB1_location']     = [-1.03966219,  0.  ,       -0.20292427] #inputs['MB1_location']
    
    
    
    

    
    prob.run_model()
    
    print "NREL 5 MW turbine test"
    print "Hub Components"
    print prob['hub.mass']  # 31644.47
    print prob['pitchSystem.mass'] # 17003.98
    print prob['spinner.mass'] # 1810.50
    print prob['adder.hub_system_mass'] 
#     print prob['hub_system_cm']     
#     print prob['hub_system_I']   
#     print prob['hub_system_mass']   
    print 'Done in ' + str(time() - start) + ' seconds'
               
        
        
        