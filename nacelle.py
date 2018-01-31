from openmdao.api import ExplicitComponent, Group, IndepVarComp

import numpy as np
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil
import algopy
import scipy as scp
import scipy.optimize as opt
from scipy import integrate


class AboveYawMassAdder_drive(ExplicitComponent):

    def setup(self):
        # variables
        self.add_input('machine_rating', units='kW', desc='machine rating', val=0.0)
        self.add_input('lss_mass', units='kg', desc='component mass', val=0.0)
        self.add_input('main_bearing_mass', units='kg', desc='component mass', val=0.0)
        self.add_input('second_bearing_mass', units='kg', desc='component mass', val=0.0)
        self.add_input('gearbox_mass', units='kg', desc='component mass', val=0.0)
        self.add_input('hss_mass', units='kg', desc='component mass', val=0.0)
        self.add_input('generator_mass', units='kg', desc='component mass', val=0.0)
        self.add_input('bedplate_mass', units='kg', desc='component mass', val=0.0)
        self.add_input('bedplate_length', units='m', desc='component length', val=0.0)
        self.add_input('bedplate_width', units='m', desc='component width', val=0.0)
        self.add_input('transformer_mass', units='kg', desc='component mass', val=0.0)
    
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
        transformer_mass = inputs['transformer_mass']
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
        self.add_input('lss_cm', units='kg', desc='component CM', val=np.array([0.0,0.0,0.0]))
        self.add_input('main_bearing_cm', units='kg', desc='component CM', val=np.array([0.0,0.0,0.0]))
        self.add_input('second_bearing_cm', units='kg', desc='component CM', val=np.array([0.0,0.0,0.0]))
        self.add_input('gearbox_cm', units='kg', desc='component CM', val=np.array([0.0,0.0,0.0]))
        self.add_input('hss_cm', units='kg', desc='component CM', val=np.array([0.0,0.0,0.0]))
        self.add_input('generator_cm', units='kg', desc='component CM', val=np.array([0.0,0.0,0.0]))
        self.add_input('bedplate_cm', units='kg', desc='component CM', val=np.array([0.0,0.0,0.0]))
        self.add_input('lss_I', units='kg', desc='component I', val=np.array([0.0,0.0,0.0]))
        self.add_input('main_bearing_I', units='kg', desc='component I', val=np.array([0.0,0.0,0.0]))
        self.add_input('second_bearing_I', units='kg', desc='component I', val=np.array([0.0,0.0,0.0]))
        self.add_input('gearbox_I', units='kg', desc='component I', val=np.array([0.0,0.0,0.0]))
        self.add_input('hss_I', units='kg', desc='component I', val=np.array([0.0,0.0,0.0]))
        self.add_input('generator_I', units='kg', desc='component I', val=np.array([0.0,0.0,0.0]))
        self.add_input('bedplate_I', units='kg', desc='component I', val=np.array([0.0,0.0,0.0]))
        self.add_input('transformer_mass', units='kg', desc='component mass', val=0.0)
        self.add_input('transformer_cm', units='kg', desc='component CM', val=np.array([0.0,0.0,0.0]))
        self.add_input('transformer_I', units='kg', desc='component I', val=np.array([0.0,0.0,0.0]))
    
        # returns
        self.add_output('nacelle_mass', units='kg', desc='overall component mass', val=0.0)
        self.add_output('nacelle_cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', val=np.array([0.0,0.0,0.0]))
        self.add_output('nacelle_I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', val=np.array([0.0,0.0,0.0]))

    
    
    
    def compute(self, inputs, outputs):

        add_Nacelle(self)







        
        
if __name__ == "__main__":
    
    from openmdao.api import Problem, view_model
    
    
    # get and set values of the variables using Problem
    prob = Problem(HubSE())
    prob.setup()
    view_model(prob)
    prob.run_model()
    
    print prob['above_yaw_mass']  # 31644.47
    print prob['length'] # 17003.98
               
        
        
        