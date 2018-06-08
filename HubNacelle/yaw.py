from time import time
from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model 

from fixed_parameters import safety_factor, beautify
from drivese_utils import size_YawSystem


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class Yaw(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_thrust', units='N', desc='maximum rotor thrust')
        self.add_input('tower_top_diameter', units='m', desc='tower top diameter')
        self.add_input('above_yaw_mass', units='kg', desc='above yaw mass')
        self.add_input('bedplate_height', units = 'm', desc = 'bedplate height')
    
        # outputs
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)    

        
        
#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class DriveSE(Yaw):
    def compute(self, inputs, outputs):
        # inputs
        self.rotor_diameter = inputs['rotor_diameter']
        self.rotor_thrust = inputs['rotor_thrust']*safety_factor
        self.tower_top_diameter = inputs['tower_top_diameter']
        self.above_yaw_mass = inputs['above_yaw_mass']
        self.bedplate_height = inputs['bedplate_height']
        self.yaw_motors_number = 0 # will be derived emperically in the next step
        
        size_YawSystem(self)
        
        outputs['mass'] = self.mass
        outputs['cm'] = self.cm
        outputs['I'] = self.I
    
    
    
            