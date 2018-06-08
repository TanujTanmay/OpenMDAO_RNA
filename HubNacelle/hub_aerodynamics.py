from math import radians, sin, cos
import numpy as np

from openmdao.api import ExplicitComponent 
from fixed_parameters import g, safety_factor


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class HubAerodynamics(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('hub_mass', units = 'kg', desc = 'mass of the hub')
        self.add_input('pitch_mass', units = 'kg', desc = 'mass of pitching system')
        self.add_input('spinner_mass', units = 'kg', desc = 'mass of the spinner')
        
        self.add_input('blade_number', desc='number of turbine blades')
        self.add_input('blade_mass', units = 'kg', desc='mass of the one blade')
        self.add_input('shaft_angle', units='deg', desc='angle of the main shaft inclindation wrt the horizontal')
        self.add_input('rotor_torque', units='N*m', desc='rotor torque')
        self.add_input('rotor_thrust',  units='N', desc='rotor thrust')
    
        # outputs
        self.add_output('hub_assembly_mass', units = 'kg', desc = 'mass of the hub assembly')
        self.add_output('rotor_mass', units = 'kg', desc = 'mass of the rotor = hub + blades')
        self.add_output('rotor_force', units='N', desc='rotor load vector in hub coordinate system', val=[0., 0., 0.])
        self.add_output('rotor_moment',  units='N*m', desc='rotor moment vector in hub coordinate system', val=[0., 0., 0.])
        

#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class Tanuj(HubAerodynamics):
    def compute(self, inputs, outputs):
        # inputs    
        hub_mass = inputs['hub_mass'] + inputs['pitch_mass'] + inputs['spinner_mass']
        blade_number = inputs['blade_number']
        blade_mass = inputs['blade_mass']
        shaft_angle = radians(abs(inputs['shaft_angle']))
        rotor_torque = inputs['rotor_torque']
        rotor_thrust = inputs['rotor_thrust']
        
        # Forces and moments at hub centre
        rotor_mass = (blade_mass * blade_number) + hub_mass
        fx = rotor_thrust + rotor_mass * g * sin(shaft_angle)
        fy = 0 # cancels for all blades
        fz = -1 * rotor_mass * g * cos(shaft_angle)        
        mx = rotor_torque
        my = 0
        mz = 0
        
        # outputs
        outputs['hub_assembly_mass'] = hub_mass
        outputs['rotor_mass'] = rotor_mass
        outputs['rotor_force'] = np.array([fx, fy, fz])*safety_factor
        outputs['rotor_moment'] = np.array([mx, my, mz])*safety_factor

        