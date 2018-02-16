from openmdao.api import ExplicitComponent, Group, IndepVarComp
from input_generator import *

import numpy as np
import pandas
from time import time, clock
import datetime as dt
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil
import os

from fixed_parameters import num_bins, num_nodes, rho_air, spanwise_params, z0, h_ref, wind_shear
from bem import bem_rotor




class HubWindSpeed(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('pot_wind_speed', units = 'm/s', desc = 'potential wind speed at reference height of 10m')
        self.add_input('hub_height', units = 'm', desc = 'hub radius')
        
        self.add_output('hub_wind_speed', units='m/s', desc='wind speed at hub height')
        
    def compute(self, inputs, outputs):
         
        pot_wind_speed = inputs['pot_wind_speed']
        hub_height = inputs['hub_height']
        
        # reference height to 60m (log law)
        u_60 = pot_wind_speed * np.log(60/z0) / np.log(h_ref/z0)
            
        # 60m to hub height (exponential law)
        outputs['hub_wind_speed'] = u_60 * (hub_height/60) ** wind_shear    
        
        
        
        

class AnnulusAero(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('wind_speed', units = 'm/s', desc = 'wind speed')
        self.add_input('blade_number', desc='number of blades')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        self.add_input('hub_radius', units = 'm', desc = 'hub radius')
        self.add_input('pitch', units = 'deg', desc = 'blade pitch angle')
        self.add_input('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        self.add_input('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        self.add_input('span_twist', units='deg',desc='list of blade node twist angle', shape=num_nodes)
        self.add_input('span_airfoil', desc='list of blade node Airfoil ID', shape=num_nodes)
     
        # outputs
        self.add_output('span_area', units='m**2', desc='list of spanwise area', shape=num_nodes)
        self.add_output('span_cp', desc='list of spanwise Cp', shape=num_nodes)
        self.add_output('span_cq', desc='list of spanwise Cq', shape=num_nodes)
        self.add_output('span_ct', desc='list of spanwise Ct', shape=num_nodes)
        self.add_output('span_fx', units = 'N/m', desc='list of spanwise Fx - normal to the plane of rotation', shape=num_nodes)
        self.add_output('span_fy', units = 'N/m', desc='list of spanwise Fy - tangential to the plane of rotation', shape=num_nodes)
        
 
 
    def compute(self, inputs, outputs):
        
        wind_speed = inputs['wind_speed'] 
        blade_number = inputs['blade_number']
        rotor_diameter = inputs['rotor_diameter']
        rotor_speed = inputs['rotor_speed']
        hub_radius = inputs['hub_radius']
        pitch = inputs['pitch']
        span_r = inputs['span_r']
        span_chord = inputs['span_chord']
        span_twist = inputs['span_twist']
        span_airfoil = inputs['span_airfoil']
        
        rotor_radius = rotor_diameter/2.0
        tsr = (rotor_speed * 2 * pi / 60.0) * rotor_radius/wind_speed
        
        result = bem_rotor(wind_speed, blade_number, rotor_radius, hub_radius, tsr, pitch, \
                span_r, span_chord, span_twist, span_airfoil, is_prandtl=1, is_glauert=1)
        
        
        outputs['span_area'] = result['area'].tolist()
        outputs['span_cp'] = result['cp'].tolist()
        outputs['span_cq'] = result['cq'].tolist()
        outputs['span_ct'] = result['ct'].tolist()
        outputs['span_fx'] = result['fx'].tolist()
        outputs['span_fy'] = result['fy'].tolist()
        
        


class RotorAero(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('wind_speed', units = 'm/s', desc = 'wind speed')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        self.add_input('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        self.add_input('span_area', units='m**2', desc='list of blade node area', shape=num_nodes)
        self.add_input('span_cp', desc='list of spanwise Cp', shape=num_nodes)
        self.add_input('span_ct', desc='list of spanwise Ct', shape=num_nodes)
        self.add_input('span_fx', units = 'N/m', desc='list of spanwise Fx - normal to the plane of rotation', shape=num_nodes)
        self.add_input('span_fy', units = 'N/m', desc='list of spanwise Fy - tangential to the plane of rotation', shape=num_nodes)
        
        
        # outputs
        self.add_output('rotor_tsr', desc='rotor tip speed ratio')
        self.add_output('swept_area', units='m**2', desc='rotor tip speed ratio')
        self.add_output('rotor_cp', desc='rotor power coefficient')
        self.add_output('rotor_cq', desc='rotor torque coefficient')
        self.add_output('rotor_ct',  desc='rotor thrust coefficient')
        self.add_output('rotor_power', units='W', desc='rotor power')
        self.add_output('rotor_torque', units='N*m', desc='rotor torque')
        self.add_output('rotor_thrust',  units='N', desc='rotor thrust')
        
        
    def compute(self, inputs, outputs):
        
        wind_speed = inputs['wind_speed']
        rotor_diameter = inputs['rotor_diameter']
        rotor_speed = inputs['rotor_speed']
        span_r = inputs['span_r']
        span_area = inputs['span_area'] 
        span_cp = inputs['span_cp']
        span_ct = inputs['span_ct']
        span_fx = inputs['span_fx']
        span_fy = inputs['span_fy']
        
        swept_area = pi * (rotor_diameter/2)**2
        rotational_speed = rotor_speed * (2*pi/60)
        
        tsr = (rotor_speed * 2 * pi / 60.0) * rotor_radius/wind_speed
        cp = np.trapz(span_cp, span_area)/swept_area
        power = cp * 0.5 * rho_air * swept_area * wind_speed**3
        cq = cp/tsr
        torque = power/rotational_speed
        ct = np.trapz(span_ct, span_area)/swept_area 
        thrust = ct *  0.5 * rho_air * swept_area * wind_speed**2 
        
        outputs['rotor_tsr'] = rotor_tsr
        outputs['swept_area'] = swept_area
        outputs['rotor_cp'] = rotor_cp
        outputs['rotor_cq'] = rotor_cq
        outputs['rotor_ct'] = rotor_ct
        outputs['rotor_power'] = rotor_power
        outputs['rotor_torque'] = rotor_torque
        outputs['rotor_thrust'] = rotor_thrust
        

class HubAero(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('wind_speed', units = 'm/s', desc = 'wind speed')
        self.add_input('hub_radius', units='m', desc='rotor diameter')
        self.add_input('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        self.add_input('span_fx', units = 'N/m', desc='list of spanwise Fx - normal to the plane of rotation', shape=num_nodes)
        self.add_input('span_fy', units = 'N/m', desc='list of spanwise Fy - tangential to the plane of rotation', shape=num_nodes)
        
        
        # outputs
        self.add_output('root_bending_moment',  units='N*m', desc='bending moment vector at the hub', shape=3)
        self.add_output('root_force',  units='N', desc='force vector at the hub', shape=3)
        

    def compute(self, inputs, outputs):
        
        wind_speed = inputs['wind_speed']
        hub_radius = inputs['hub_radius']      
        span_r = inputs['span_r'] 
        span_fx = inputs['span_fx'] 
        span_fy = inputs['span_fy']  
        
        hub_cd = 0.2
        fx = hub_cd * 0.5 * rho_air *  (wind_speed**2) * (pi * hub_radius**2)
        outputs['root_force'] = [fx, 0, 0]
        
        mx = np.trapz(span_fx, span_r)
        my = np.trapz(span_fy, span_r)
        outputs['root_bending_moment'] = [mx, my, 0]
        
        
        
        
        
class AeroAdder(Group):
    def setup(self):
        
        self.add_subsystem('wind', HubWindSpeed(), \
                           promotes_inputs = ['pot_wind_speed', 'hub_height'], \
                           promotes_outputs=[('hub_wind_speed', 'wind_speed')])
             
        self.add_subsystem('annulus', AnnulusAero(), \
                           promotes_inputs = ['blade_number', 'rotor_diameter', 'rotor_speed', 'hub_radius', \
                                              'pitch', 'span_r', 'span_chord', 'span_twist', 'span_airfoil'], \
                           promotes_outputs=['span_fx', 'span_fy'])
        
        self.add_subsystem('rotor', RotorAero(), \
                           promotes_inputs = ['rotor_diameter', 'rotor_speed', 'span_r'], \
                           promotes_outputs=['*'])
        
        self.add_subsystem('hub', HubAero(), \
                           promotes_inputs = ['hub_radius', 'span_r'], \
                           promotes_outputs=['root_bending_moment', 'root_force'])
        
        
        self.connect('wind_speed', ['annulus.wind_speed', 'rotor.wind_speed', 'hub.wind_speed'])  
        self.connect('annulus.span_area', 'rotor.span_area')    
        self.connect('annulus.span_cp', 'rotor.span_cp')  
        self.connect('annulus.span_ct', 'rotor.span_ct')  
        self.connect('span_fx', ['rotor.span_fx', 'hub.span_fx'])  
        self.connect('span_fy', ['rotor.span_fy', 'hub.span_fy']) 
        
        
        
           
        





        
class RotorAeroGroupTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        i.add_output('pot_wind_speed', units = 'm/s', desc = 'potential wind speed at reference height of 10m')
        i.add_output('hub_height', units = 'm', desc = 'hub radius')
        i.add_output('blade_number', desc='number of blades')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('pitch', units = 'deg', desc = 'blade pitch angle')
        i.add_output('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        i.add_output('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        i.add_output('span_twist', units='deg',desc='list of blade node twist angle', shape=num_nodes)
        i.add_output('span_airfoil', desc='list of blade node Airfoil ID', shape=num_nodes)

                
        self.add_subsystem('dof', i)   
        self.add_subsystem('aero', AeroAdder())
               
        # parameters
        self.connect('dof.pot_wind_speed', 'aero.pot_wind_speed')
        self.connect('dof.hub_height', 'aero.hub_height')
        self.connect('dof.blade_number', 'aero.blade_number')
        self.connect('dof.rotor_diameter', 'aero.rotor_diameter')
        self.connect('dof.rotor_speed', 'aero.rotor_speed')
        self.connect('dof.hub_radius', 'aero.hub_radius')
        self.connect('dof.pitch', 'aero.pitch')
        self.connect('dof.span_r', 'aero.span_r')
        self.connect('dof.span_chord', 'aero.span_chord')
        self.connect('dof.span_twist', 'aero.span_twist')
        self.connect('dof.span_airfoil', 'aero.span_airfoil')
        
        
        


if __name__ == "__main__":
    
    from openmdao.api import Problem, view_model
     
    start = time()
    
    # get and set values of the variables using Problem
    prob = Problem(RotorAeroGroupTest())
    prob.setup()
    view_model(prob, outfile='rotor_aero_new.html')
    
    prob['dof.pot_wind_speed'] = 10.0 
    prob['dof.hub_height'] = 90.0
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 100.0
    prob['dof.rotor_speed'] = 19.1
    prob['dof.hub_radius'] = 1.5
    prob['dof.pitch'] = 0.0
    prob['dof.span_r'] = [10.0, 20.0, 30.0, 40.0, 50.0]
    prob['dof.span_chord'] = [3.4, 1.9, 1.3, 1.0, 0.8]
    prob['dof.span_twist'] = [5.9, -2.3, -5.5, -7.0, -7.9]
    prob['dof.span_airfoil'] = [2, 2, 2, 2, 2]
     
     
    prob.run_model()
     
    print "My Rotor"
    print prob['aero.wind_speed'] 
    print prob['aero.span_fx'] 
    print prob['aero.span_fy']
    print prob['aero.rotor_tsr'] 
    print prob['aero.rotor_power'] 
    print prob['aero.rotor_torque'] 
    print prob['aero.rotor_thrust'] 
    print prob['aero.root_bending_moment']
    print prob['aero.root_force']
 
  
    print 'Done in ' + str(time() - start) + ' seconds'


 