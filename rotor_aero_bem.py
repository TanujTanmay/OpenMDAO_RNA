from openmdao.api import ExplicitComponent, Group, IndepVarComp
from input_generator import *

import numpy as np
import pandas
from time import time, clock
import datetime as dt
from math import pi, cos, sqrt, log10, log, floor, ceil, atan, degrees, radians, acos, sqrt, cos, sin, exp
import os

from fixed_parameters import num_bins, num_nodes, rho_air, spanwise_params, z0, h_ref, wind_shear, degree_blade_param, airfoils_db, g
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
        
        
        
        
        

class BladeDesign(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('wind_speed', units='m/s', desc='wind speed at hub height')
        self.add_input('blade_number', desc='number of blades')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_speed', units='rpm', desc='rotor rotational speed')
        self.add_input('hub_radius', units = 'm', desc = 'hub radius')
        self.add_input('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        self.add_input('span_airfoil', desc='list of blade node Airfoil ID', shape=num_nodes)
        self.add_input('chord_params', desc = 'list of coefficients to describe chord distribution', shape=degree_blade_param)
        self.add_input('twist_params', desc = 'list of coefficients to describe twist distribution', shape=degree_blade_param)
        self.add_input('adjust_pitch', desc = 'adjust pitch for tip twist to be 0')
        
        self.add_output('span_dr', units='m', desc='list of blade node thickness', shape=num_nodes)
        self.add_output('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        self.add_output('span_twist', units='deg',desc='list of blade node twist angle', shape=num_nodes)
        self.add_output('root_chord', units='m', desc='root chord length')
        self.add_output('pitch', units='deg',desc='blade pitch angle')
        
    def compute(self, inputs, outputs):
        
        blade_number = inputs['blade_number'] 
        rotor_diameter = inputs['rotor_diameter']
        rotor_speed = inputs['rotor_speed']
        hub_radius = inputs['hub_radius']
        wind_speed = inputs['wind_speed']
        span_r = np.array(inputs['span_r']) 
        span_airfoil = inputs['span_airfoil']
        chord_params = inputs['chord_params']
        twist_params = inputs['twist_params']
        adjust_pitch = inputs['adjust_pitch']
        
        
        omega = rotor_speed * (2*pi/60.0)
        rotor_radius = rotor_diameter/2.0
        tsr = 9 #omega*rotor_radius/wind_speed
        mu = np.divide(span_r, rotor_radius)
        tsr_r = np.multiply(mu, tsr)
        
        mu_root = hub_radius/rotor_radius
        tsr_root = tsr*mu_root
        
        
        
        
        # polynomial distribution
        if np.any(chord_params):
            span_chord = np.polyval(chord_params, mu)
            span_twist = np.polyval(twist_params, mu)
            root_chord = np.polyval(chord_params, mu_root)
            
        # optimal distribution    
        else:
            span_chord = np.array([])
            span_twist = np.array([])
            
            for i in range(len(span_r)):
                # airfoil properties
                airfoil_id = int(span_airfoil[i])
                airfoil_name = 'Airfoils//' + airfoils_db[airfoil_id]
                airfoil = pd.read_csv(airfoil_name, skiprows=9)
                airfoil['Cl_Cd'] = airfoil['Cl']/airfoil['Cd']
                cl_opt = airfoil.loc[airfoil['Cl_Cd'].argmax(), 'Cl']
                alpha_opt = airfoil.loc[airfoil['Cl_Cd'].argmax(), 'Alpha']
                
                # root_chord
                if i==0:
                    phi_opt = degrees(atan(2.0/(3.0 * tsr_root)))
                    root_chord = (8*pi*rotor_radius*sin(radians(phi_opt)))/(3*blade_number*cl_opt*tsr)
                
                phi_opt = degrees(atan(2.0/(3.0 * tsr_r[i])))
                
                chord = (8*pi*rotor_radius*sin(radians(phi_opt)))/(3*blade_number*cl_opt*tsr)
                twist = phi_opt - alpha_opt
                
                span_chord = np.append(span_chord, chord)
                span_twist = np.append(span_twist, twist) 
        
        # pitch the blade to make tip twist angle as 0
        if(adjust_pitch):
            pitch = span_twist[-1]
            span_twist = np.subtract(span_twist, pitch)
        else:
            pitch = 0
            
        # blade node thickness
        dr = 2.0 * (span_r[0] - hub_radius)
        span_dr = np.array(dr)    # first annulus
        
        for i in range(1, len(span_r)):
            dr = 2.0 * (span_r[i] - (span_r[i-1] + span_dr[-1]/2.0))
            span_dr = np.append(span_dr, dr)
        
        outputs['span_dr'] = span_dr
        outputs['span_chord'] = span_chord
        outputs['span_twist'] = span_twist
        outputs['root_chord'] = 3.4 #root_chord
        outputs['pitch'] = pitch
        
        
        
        
                

  
        
        
        
        

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
        self.add_input('span_dr', units='m', desc='list of blade annulus thickness', shape=num_nodes)
        self.add_input('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        self.add_input('span_twist', units='deg',desc='list of blade node twist angle', shape=num_nodes)
        self.add_input('span_airfoil', desc='list of blade node Airfoil ID', shape=num_nodes)
     
        # outputs
        self.add_output('span_power', units='W', desc='list of spanwise power', shape=num_nodes)
        self.add_output('span_thrust', units='N', desc='list of spanwise thrust', shape=num_nodes)
        self.add_output('span_fx', units='N/m', desc='list of spanwise normal force to the plane of rotation', shape=num_nodes)
        self.add_output('span_fy', units='N/m', desc='list of spanwise tangential force to the plane of rotation', shape=num_nodes)
        
 
 
    def compute(self, inputs, outputs):
        
        wind_speed = inputs['wind_speed'] 
        blade_number = inputs['blade_number']
        rotor_diameter = inputs['rotor_diameter']
        rotor_speed = inputs['rotor_speed']
        hub_radius = inputs['hub_radius']
        pitch = inputs['pitch']
        span_r = inputs['span_r']
        span_dr = inputs['span_dr']
        span_chord = inputs['span_chord']
        span_twist = inputs['span_twist']
        span_airfoil = inputs['span_airfoil']
        
        rotor_radius = rotor_diameter/2.0
        tsr = (rotor_speed * 2 * pi / 60.0) * rotor_radius/wind_speed
        
        result = bem_rotor(wind_speed, blade_number, rotor_radius, hub_radius, tsr, pitch, \
                span_r, span_dr, span_chord, span_twist, span_airfoil, is_prandtl=1, is_glauert=1)
        
        
        
        # Plotting
        params = ['chord', 'twist', 'alpha', 'phi', 'aA', 'aT', 'cl', 'cd', 'cp', 'ct']
        for param in params:
            # save the plots
            plt.close('all')
            plt.plot(np.array(result['r']), np.array(result[param]))
            plt.xlabel('Span [m]')
            plt.ylabel(param)
            #plt.show()
            plt.savefig('Plots/' + param + '.png')
        
        
        outputs['span_power'] = np.array(result['power'])
        outputs['span_thrust'] = np.array(result['thrust'])
        outputs['span_fx'] = np.array(result['fx'])
        outputs['span_fy'] = np.array(result['fy'])
        
        


class RotorAero(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('wind_speed', units = 'm/s', desc = 'wind speed')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        self.add_input('span_power', units='W', desc='list of spanwise power', shape=num_nodes)
        self.add_input('span_thrust',  units='N', desc='list of spanwise thrust', shape=num_nodes)
        
        
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
        span_power = inputs['span_power']
        span_thrust = inputs['span_thrust']
        
        rotor_radius = rotor_diameter/2.0
        swept_area = pi * rotor_radius**2
        omega = rotor_speed * (2*pi/60.0)
        tsr = omega*rotor_radius/wind_speed
        
        power = np.sum(span_power)
        torque = power/omega
        thrust = np.sum(span_thrust)
        
        cp = power/(0.5 * rho_air * (wind_speed ** 3) * swept_area)
        cq = cp/tsr
        ct = np.sum(thrust)/(0.5 * rho_air * (wind_speed ** 2) * swept_area) 
        
        outputs['rotor_tsr'] = tsr
        outputs['swept_area'] = swept_area
        outputs['rotor_cp'] = cp
        outputs['rotor_cq'] = cq
        outputs['rotor_ct'] = ct
        outputs['rotor_power'] = power
        outputs['rotor_torque'] = torque
        outputs['rotor_thrust'] = thrust
        
        
        # Generate aerodynamic power curve
        
        
        
        
        


        
        
        
        
        
class RotorAeroAdder(Group):
    def setup(self):
        
        self.add_subsystem('wind', HubWindSpeed(), \
                           promotes_inputs = ['pot_wind_speed', 'hub_height'], \
                           promotes_outputs=[('hub_wind_speed', 'wind_speed')])
        
        
        self.add_subsystem('profile', BladeDesign(), \
                           promotes_inputs = ['blade_number', 'rotor_diameter', 'rotor_speed', 'hub_radius', 'span_r', 'span_airfoil', \
                                              'chord_params', 'twist_params', 'adjust_pitch'], \
                           promotes_outputs =['span_dr', 'span_chord', 'span_twist',  'root_chord', 'pitch'])
             
        self.add_subsystem('annulus', AnnulusAero(), \
                           promotes_inputs = ['blade_number', 'rotor_diameter', 'rotor_speed', 'hub_radius', \
                                              'span_r', 'span_airfoil'], \
                           promotes_outputs=['span_fx', 'span_fy']
                           )
        
        self.add_subsystem('rotor', RotorAero(), \
                           promotes_inputs = ['rotor_diameter', 'rotor_speed'], \
                           promotes_outputs=['*'])
        
#         self.add_subsystem('hub', HubAero(), \
#                            promotes_inputs = ['hub_radius', 'span_r'], \
#                            promotes_outputs=['root_bending_moment', 'root_force'])
        
        
        self.connect('wind_speed', ['profile.wind_speed', 'annulus.wind_speed', 'rotor.wind_speed']) #, 'hub.wind_speed']) 
        self.connect('span_dr', ['annulus.span_dr']) #, 'hub.span_dr']) 
        self.connect('span_chord', 'annulus.span_chord') 
        self.connect('span_twist', 'annulus.span_twist')  
        self.connect('pitch', 'annulus.pitch')  
        self.connect('annulus.span_power', 'rotor.span_power')  
        self.connect('annulus.span_thrust', 'rotor.span_thrust')  
#         self.connect('span_fx', 'hub.span_fx')  
#         self.connect('span_fy', 'hub.span_fy') 
        
        
        
           
        





        
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
        i.add_output('adjust_pitch', desc = 'adjust pitch for tip twist to be 0')
        i.add_output('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        i.add_output('chord_params', desc='list of blade node chord length', shape=degree_blade_param)
        i.add_output('twist_params', desc='list of blade node twist angle', shape=degree_blade_param)
        i.add_output('span_airfoil', desc='list of blade node Airfoil ID', shape=num_nodes)

                
        self.add_subsystem('dof', i)   
        self.add_subsystem('aero', RotorAeroAdder())
               
        # parameters
        self.connect('dof.pot_wind_speed', 'aero.pot_wind_speed')
        self.connect('dof.hub_height', 'aero.hub_height')
        self.connect('dof.blade_number', 'aero.blade_number')
        self.connect('dof.rotor_diameter', 'aero.rotor_diameter')
        self.connect('dof.rotor_speed', 'aero.rotor_speed')
        self.connect('dof.hub_radius', 'aero.hub_radius')
        self.connect('dof.adjust_pitch', 'aero.adjust_pitch')
        self.connect('dof.span_r', 'aero.span_r')
        self.connect('dof.chord_params', 'aero.chord_params')
        self.connect('dof.twist_params', 'aero.twist_params')
        self.connect('dof.span_airfoil', 'aero.span_airfoil')
        
        
        


if __name__ == "__main__":
    
    from openmdao.api import Problem, view_model
     
    start = time()
    
    # get and set values of the variables using Problem
    prob = Problem(RotorAeroGroupTest())
    prob.setup()
    view_model(prob, outfile='rotor_aero_siemens.html')
    
    prob['dof.pot_wind_speed'] = 8.3 
    prob['dof.hub_height'] = 80.0
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 108.0
    prob['dof.rotor_speed'] = 16.0
    prob['dof.hub_radius'] = 4.0
    prob['dof.adjust_pitch'] = 1
    prob['dof.span_r'] = [8.0, 16.0, 25.0, 35.0, 47.0]
    #prob['dof.span_dr'] = [2.0, 8.0, 10.0, 10.0, 10.0]
    prob['dof.chord_params'] = [0.0, 0.0, 0.0] #[3.5, 2.8, 1.7, 1.3, 1.0]
    prob['dof.twist_params'] = [0.0, 0.0, 0.0] #[10.5, 5.8, 0.1, -2.2, -3.5]
    prob['dof.span_airfoil'] = [1, 1, 1, 1, 1] 
     
     
    prob.run_model()
     
    print "Rotor Aerodynamics"
    print 'wind_speed = ' + str(prob['aero.wind_speed'])
    print 'rotor_tsr = ' + str(prob['aero.rotor_tsr'])
    print 'swept_area = ' + str(prob['aero.swept_area'])
    print 'rotor_cp = ' + str(prob['aero.rotor_cp'])
    print 'rotor_cq = ' + str(prob['aero.rotor_cq'])
    print 'rotor_ct = ' + str(prob['aero.rotor_ct'])
    print 'rotor_power = ' + str(prob['aero.rotor_power'])
    print 'rotor_torque = ' + str(prob['aero.rotor_torque'])
    print 'rotor_thrust = ' + str(prob['aero.rotor_thrust'])
    print 'pitch = ' + str(prob['aero.pitch'])
    print 'span_dr = ' + str(prob['aero.span_dr'])
    print 'span_chord = ' + str(prob['aero.span_chord'])
    print 'span_twist = ' + str(prob['aero.span_twist'])
    print 'span_fx = ' + str(prob['aero.span_fx'])
    print 'span_fy = ' + str(prob['aero.span_fy'])
    print 'root_chord = ' + str(prob['aero.root_chord'])
 
  
    print 'Done in ' + str(time() - start) + ' seconds'


 