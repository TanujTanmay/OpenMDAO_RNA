from hub_adder import HubAdder, NacelleAdder
from rotor import RotorScaled
from cost import RNACost
import matplotlib.pyplot as plt

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model

import numpy as np
from time import time, clock
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil

from fixed_parameters import num_airfoils, num_nodes, beautify

class RNA_SE(Group):
    def setup(self):
        
        self.add_subsystem('rotor', RotorScaled(), \
                           promotes_inputs=['*'], \
                           promotes_outputs=['*'])
        
        self.add_subsystem('nacelle', NacelleAdder(), \
                           promotes_inputs=['rotor_diameter', 'rotor_speed', 'machine_rating', 'gear_ratio', 'crane', 'shaft_angle', \
                                            'shaft_ratio', 'Np', 'shrink_disc_mass', 'carrier_mass', 'flange_length', 'overhang', 'L_rb', \
                                            'gearbox_cm_x', 'tower_top_diameter', 'hss_length'], \
                           promotes_outputs=['*'])        
        
        
        self.add_subsystem('hub', HubAdder(), \
                           promotes_inputs=['rotor_diameter', 'blade_number', 'machine_rating', \
                                            'L_rb', 'shaft_angle', ('blade_root_diameter', 'root_chord')], \
                           promotes_outputs=['*'])
        
        
        self.add_subsystem('cost', RNACost(), \
                           promotes_inputs=['rotor_diameter', 'blade_number', 'machine_rating'], \
                           promotes_outputs=['*'])
        
        
        
        # connections
        
        self.connect('rotor_torque', 'nacelle.rotor_torque') 
        self.connect('rotor_thrust', 'nacelle.rotor_thrust') 
        self.connect('rotor_mass', 'nacelle.rotor_mass') 
        self.connect('rotor_moment', ['nacelle.rotor_bending_moment_x', 'hub.rotor_bending_moment'], src_indices=[0]) 
        self.connect('rotor_moment', 'nacelle.rotor_bending_moment_y', src_indices=[1]) 
        self.connect('rotor_moment', 'nacelle.rotor_bending_moment_z', src_indices=[2]) 
        self.connect('rotor_force', 'nacelle.rotor_force_x', src_indices=[0]) 
        self.connect('rotor_force', 'nacelle.rotor_force_y', src_indices=[1]) 
        self.connect('rotor_force', 'nacelle.rotor_force_z', src_indices=[2]) 
        
        #self.connect('root_chord', 'hub.blade_root_diameter')
        self.connect('blade_mass', ['hub.blade_mass', 'cost.blade_mass'])
        #self.connect('root_bending_moment', 'hub.rotor_bending_moment', src_indices=[0])
        self.connect('MB1_location', 'hub.MB1_location')
        
        self.connect('hub_mass', 'cost.hub_mass')
        self.connect('pitch_system_mass', 'cost.pitch_system_mass')
        self.connect('spinner_mass', 'cost.spinner_mass')
        self.connect('low_speed_shaft_mass', 'cost.low_speed_shaft_mass')
        self.connect('main_bearing_mass', 'cost.main_bearing_mass')
        self.connect('gearbox_mass', 'cost.gearbox_mass')
        self.connect('generator_mass', 'cost.generator_mass')
        self.connect('high_speed_side_mass', 'cost.high_speed_side_mass')
        self.connect('vs_electronics_mass', 'cost.vs_electronics_mass')
        self.connect('yaw_system_mass', 'cost.yaw_system_mass')
        self.connect('mainframe_mass', 'cost.mainframe_mass')
        self.connect('electrical_mass', 'cost.electrical_mass')
        self.connect('hvac_mass', 'cost.hvac_mass')
        self.connect('cover_mass', 'cost.cover_mass')
        self.connect('controls_mass', 'cost.controls_mass')
                
        



class RNA_usecase(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        i.add_output('design_tsr', desc='design tip speed ratio')
        i.add_output('blade_number', desc='number of blades')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('root_chord', units = 'm', desc = 'length of root chord')        
        i.add_output('span_airfoil_r', units='m', desc='list of blade node radial location at which the airfoils are specified', shape=num_airfoils)
        i.add_output('span_airfoil_id', desc='list of blade node Airfoil ID', shape=num_airfoils)
        i.add_output('adjust_pitch', desc = 'adjust pitch for tip twist to be 0')
        i.add_output('thickness_factor', desc='scaling factor for laminate thickness')
        i.add_output('wind_speed', units = 'm/s', desc = 'hub wind speed')
        i.add_output('hub_height', units = 'm', desc = 'hub radius')
        i.add_output('precone', units='deg', desc='blade precone angle')
        i.add_output('yaw', units = 'deg', desc = 'rotor yaw misalignment angle')
        i.add_output('overhang', units='m', desc='overhang distance')
        i.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        i.add_output('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        i.add_output('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        i.add_output('machine_rating', units='kW', desc='machine rating')
        i.add_output('gear_ratio', desc='overall gearbox ratio')
        i.add_output('crane', desc='flag for presence of crane')
        #i.add_output('shaft_angle', units = 'deg', desc = 'shaft tilt')
        i.add_output('shaft_ratio', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
        i.add_output('Np', desc='number of planets in each stage', shape=3)
        i.add_output('shrink_disc_mass', units='kg', desc='Mass of the shrink disc')
        i.add_output('carrier_mass', units='kg', desc='Carrier mass')
        i.add_output('flange_length', units='m', desc='flange length')
        #i.add_output('overhang', units = 'm', desc = 'overhang')
        i.add_output('L_rb', units='m', desc='distance between hub center and upwind main bearing')
        i.add_output('gearbox_cm_x', units = 'm', desc = 'distance from tower-top center to gearbox cm--negative for upwind')
        i.add_output('tower_top_diameter', units='m', desc='diameter of tower top')
        i.add_output('hss_length', units = 'm', desc = 'optional high speed shaft length determined by user')
         
        
        # parameters
        self.add_subsystem('dof', i, promotes=['*'])        
        self.add_subsystem('rna', RNA_SE(), promotes_inputs=['*'])
        
        
#         self.connect('dof.pot_wind_speed', 'rna.pot_wind_speed')
#         self.connect('dof.hub_height', 'rna.hub_height')
#         self.connect('dof.blade_number', 'rna.blade_number')
#         self.connect('dof.rotor_diameter', 'rna.rotor_diameter')
#         self.connect('dof.rotor_speed', 'rna.rotor_speed')
#         self.connect('dof.hub_radius', 'rna.hub_radius')
#         self.connect('dof.span_r', 'rna.span_r')
#         self.connect('dof.span_airfoil', 'rna.span_airfoil')
#         self.connect('dof.span_thickness', 'rna.span_thickness')
#         self.connect('dof.chord_params', 'rna.chord_params')
#         self.connect('dof.twist_params', 'rna.twist_params')
#         self.connect('dof.adjust_pitch', 'rna.adjust_pitch')
#         self.connect('dof.machine_rating', 'rna.machine_rating')
#         self.connect('dof.gear_ratio', 'rna.gear_ratio')
#         self.connect('dof.crane', 'rna.crane')
#         self.connect('dof.shaft_angle', 'rna.shaft_angle')
#         self.connect('dof.shaft_ratio', 'rna.shaft_ratio')
#         self.connect('dof.Np', 'rna.Np')
#         self.connect('dof.shrink_disc_mass', 'rna.shrink_disc_mass')
#         self.connect('dof.carrier_mass', 'rna.carrier_mass')
#         self.connect('dof.flange_length', 'rna.flange_length')
#         self.connect('dof.overhang', 'rna.overhang')
#         self.connect('dof.L_rb', 'rna.L_rb')
#         self.connect('dof.gearbox_cm_x', 'rna.gearbox_cm_x')
#         self.connect('dof.tower_top_diameter', 'rna.tower_top_diameter')
#         self.connect('dof.hss_length', 'rna.hss_length')



def TestNREL():
    
    from openmdao.api import Problem, view_model
     
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RNA_usecase())
    prob.setup()
    view_model(prob, outfile='N2/rna_nrel.html')
     
    prob['dof.design_tsr'] = 7.0
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 126.0
    prob['dof.hub_radius'] = 1.5
    prob['dof.root_chord'] = 3.4
    prob['dof.span_airfoil_r'] =  [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
    prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]
    prob['dof.adjust_pitch'] = 1    
    prob['dof.thickness_factor'] = 1.0
    prob['dof.wind_speed'] = 12.2
    prob['dof.hub_height'] = 90.0
    prob['dof.precone'] = -2.5
    prob['dof.yaw'] = 0.0
    #prob['dof.overhang'] = -5.0191
    prob['dof.shaft_angle'] = -5.0
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 5000.0
    
    prob['dof.gear_ratio'] = 97.0
    prob['dof.crane'] = 1
    prob['dof.shaft_ratio'] = 0.1
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.shrink_disc_mass'] = 5000.0/3.0
    prob['dof.carrier_mass'] = 8000.
    prob['dof.flange_length'] = 0.5
    prob['dof.overhang'] = 5.
    prob['dof.L_rb'] = 1.912
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.tower_top_diameter'] = 3.87
    prob['dof.hss_length'] = 1.5
    
    
     
     
    prob.run_model()
     
    print "NREL 5 MW"
    print 'rated_wind_speed = ' + beautify(prob['rna.rated_wind_speed'])
    print 'swept_area = ' + beautify(prob['rna.swept_area'])
    print 'rotor_cp = ' + str(prob['rna.rotor_cp'])
    print 'rotor_cq = ' + str(prob['rna.rotor_cq'])
    print 'rotor_ct = ' + str(prob['rna.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['rna.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['rna.rotor_torque'])
    print 'rotor_thrust = ' + beautify(prob['rna.rotor_thrust'])
    print 'pitch = ' + beautify(prob['rna.pitch'])
    print 'span_r = ' + beautify(prob['rna.span_r'])
    print 'span_airfoil = ' + beautify(prob['rna.span_airfoil'])
    print 'span_dr = ' + beautify(prob['rna.span_dr'])
    print 'span_chord = ' + beautify(prob['rna.span_chord'])
    print 'span_twist = ' + beautify(prob['rna.span_twist'])
    print 'span_mass = ' + beautify(prob['rna.span_mass'])
    print 'hub_diameter = ' + beautify(prob['rna.hub_diameter'])
    print 'span_fx = ' + beautify(prob['rna.span_fx'])
    print 'span_fy = ' + beautify(prob['rna.span_fy'])
    print 'span_moment_flap = ' + beautify(prob['rna.span_moment_flap'])
    print 'span_moment_edge = ' + beautify(prob['rna.span_moment_edge'])
    print 'span_stress_flap = ' + beautify(prob['rna.span_stress_flap'])
    print 'span_stress_edge = ' + beautify(prob['rna.span_stress_edge'])
    print 'span_stress_gravity = ' + beautify(prob['rna.span_stress_gravity'])
    print 'tip_deflection = ' + beautify(prob['rna.tip_deflection'])
    print 'hub_system_mass = ' + beautify(prob['rna.hub_system_mass'])
    print 'nacelle_mass = ' + beautify(prob['rna.nacelle_mass'])
    print 'rotor_mass = ' + beautify(prob['rna.rotor_mass'])
    print 'cost_rna_total = ' + beautify(prob['rna.cost_rna_total'])
    print 'rotor_moment = ' + beautify(prob['rna.rotor_moment'])
    print 'rotor_force = ' + beautify(prob['rna.rotor_force'])

    
    
  
    print 'Done in ' + str(time() - start) + ' seconds'

      
    
    
if __name__ == "__main__":
    TestNREL()
    
           