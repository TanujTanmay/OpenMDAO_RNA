import numpy as np
from time import time, clock
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil
import matplotlib.pyplot as plt

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model

from hub_adder import HubAdder, NacelleAdder
from rotor import Rotor
from cost import RNACost
from rna_wrapper import RNAWrapper
from fixed_parameters import num_airfoils, num_nodes, beautify


#############################################################################
################################  WORKFLOWS #################################
#############################################################################
class RNA(Group):
    def setup(self):
        
        self.add_subsystem('wrap', RNAWrapper(), \
                           promotes_inputs=['*'], \
                           promotes_outputs=['*'])
        
        self.add_subsystem('rotor', Rotor(), \
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
        self.connect('rotor_moment', ['nacelle.rotor_bending_moment_x'], src_indices=[0]) 
        self.connect('rotor_moment', ['nacelle.rotor_bending_moment_y', 'hub.rotor_bending_moment'], src_indices=[1]) 
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






#############################################################################
##############################  UNIT TESTING ################################
############################################################################# 
class RNATest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        #i.add_output('in_design_tsr', desc='design tip speed ratio')
        i.add_output('in_rotor_diameter', units='m', desc='rotor diameter')
        #i.add_output('in_thickness_factor', desc='scaling factor for laminate thickness')         
        
        # parameters
        self.add_subsystem('dof', i, promotes=['*'])        
        self.add_subsystem('rna', RNA(), promotes_inputs=['*'])

    def test(self):
    
        start = time()
        
        # workflow setup
        prob = Problem(self)
        prob.setup()
        view_model(prob, outfile='N2/rna.html')
        
        # define inputs 
        #prob['dof.in_design_tsr'] = 7.0
        prob['dof.in_rotor_diameter'] = 126.0 
        #prob['dof.in_thickness_factor'] = 1.0
        
        prob.run_model()
        
        # print outputs  
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
        print 'mb_mass = ' + beautify(prob['rna.main_bearing_mass'])
        print 'gb_mass = ' + beautify(prob['rna.gearbox_mass'])
        print 'rotor_mass = ' + beautify(prob['rna.rotor_mass'])
        print 'cost_rna_total = ' + beautify(prob['rna.cost_rna_total'])
        print 'rotor_moment = ' + beautify(prob['rna.rotor_moment'])
        print 'rotor_force = ' + beautify(prob['rna.rotor_force'])
    
        print 'Done in ' + str(time() - start) + ' seconds'    

        
        
   
    
if __name__ == "__main__":
    RNATest().test()
    
           