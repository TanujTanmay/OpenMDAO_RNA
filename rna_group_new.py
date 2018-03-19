from hub_adder import HubAdder, NacelleAdder
from rotor_aero_bem import RotorAeroAdder
from rotor_structures import RotorStructAdder
from cost import RNACost

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model

import numpy as np
from time import time, clock
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil

from fixed_parameters import degree_blade_param, num_nodes

class RNA_SE(Group):
    def setup(self):
        
        self.add_subsystem('aero', RotorAeroAdder(), \
                           promotes_inputs=['*'], \
                           promotes_outputs=['*'])
        
        self.add_subsystem('struct', RotorStructAdder(), \
                           promotes_inputs=['blade_number', 'rotor_diameter', 'shaft_angle', 'hub_radius', 'span_r', 'span_thickness'], \
                           promotes_outputs=['*'])
        
        self.add_subsystem('nacelle', NacelleAdder(), \
                           promotes_inputs=['rotor_diameter', 'rotor_speed', 'machine_rating', 'gear_ratio', 'crane', 'shaft_angle', \
                                            'shaft_ratio', 'Np', 'shrink_disc_mass', 'carrier_mass', 'flange_length', 'overhang', 'L_rb', \
                                            'gearbox_cm_x', 'tower_top_diameter', 'hss_length'], \
                           promotes_outputs=['*'])        
        
        
        self.add_subsystem('hub', HubAdder(), \
                           promotes_inputs=['rotor_diameter', 'blade_number', 'machine_rating', \
                                            'L_rb', 'shaft_angle'], \
                           promotes_outputs=['*'])
        
        
        self.add_subsystem('cost', RNACost(), \
                           promotes_inputs=['rotor_diameter', 'blade_number', 'machine_rating'], \
                           promotes_outputs=['*'])
        
        
        
        # connections
        self.connect('wind_speed', 'struct.wind_speed')
        self.connect('rotor_thrust', 'struct.rotor_thrust')
        self.connect('span_dr', 'struct.span_dr')  
        self.connect('span_chord', 'struct.span_chord')  
        self.connect('span_fx', 'struct.span_fx') 
        self.connect('span_fy', 'struct.span_fy') 
        
        self.connect('rotor_torque', 'nacelle.rotor_torque') 
        self.connect('rotor_thrust', 'nacelle.rotor_thrust') 
        self.connect('rotor_mass', 'nacelle.rotor_mass') 
        self.connect('root_bending_moment', 'nacelle.rotor_bending_moment_x', src_indices=[0]) 
        self.connect('root_bending_moment', 'nacelle.rotor_bending_moment_y', src_indices=[1]) 
        self.connect('root_bending_moment', 'nacelle.rotor_bending_moment_z', src_indices=[2]) 
        self.connect('root_force', 'nacelle.rotor_force_x', src_indices=[0]) 
        self.connect('root_force', 'nacelle.rotor_force_y', src_indices=[1]) 
        self.connect('root_force', 'nacelle.rotor_force_z', src_indices=[2]) 
        
        self.connect('root_chord', 'hub.blade_root_diameter')
        self.connect('blade_mass', ['hub.blade_mass', 'cost.blade_mass'])
        self.connect('root_bending_moment', 'hub.rotor_bending_moment', src_indices=[0])
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
        i.add_output('pot_wind_speed', units = 'm/s', desc = 'potential wind speed at reference height of 10m')
        i.add_output('hub_height', units = 'm', desc = 'hub radius')
        i.add_output('blade_number', desc='number of blades')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        i.add_output('span_airfoil', desc='list of blade node Airfoil ID', shape=num_nodes)
        i.add_output('span_thickness', units='m', desc='spanwise blade thickness', shape=num_nodes)
        i.add_output('chord_params', desc='list of blade node chord length', shape=degree_blade_param)
        i.add_output('twist_params', desc='list of blade node twist angle', shape=degree_blade_param)
        i.add_output('adjust_pitch', desc = 'adjust pitch for tip twist to be 0')
        i.add_output('machine_rating', units='kW', desc='machine rating of generator')
        i.add_output('gear_ratio', desc='overall gearbox ratio')
        i.add_output('crane', desc='flag for presence of crane')
        i.add_output('shaft_angle', units = 'deg', desc = 'shaft tilt')
        i.add_output('shaft_ratio', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
        i.add_output('Np', desc='number of planets in each stage', shape=3)
        i.add_output('shrink_disc_mass', units='kg', desc='Mass of the shrink disc')
        i.add_output('carrier_mass', units='kg', desc='Carrier mass')
        i.add_output('flange_length', units='m', desc='flange length')
        i.add_output('overhang', units = 'm', desc = 'overhang')
        i.add_output('L_rb', units='m', desc='distance between hub center and upwind main bearing')
        i.add_output('gearbox_cm_x', units = 'm', desc = 'distance from tower-top center to gearbox cm--negative for upwind')
        i.add_output('tower_top_diameter', units='m', desc='diameter of tower top')
        i.add_output('hss_length', units = 'm', desc = 'optional high speed shaft length determined by user')
         
        
        # parameters
        self.add_subsystem('dof', i)        
        self.add_subsystem('rna', RNA_SE())
        
        
        self.connect('dof.pot_wind_speed', 'rna.pot_wind_speed')
        self.connect('dof.hub_height', 'rna.hub_height')
        self.connect('dof.blade_number', 'rna.blade_number')
        self.connect('dof.rotor_diameter', 'rna.rotor_diameter')
        self.connect('dof.rotor_speed', 'rna.rotor_speed')
        self.connect('dof.hub_radius', 'rna.hub_radius')
        self.connect('dof.span_r', 'rna.span_r')
        self.connect('dof.span_airfoil', 'rna.span_airfoil')
        self.connect('dof.span_thickness', 'rna.span_thickness')
        self.connect('dof.chord_params', 'rna.chord_params')
        self.connect('dof.twist_params', 'rna.twist_params')
        self.connect('dof.adjust_pitch', 'rna.adjust_pitch')
        self.connect('dof.machine_rating', 'rna.machine_rating')
        self.connect('dof.gear_ratio', 'rna.gear_ratio')
        self.connect('dof.crane', 'rna.crane')
        self.connect('dof.shaft_angle', 'rna.shaft_angle')
        self.connect('dof.shaft_ratio', 'rna.shaft_ratio')
        self.connect('dof.Np', 'rna.Np')
        self.connect('dof.shrink_disc_mass', 'rna.shrink_disc_mass')
        self.connect('dof.carrier_mass', 'rna.carrier_mass')
        self.connect('dof.flange_length', 'rna.flange_length')
        self.connect('dof.overhang', 'rna.overhang')
        self.connect('dof.L_rb', 'rna.L_rb')
        self.connect('dof.gearbox_cm_x', 'rna.gearbox_cm_x')
        self.connect('dof.tower_top_diameter', 'rna.tower_top_diameter')
        self.connect('dof.hss_length', 'rna.hss_length')





def Test1():
    
    from openmdao.api import Problem, view_model
      
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RNA_usecase())
    prob.setup()
#     view_model(prob, outfile='rna_uc3.html')
#      
#     prob['dof.pot_wind_speed'] = 8.0 
#     prob['dof.hub_height'] = 90.0
#     prob['dof.blade_number'] = 3
#     prob['dof.rotor_diameter'] = 100.0
#     prob['dof.rotor_speed'] = 15.8
#     prob['dof.hub_radius'] = 10.0
#     prob['dof.span_r'] = [11.0, 16.0, 25.0, 35.0, 45.0]
#     prob['dof.span_airfoil'] = [2, 2, 2, 2, 2] 
#     prob['dof.span_thickness'] = [2.0, 1.0, 0.75, 0.5, 0.35]
#     prob['dof.chord_params'] = [0.0, 0.0, 0.0] #[3.5, 2.8, 1.7, 1.3, 1.0]
#     prob['dof.twist_params'] = [0.0, 0.0, 0.0] #[10.5, 5.8, 0.1, -2.2, -3.5]
#     prob['dof.adjust_pitch'] = 1  
#     prob['dof.machine_rating'] = 2300.
#     prob['dof.gear_ratio'] = 96.76
#     prob['dof.crane'] = 1
#     prob['dof.shaft_angle'] = -5.0
#     prob['dof.shaft_ratio'] = 0.1
#     prob['dof.Np'] = np.array([3.0,3.0,1.0,])
#     prob['dof.shrink_disc_mass'] = 1666.5
#     prob['dof.carrier_mass'] = 8000.
#     prob['dof.flange_length'] = 0.5
#     prob['dof.overhang'] = 5.
#     prob['dof.L_rb'] = 1.912
#     prob['dof.gearbox_cm_x'] = 0.0
#     prob['dof.tower_top_diameter'] = 3.78
#     prob['dof.hss_length'] = 0.
#     
#     
#      
#      
#     prob.run_model()
#      
#     print "My Rotor"
#     print prob['rna.rotor_power'] # 1580223.
#     print prob['rna.rotor_cp'] # 0.3131954
#     print prob['rna.rotor_ct'] # 1.134795
#     print prob['rna.rotor_tsr'] # 10.34011
#     print prob['rna.span_dr'] # 1.11918757e+08
#     print prob['rna.span_chord'] # 6583456.3
#     print prob['rna.span_twist'] # 1.85633554e-17
#     print prob['rna.root_chord'] # 1.85633554e-17
#     print prob['rna.span_fx'] # 1.58230231
#     print prob['rna.span_fy'] # 58500.
#     print prob['rna.root_bending_moment'] # 46332.44963208
#     print prob['rna.root_force'] # 3.74
#     print prob['rna.span_moment_flap'] # 42361.26557694
#     print prob['rna.span_moment_edge'] # 46332.44963208
#     print prob['rna.span_stress_flap'] # 3.74
#     print prob['rna.span_stress_edge'] # 42361.26557694
#     print prob['rna.span_stress_gravity'] # 42361.26557694
#     
#     print prob['rna.hub_diameter'] # 42361.26557694
#     print prob['rna.tip_deflection'] # 42361.26557694
#     print prob['rna.blade_mass'] # 46332.44963208
#     print prob['rna.rotor_mass'] # 3.74
#     print prob['rna.nacelle_mass'] # 42361.26557694
#     print prob['rna.hub_mass'] # 42361.26557694
#     
#     print prob['rna.hub_system_mass'] # 3.74
#     print prob['rna.low_speed_shaft_mass'] # 42361.26557694
#     print prob['rna.main_bearing_mass'] # 42361.26557694
#     
#     print prob['rna.gearbox_mass'] # 3.74
#     print prob['rna.high_speed_side_mass'] # 42361.26557694
#     print prob['rna.generator_mass'] # 42361.26557694
#     print prob['rna.bedplate_mass'] # 3.74
#     print prob['rna.yaw_system_mass'] # 42361.26557694
#     
#     print prob['rna.cost_rotor']
#     print prob['rna.cost_hub']
#     print prob['rna.cost_pitch']
#     print prob['rna.cost_spinner']
#     print prob['rna.cost_lss']
#     print prob['rna.cost_bearing']
#     print prob['rna.cost_gearbox']
#     print prob['rna.cost_generator']
#     print prob['rna.cost_hss']
#     print prob['rna.cost_vs_electronics']
#     print prob['rna.cost_yaw']
#     print prob['rna.cost_mainframe']
#     print prob['rna.cost_electrical']
#     print prob['rna.cost_hvac']
#     print prob['rna.cost_cover']
#     print prob['rna.cost_controls']
#     print prob['rna.cost_rna_total']
#     
#     
#   
#     print 'Done in ' + str(time() - start) + ' seconds'
    
    
    
    
    
def TestSiemens():
    
    from openmdao.api import Problem, view_model
     
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RNA_usecase())
    prob.setup()
    view_model(prob, outfile='rna_siemens2.html')
     
    prob['dof.pot_wind_speed'] = 8.3
    prob['dof.hub_height'] = 80.0
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 108.0
    prob['dof.rotor_speed'] = 16.0
    prob['dof.hub_radius'] = 2.0
    prob['dof.span_r'] = [7.0, 16.0, 25.0, 35.0, 47.0]
    prob['dof.span_airfoil'] = [1, 1, 1, 1, 1] 
    prob['dof.span_thickness'] = [2.0, 1.0, 0.75, 0.5, 0.35]
    prob['dof.chord_params'] = [0.0, 0.0, 0.0] #[3.5, 2.8, 1.7, 1.3, 1.0]
    prob['dof.twist_params'] = [0.0, 0.0, 0.0] #[10.5, 5.8, 0.1, -2.2, -3.5]
    prob['dof.adjust_pitch'] = 1  
    prob['dof.machine_rating'] = 2300.
    prob['dof.gear_ratio'] = 91.0
    prob['dof.crane'] = 0
    prob['dof.shaft_angle'] = 0.0 #-6.0
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
    
    
     
     
    prob.run_model()
     
    print "Siemens SWT-2.3-108"
    print 'wind_speed = ' + str(prob['rna.wind_speed'])
    print 'rotor_tsr = ' + str(prob['rna.rotor_tsr'])
    print 'swept_area = ' + str(prob['rna.swept_area'])
    print 'rotor_cp = ' + str(prob['rna.rotor_cp'])
    print 'rotor_cq = ' + str(prob['rna.rotor_cq'])
    print 'rotor_ct = ' + str(prob['rna.rotor_ct'])
    print 'rotor_power = ' + str(prob['rna.rotor_power'])
    print 'rotor_torque = ' + str(prob['rna.rotor_torque'])
    print 'rotor_thrust = ' + str(prob['rna.rotor_thrust'])
    print 'root_bending_moment = ' + str(prob['rna.root_bending_moment'])
    print 'root_force = ' + str(prob['rna.root_force'])
    print 'pitch = ' + str(prob['rna.pitch'])
    print 'span_dr = ' + str(prob['rna.span_dr'])
    print 'span_chord = ' + str(prob['rna.span_chord'])
    print 'span_twist = ' + str(prob['rna.span_twist'])
    print 'root_chord = ' + str(prob['rna.root_chord'])
    print 'hub_diameter = ' + str(prob['rna.hub_diameter'])
    print 'span_fx = ' + str(prob['rna.span_fx'])
    print 'span_fy = ' + str(prob['rna.span_fy'])
    print 'span_moment_flap = ' + str(prob['rna.span_moment_flap'])
    print 'span_moment_edge = ' + str(prob['rna.span_moment_edge'])
    print 'span_stress_flap = ' + str(prob['rna.span_stress_flap'])
    print 'span_stress_edge = ' + str(prob['rna.span_stress_edge'])
    print 'span_stress_gravity = ' + str(prob['rna.span_stress_gravity'])
    print 'tip_deflection = ' + str(prob['rna.tip_deflection'])
    print 'hub_mass = ' + str(prob['rna.hub_mass'])
    print 'pitch_system_mass = ' + str(prob['rna.pitch_system_mass'])
    print 'spinner_mass = ' + str(prob['rna.spinner_mass'])
    print 'low_speed_shaft_mass = ' + str(prob['rna.low_speed_shaft_mass'])
    print 'main_bearing_mass = ' + str(prob['rna.main_bearing_mass'])
    print 'gearbox_mass = ' + str(prob['rna.gearbox_mass'])
    print 'generator_mass = ' + str(prob['rna.generator_mass'])
    print 'high_speed_side_mass = ' + str(prob['rna.high_speed_side_mass'])
    print 'vs_electronics_mass = ' + str(prob['rna.vs_electronics_mass'])
    print 'yaw_system_mass = ' + str(prob['rna.yaw_system_mass'])
    print 'mainframe_mass = ' + str(prob['rna.mainframe_mass'])
    print 'electrical_mass = ' + str(prob['rna.electrical_mass'])
    print 'hvac_mass = ' + str(prob['rna.hvac_mass'])
    print 'cover_mass = ' + str(prob['rna.cover_mass'])
    print 'controls_mass = ' + str(prob['rna.controls_mass'])
    print 'hub_system_mass = ' + str(prob['rna.hub_system_mass'])
    print 'nacelle_mass = ' + str(prob['rna.nacelle_mass'])
    print 'blade_mass = ' + str(prob['rna.blade_mass'])
    print 'rotor_mass = ' + str(prob['rna.rotor_mass'])
    print 'cost_rotor = ' + str(prob['rna.cost_rotor'])
    print 'cost_hub = ' + str(prob['rna.cost_hub'])
    print 'cost_pitch = ' + str(prob['rna.cost_pitch'])
    print 'cost_spinner = ' + str(prob['rna.cost_spinner'])
    print 'cost_lss = ' + str(prob['rna.cost_lss'])
    print 'cost_bearing = ' + str(prob['rna.cost_bearing'])
    print 'cost_gearbox = ' + str(prob['rna.cost_gearbox'])
    print 'cost_generator = ' + str(prob['rna.cost_generator'])
    print 'cost_hss = ' + str(prob['rna.cost_hss'])
    print 'cost_vs_electronics = ' + str(prob['rna.cost_vs_electronics'])
    print 'cost_yaw = ' + str(prob['rna.cost_yaw'])
    print 'cost_mainframe = ' + str(prob['rna.cost_mainframe'])
    print 'cost_electrical = ' + str(prob['rna.cost_electrical'])
    print 'cost_hvac = ' + str(prob['rna.cost_hvac'])
    print 'cost_cover = ' + str(prob['rna.cost_cover'])
    print 'cost_controls = ' + str(prob['rna.cost_controls'])
    print 'cost_rna_total = ' + str(prob['rna.cost_rna_total'])

    
    
  
    print 'Done in ' + str(time() - start) + ' seconds'    
    
    
    
    
if __name__ == "__main__":
    TestSiemens()    
    
           