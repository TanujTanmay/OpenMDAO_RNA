from hub_adder import HubAdder, NacelleAdder
from rotor import Rotor
from cost import RNACost
from wind_conditions import HubWindSpeed
import matplotlib.pyplot as plt

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model

import numpy as np
from time import time, clock
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil

from fixed_parameters import num_airfoils, num_nodes, beautify

class RNA_SE(Group):
    def setup(self):
        
        self.add_subsystem('wind', HubWindSpeed(), \
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
        self.connect('root_bending_moment', ['nacelle.rotor_bending_moment_x', 'hub.rotor_bending_moment'], src_indices=[0]) 
        self.connect('root_bending_moment', 'nacelle.rotor_bending_moment_y', src_indices=[1]) 
        self.connect('root_bending_moment', 'nacelle.rotor_bending_moment_z', src_indices=[2]) 
        self.connect('root_force', 'nacelle.rotor_force_x', src_indices=[0]) 
        self.connect('root_force', 'nacelle.rotor_force_y', src_indices=[1]) 
        self.connect('root_force', 'nacelle.rotor_force_z', src_indices=[2]) 
        
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
        i.add_output('pot_wind_speed', units = 'm/s', desc = 'potential wind speed at reference height of 10m')
        i.add_output('hub_height', units = 'm', desc = 'hub radius')
        i.add_output('design_tsr', desc='design tip speed ratio')
        i.add_output('blade_number', desc='number of blades')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('root_chord', units = 'm', desc = 'length of root chord')        
        i.add_output('span_airfoil_r', units='m', desc='list of blade node radial location at which the airfoils are specified', shape=num_airfoils)
        i.add_output('span_airfoil_id', desc='list of blade node Airfoil ID', shape=num_airfoils)
        i.add_output('adjust_pitch', desc = 'adjust pitch for tip twist to be 0')
        i.add_output('thickness_factor', desc='scaling factor for laminate thickness')
        #i.add_output('wind_speed', units = 'm/s', desc = 'hub height wind speed')
        i.add_output('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        i.add_output('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        i.add_output('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        i.add_output('machine_rating', units='kW', desc='machine rating')
        i.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        
        i.add_output('gear_ratio', desc='overall gearbox ratio')
        i.add_output('crane', desc='flag for presence of crane')
        #i.add_output('shaft_angle', units = 'deg', desc = 'shaft tilt')
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
    #view_model(prob, outfile='rna_new.html')
     
    prob['dof.pot_wind_speed'] = 7.8
    prob['dof.hub_height'] = 80.0
    
    prob['dof.design_tsr'] = 7.4
    prob['dof.rotor_speed'] = 12.3
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 108.0
    prob['dof.hub_radius'] = 1.5
    prob['dof.root_chord'] = 3.4
    prob['dof.span_airfoil_r'] = [4.0, 10.5]
    prob['dof.span_airfoil_id'] = [0, 7]
    prob['dof.adjust_pitch'] = 1    
    prob['dof.thickness_factor'] = 0.465
    #prob['dof.wind_speed'] = 9.0 
    
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 2300.0/0.95
    prob['dof.shaft_angle'] = 5.0
    
    prob['dof.gear_ratio'] = 91.0
    prob['dof.crane'] = 0
    prob['dof.shaft_ratio'] = 0.1
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.shrink_disc_mass'] = 2300.0/3.0
    prob['dof.carrier_mass'] = 2500.
    prob['dof.flange_length'] = 0.3
    prob['dof.overhang'] = 4.
    prob['dof.L_rb'] = 1.0
    prob['dof.gearbox_cm_x'] = 0.0
    prob['dof.tower_top_diameter'] = 2.5
    prob['dof.hss_length'] = 0.
    
    
     
     
    prob.run_model()
     
    print "Siemens SWT-2.3-108"
    print 'wind_speed = ' + beautify(prob['rna.wind_speed'])
    print 'rated_wind_speed = ' + beautify(prob['rna.rated_wind_speed'])
    print 'rotor_tsr = ' + beautify(prob['rna.rotor_tsr'])
    print 'swept_area = ' + beautify(prob['rna.swept_area'])
    print 'rotor_cp = ' + str(prob['rna.rotor_cp'])
    print 'rotor_cq = ' + str(prob['rna.rotor_cq'])
    print 'rotor_ct = ' + str(prob['rna.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['rna.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['rna.rotor_torque'])
    print 'rotor_thrust = ' + beautify(prob['rna.rotor_thrust'])
    print 'root_bending_moment = ' + beautify(prob['rna.root_bending_moment'])
    print 'root_force = ' + beautify(prob['rna.root_force'])
    print 'pitch = ' + beautify(prob['rna.pitch'])
    print 'span_dr = ' + beautify(prob['rna.span_dr'])
    print 'span_chord = ' + beautify(prob['rna.span_chord'])
    print 'span_twist = ' + beautify(prob['rna.span_twist'])
    #print 'root_chord = ' + beautify(prob['rna.root_chord'])
    print 'hub_diameter = ' + beautify(prob['rna.hub_diameter'])
    print 'span_fx = ' + beautify(prob['rna.span_fx'])
    print 'span_fy = ' + beautify(prob['rna.span_fy'])
    print 'span_moment_flap = ' + beautify(prob['rna.span_moment_flap'])
    print 'span_moment_edge = ' + beautify(prob['rna.span_moment_edge'])
    print 'span_stress_flap = ' + beautify(prob['rna.span_stress_flap'])
    print 'span_stress_edge = ' + beautify(prob['rna.span_stress_edge'])
    print 'span_stress_gravity = ' + beautify(prob['rna.span_stress_gravity'])
    print 'tip_deflection = ' + beautify(prob['rna.tip_deflection'])
#     print 'hub_mass = ' + beautify(prob['rna.hub_mass'])
#     print 'pitch_system_mass = ' + beautify(prob['rna.pitch_system_mass'])
#     print 'spinner_mass = ' + beautify(prob['rna.spinner_mass'])
#     print 'low_speed_shaft_mass = ' + beautify(prob['rna.low_speed_shaft_mass'])
#     print 'main_bearing_mass = ' + beautify(prob['rna.main_bearing_mass'])
#     print 'gearbox_mass = ' + beautify(prob['rna.gearbox_mass'])
#     print 'generator_mass = ' + beautify(prob['rna.generator_mass'])
#     print 'high_speed_side_mass = ' + beautify(prob['rna.high_speed_side_mass'])
#     print 'vs_electronics_mass = ' + beautify(prob['rna.vs_electronics_mass'])
#     print 'yaw_system_mass = ' + beautify(prob['rna.yaw_system_mass'])
#     print 'mainframe_mass = ' + beautify(prob['rna.mainframe_mass'])
#     print 'electrical_mass = ' + beautify(prob['rna.electrical_mass'])
#     print 'hvac_mass = ' + beautify(prob['rna.hvac_mass'])
#     print 'cover_mass = ' + beautify(prob['rna.cover_mass'])
#     print 'controls_mass = ' + beautify(prob['rna.controls_mass'])
    print 'hub_system_mass = ' + beautify(prob['rna.hub_system_mass'])
    print 'nacelle_mass = ' + beautify(prob['rna.nacelle_mass'])
#     print 'blade_mass = ' + beautify(prob['rna.blade_mass'])
    print 'rotor_mass = ' + beautify(prob['rna.rotor_mass'])
#     print 'cost_rotor = ' + beautify(prob['rna.cost_rotor'])
#     print 'cost_hub = ' + beautify(prob['rna.cost_hub'])
#     print 'cost_pitch = ' + beautify(prob['rna.cost_pitch'])
#     print 'cost_spinner = ' + beautify(prob['rna.cost_spinner'])
#     print 'cost_lss = ' + beautify(prob['rna.cost_lss'])
#     print 'cost_bearing = ' + beautify(prob['rna.cost_bearing'])
#     print 'cost_gearbox = ' + beautify(prob['rna.cost_gearbox'])
#     print 'cost_generator = ' + beautify(prob['rna.cost_generator'])
#     print 'cost_hss = ' + beautify(prob['rna.cost_hss'])
#     print 'cost_vs_electronics = ' + beautify(prob['rna.cost_vs_electronics'])
#     print 'cost_yaw = ' + beautify(prob['rna.cost_yaw'])
#     print 'cost_mainframe = ' + beautify(prob['rna.cost_mainframe'])
#     print 'cost_electrical = ' + beautify(prob['rna.cost_electrical'])
#     print 'cost_hvac = ' + beautify(prob['rna.cost_hvac'])
#     print 'cost_cover = ' + beautify(prob['rna.cost_cover'])
#     print 'cost_controls = ' + beautify(prob['rna.cost_controls'])
    print 'cost_rna_total = ' + beautify(prob['rna.cost_rna_total'])

    
    
  
    print 'Done in ' + str(time() - start) + ' seconds'    
    
    
    
    
    
    
    
    
def TestVestas():
    
    from openmdao.api import Problem, view_model
     
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RNA_usecase())
    prob.setup()
    #view_model(prob, outfile='rna_new.html')
     
    prob['dof.pot_wind_speed'] = 8.3
    prob['dof.hub_height'] = 90.0
    
    prob['dof.design_tsr'] = 8.1
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 120.0
    prob['dof.hub_radius'] = 2.3
    prob['dof.root_chord'] = 4.2
    prob['dof.span_airfoil_r'] = [4.0, 12.0]
    prob['dof.span_airfoil_id'] = [0, 1]
    prob['dof.adjust_pitch'] = 1    
    prob['dof.thickness_factor'] = 0.5
    #prob['dof.wind_speed'] = 9.0 
    prob['dof.rotor_speed'] = 13.0
    prob['dof.cut_in_speed'] = 4.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 3600.0
    prob['dof.shaft_angle'] = 0.0
    
    prob['dof.gear_ratio'] = 119.0
    prob['dof.crane'] = 0
    prob['dof.shaft_ratio'] = 0.1
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.shrink_disc_mass'] = 3600.0/3.0
    prob['dof.carrier_mass'] = 5000.
    prob['dof.flange_length'] = 0.3
    prob['dof.overhang'] = 4.
    prob['dof.L_rb'] = 1.5
    prob['dof.gearbox_cm_x'] = 0.0
    prob['dof.tower_top_diameter'] = 2.3
    prob['dof.hss_length'] = 0.
    
    
     
     
    prob.run_model()
     
    print "Siemens SWT-2.3-108"
    print 'wind_speed = ' + beautify(prob['rna.wind_speed'])
    print 'rated_wind_speed = ' + beautify(prob['rna.rated_wind_speed'])
    print 'rotor_tsr = ' + beautify(prob['rna.rotor_tsr'])
    print 'swept_area = ' + beautify(prob['rna.swept_area'])
    print 'rotor_cp = ' + beautify(prob['rna.rotor_cp'])
    print 'rotor_cq = ' + beautify(prob['rna.rotor_cq'])
    print 'rotor_ct = ' + beautify(prob['rna.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['rna.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['rna.rotor_torque'])
    print 'rotor_thrust = ' + beautify(prob['rna.rotor_thrust'])
    print 'root_bending_moment = ' + beautify(prob['rna.root_bending_moment'])
    print 'root_force = ' + beautify(prob['rna.root_force'])
    print 'pitch = ' + beautify(prob['rna.pitch'])
    print 'span_dr = ' + beautify(prob['rna.span_dr'])
    print 'span_chord = ' + beautify(prob['rna.span_chord'])
    print 'span_twist = ' + beautify(prob['rna.span_twist'])
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

    
    
  
    print 'Done in ' + str(time() - start) + ' seconds'       
    
    
    
    
    
    
def TestNREL():
    
    from openmdao.api import Problem, view_model
     
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RNA_usecase())
    prob.setup()
    view_model(prob, outfile='rna_nrel.html')
     
    prob['dof.pot_wind_speed'] = 9.1 #11.4
    prob['dof.hub_height'] = 90.0
    
    prob['dof.design_tsr'] = 7.0
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 126.0
    prob['dof.hub_radius'] = 1.5
    prob['dof.root_chord'] = 3.4
    prob['dof.span_airfoil_r'] =  [1.5, 7.9, 10.5, 14.3, 22.0, 27.1, 34.8, 42.5]
    prob['dof.span_airfoil_id'] = [0,   1,   2,    3,    4,    5,    6,    7 ]
    prob['dof.adjust_pitch'] = 1    
    prob['dof.thickness_factor'] = 1.0
    #prob['dof.wind_speed'] = 9.0 
    prob['dof.rotor_speed'] = 11.9
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 5000.0/0.944
    prob['dof.shaft_angle'] = 5.0
    
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
    print 'wind_speed = ' + beautify(prob['rna.wind_speed'])
    print 'rated_wind_speed = ' + beautify(prob['rna.rated_wind_speed'])
    print 'rotor_tsr = ' + beautify(prob['rna.rotor_tsr'])
    print 'swept_area = ' + beautify(prob['rna.swept_area'])
    print 'rotor_cp = ' + str(prob['rna.rotor_cp'])
    print 'rotor_cq = ' + str(prob['rna.rotor_cq'])
    print 'rotor_ct = ' + str(prob['rna.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['rna.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['rna.rotor_torque'])
    print 'rotor_thrust = ' + beautify(prob['rna.rotor_thrust'])
    print 'root_bending_moment = ' + beautify(prob['rna.root_bending_moment'])
    print 'root_force = ' + beautify(prob['rna.root_force'])
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
    print 'root_bending_moment = ' + beautify(prob['rna.root_bending_moment'])
    print 'root_force = ' + beautify(prob['rna.root_force'])

    
    
  
    print 'Done in ' + str(time() - start) + ' seconds'   
    
    
    
    
def Scaling():
    
    from openmdao.api import Problem, view_model
     
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RNA_usecase())
    prob.setup()
    #view_model(prob, outfile='rna_new.html')
    
    span_af = [1.5, 7.9, 10.5, 14.3, 22.0, 27.1, 34.8, 42.5]
     
    prob['dof.pot_wind_speed'] = 9.1 #11.4
    prob['dof.hub_height'] = 90.0
    
    prob['dof.design_tsr'] = 7.0
    prob['dof.blade_number'] = 3
    prob['dof.hub_radius'] = 1.5
    
    
    prob['dof.span_airfoil_id'] = [0,   1,   2,    3,    4,    5,    6,    7 ]
    prob['dof.adjust_pitch'] = 1    
    prob['dof.thickness_factor'] = 1.0
    
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    
    prob['dof.shaft_angle'] = 0.0
    
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
    
    td = []
    flap = []
    grav = []
    mb_mass = []
    rotor_mass = []
    gb_mass = []
    
    for d in range(100,150):
        prob['dof.rotor_diameter'] = d
        s = d/126.0
        prob['dof.machine_rating'] = (5000.0/0.944) * s**2
        prob['dof.rotor_speed'] = 11.9/s
        prob['dof.root_chord'] = 3.4*s
        prob['dof.span_airfoil_r'] =  [x*s for x in span_af]
         
        prob.run_model()
        
        #print prob['rna.tip_deflection']
        #print prob['rna.span_stress_flap'][0]
        #print prob['rna.span_stress_gravity'][0], prob['rna.span_stress_flap'][0], prob['rna.tip_deflection'][0]
        print d, prob['rna.main_bearing_mass'][0], prob['rna.rotor_mass'][0], prob['rna.gearbox_mass'][0], \
        prob['rna.span_stress_gravity'][0], prob['rna.span_stress_flap'][0], prob['rna.tip_deflection'][0]
        
        td.append(prob['rna.tip_deflection'][0])
        flap.append(prob['rna.span_stress_flap'][0])
        grav.append(prob['rna.span_stress_gravity'][0])
        mb_mass.append(prob['rna.main_bearing_mass'][0])
        rotor_mass.append(prob['rna.rotor_mass'][0])
        gb_mass.append(prob['rna.gearbox_mass'][0])


    d = np.array(range(100,150))
    td = np.array([x/5.79 for x in td])
    flap = np.array([x/4325.0 for x in flap])
    grav = np.array([x/27354.67 for x in grav])
    mb = np.array([x/2301.86 for x in mb_mass])
    rotor = np.array([x/51141.26 for x in rotor_mass])
    gb = np.array([x/39363.14 for x in gb_mass])
    
    plt.close('all')    
    plt.xlabel('Rotor diameter [m]')
    plt.ylabel('Normalized wrt reference [-]')
     
    plt.plot(d, td, linestyle='-', label='Tip deflection')
    plt.hold(True)
     
    plt.plot(d, flap, marker='o', label='Flapwise stress at root')
    plt.hold(True)
     
    plt.plot(d, grav, linestyle='--', label='Gravity stress at root')
    plt.hold(True)
    
    plt.plot(d, mb, linestyle='-.', label='Main bearing mass')
    plt.hold(True)
     
    plt.plot(d, rotor, linestyle=':', label='Rotor mass')
    plt.hold(True)
     
    plt.plot(d, gb, marker='.', label='Gearbox mass')
    plt.hold(True)
     
    plt.legend(loc='best')
    plt.show()
    plt.savefig('Plots/scaling.png')
  
    print 'Done in ' + str(time() - start) + ' seconds'        
    



def SA():
    
    from openmdao.api import Problem, view_model
     
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RNA_usecase())
    prob.setup()
    #view_model(prob, outfile='rna_new.html')
    
    prob['dof.span_airfoil_r'] = [1.5, 10.0]
    prob['dof.span_airfoil_id'] = [0,   7 ]
    prob['dof.rotor_diameter'] = 100.0 
    prob['dof.pot_wind_speed'] = 9.1 #11.4
    prob['dof.hub_height'] = 90.0
    prob['dof.root_chord'] = 3.4
    
    
    prob['dof.hub_radius'] = 1.5
    
    
    
    prob['dof.adjust_pitch'] = 1    
    prob['dof.thickness_factor'] = 1.0
    prob['dof.machine_rating'] = (5000.0/0.944)
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    
    prob['dof.shaft_angle'] = 0.0
    
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
    
    
    cp2 = []
    cp3 = []
    
    for b in range(2,4):
        prob['dof.blade_number'] = b
        for tsr in range(5,15):
            prob['dof.design_tsr'] = tsr
            prob['dof.rotor_speed'] = (tsr*11.1*60)/(50.0*2*pi)
            
            prob.run_model()
            
            if b==2:
                cp2.append(prob['rna.rotor_cp'][0])
            else:
                cp3.append(prob['rna.rotor_cp'][0])
            
            #print prob['rna.tip_deflection']
            #print prob['rna.span_stress_flap'][0]
            print prob['dof.blade_number'], prob['dof.design_tsr'][0], prob['dof.rotor_speed'][0], prob['rna.rotor_cp'][0]


    tsr = np.array(range(5,15))
    cp2 = np.array(cp2)
    cp3 = np.array(cp3)
    
    plt.close('all')    
    plt.xlabel('Tip speed ratio [-]')
    plt.ylabel('Power coefficient [-]')
     
    plt.plot(tsr, cp2, label='2 Blades')
    plt.hold(True)
     
    plt.plot(tsr, cp3, label='3 Blades')
    plt.hold(True)

    plt.legend(loc='best')
    plt.show()
    plt.savefig('Plots/cp_lambda.png')
  
    print 'Done in ' + str(time() - start) + ' seconds'       
    
    
if __name__ == "__main__":
    Scaling()
    
           