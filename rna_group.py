from hub_adder import HubAdder, NacelleAdder
from rotor_aero import RotorAeroAdder
from rotor_struc import RotorStructAdder

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model

import numpy as np
from time import time, clock
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil

num_nodes = 6

class RNA_SE(Group):
    def setup(self):
        
        nodal_fx = []
        for node_index in range(num_nodes): # loop through each node_index, call NODE_index
            node_number = node_index + 1
            nodal_fx.append('B1N' + str(node_number) + 'Fx')
             
        self.add_subsystem('aero', RotorAeroAdder(), \
                           promotes_inputs=[('NumBlades', 'blade_number'), ('HubRad', 'hub_radius'), ('HubHt', 'hub_height'), \
                                            ('Overhang', 'overhang'), ('ShftTilt', 'shaft_angle'), ('Precone', 'precone'), \
                                            ('WndSpeed', 'wind_speed'), ('ShearExp', 'shear_exp'), \
                                            ('RotSpd', 'rotor_speed'), ('Pitch', 'pitch'), ('Yaw', 'yaw'), \
                                            'rotor_diameter', 'BlSpn', 'BlChord', 'BlTwist', 'BlAFID'], \
                           promotes_outputs=[('RtAeroPwr', 'rotor_power'), 'rotor_torque', 'rotor_thrust', \
                                             ('RtAeroCp', 'cp'), ('RtAeroCt', 'ct'), ('RtTSR', 'tsr')]) #, \
                                            # 'RtAeroFxh', ' RtAeroFyh', 'RtAeroFzh', \
                                            # 'RtAeroMxh', 'RtAeroMyh', 'RtAeroMzh'])
        
        self.add_subsystem('struc', RotorStructAdder(), \
                           promotes_inputs=['BlSpn', 'BlMass', 'BlInertia', 'root_chord', 'root_area_moment', 'blade_number', 'lifetime'], \
                           promotes_outputs=['root_stress_max', 'root_moment_max', 'miners_damage', \
                                             'tip_deflection', 'blade_mass', 'rotor_mass'])
        
        self.add_subsystem('nacelle', NacelleAdder(), \
                           promotes_inputs=['rotor_diameter', 'rotor_speed', 'machine_rating', 'gear_ratio', 'crane', 'shaft_angle', \
                                            'shaft_ratio', 'Np', 'shrink_disc_mass', 'carrier_mass', 'flange_length', 'overhang', 'L_rb', \
                                            'gearbox_cm_x', 'tower_top_diameter', 'hss_length'], \
                           promotes_outputs=['nacelle_mass',  \
                                             'low_speed_shaft_mass', 'main_bearing_mass', 'second_bearing_mass', 'gearbox_mass', \
                                             'high_speed_side_mass', 'generator_mass', 'bedplate_mass', \
                                             'yaw_system_mass', 'electrical_mass', \
                                             'vs_electronics_mass', 'hvac_mass', 'controls_mass', 'platforms_mass', \
                                             'crane_mass', 'mainframe_mass', 'cover_mass', 'above_yaw_mass'])        
        
        
        self.add_subsystem('hub', HubAdder(), \
                           promotes_inputs=['rotor_diameter', 'blade_number', 'blade_root_diameter', 'machine_rating', \
                                            'L_rb', 'shaft_angle'], \
                           promotes_outputs=['hub_diameter', 'hub_system_mass'])
        
        
        
        # connections
        for node_index in range(num_nodes): # loop through each node_index, call NODE_index
            node_number = node_index + 1
            var_name = 'B1N' + str(node_number) + 'Fx'
            self.connect('aero.' + var_name, 'struc.' + var_name)         
         
        
        self.connect('rotor_torque', 'nacelle.rotor_torque')  
        self.connect('rotor_thrust', 'nacelle.rotor_thrust') 
        self.connect('rotor_mass', 'nacelle.rotor_mass') 
        
        for x in ['x', 'y', 'z']:
            self.connect('aero.RtAeroF' + x + 'h', 'nacelle.rotor_force_' + x)
            self.connect('aero.RtAeroM' + x + 'h', 'nacelle.rotor_bending_moment_' + x) 
            
        
           
        self.connect('blade_mass', 'hub.blade_mass')
        self.connect('root_moment_max', 'hub.rotor_bending_moment')
        self.connect('nacelle.MB1_location', 'hub.MB1_location')
        



class RNA_usecase(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        i.add_output('blade_number', desc='number of blades')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius', val=1.)
        i.add_output('hub_height', units = 'm', desc = 'hub radius')
        i.add_output('overhang', units = 'm', desc = 'overhang')
        i.add_output('shaft_angle', units = 'deg', desc = 'shaft tilt')
        i.add_output('precone', units = 'deg', desc = 'blade precone')        
        i.add_output('wind_speed', units = 'm/s', desc = 'hub radius')
        i.add_output('shear_exp', desc = 'hub radius')
        i.add_output('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        i.add_output('pitch', units = 'deg', desc = 'blade pitch angle')
        i.add_output('yaw', units = 'deg', desc = 'yaw angle')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('BlSpn', units='m', desc='tuple of blade node radial location', shape=num_nodes)
        i.add_output('BlChord', units='m', desc='tuple of blade node chord length', shape=num_nodes)
        i.add_output('BlTwist', units='deg',desc='tuple of blade node twist angle', shape=num_nodes)
        i.add_output('BlAFID', desc='tuple of blade node Airfoil ID', shape=num_nodes)
        i.add_output('BlMass', units = 'kg/m', desc = 'list of blade mass per unit length for each node', shape=num_nodes)
        i.add_output('BlInertia', units = 'm**4', desc = 'list of blade node area moment of inertia', shape=num_nodes) 
        i.add_output('lifetime', units = 'year', desc = 'lifetime of the wind turbine')
        i.add_output('gear_ratio', desc='overall gearbox ratio')
        i.add_output('Np', desc='number of planets in each stage', shape=3)
        i.add_output('shrink_disc_mass', units='kg', desc='Mass of the shrink disc')
        i.add_output('carrier_mass', units='kg', desc='Carrier mass')
        i.add_output('flange_length', units='m', desc='flange length')
        i.add_output('L_rb', units='m', desc='distance between hub center and upwind main bearing')
        i.add_output('gearbox_cm_x', units = 'm', desc = 'distance from tower-top center to gearbox cm--negative for upwind')
        i.add_output('tower_top_diameter', units='m', desc='diameter of tower top')
        i.add_output('hss_length', units = 'm', desc = 'optional high speed shaft length determined by user')
        i.add_output('machine_rating', units='kW', desc='machine rating of generator')
        i.add_output('crane', desc='flag for presence of crane')
        i.add_output('shaft_ratio', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
        
        
        
        
        
        # parameters
        self.add_subsystem('dof', i)        
        self.add_subsystem('rna', RNA_SE())
        
        
        self.connect('dof.blade_number', 'rna.blade_number')
        self.connect('dof.hub_radius', 'rna.hub_radius')
        self.connect('dof.hub_height', 'rna.hub_height')
        self.connect('dof.overhang', 'rna.overhang')
        self.connect('dof.shaft_angle', 'rna.shaft_angle')
        self.connect('dof.precone', 'rna.precone')
        self.connect('dof.wind_speed', 'rna.wind_speed')
        self.connect('dof.shear_exp', 'rna.shear_exp')
        self.connect('dof.rotor_speed', 'rna.rotor_speed')
        self.connect('dof.pitch', 'rna.pitch')
        self.connect('dof.yaw', 'rna.yaw')
        self.connect('dof.rotor_diameter', 'rna.rotor_diameter')
        self.connect('dof.BlSpn', 'rna.BlSpn')
        self.connect('dof.BlChord', 'rna.BlChord')
        self.connect('dof.BlChord', ['rna.blade_root_diameter', 'rna.root_chord'], src_indices=[0]) 
        
        self.connect('dof.BlTwist', 'rna.BlTwist')
        self.connect('dof.BlAFID', 'rna.BlAFID')
        self.connect('dof.BlMass', 'rna.BlMass')
        self.connect('dof.BlInertia', 'rna.BlInertia')
        self.connect('dof.BlInertia', 'rna.root_area_moment', src_indices=[0])
        self.connect('dof.lifetime', 'rna.lifetime')
        self.connect('dof.gear_ratio', 'rna.gear_ratio')
        self.connect('dof.Np', 'rna.Np')
        self.connect('dof.shrink_disc_mass', 'rna.shrink_disc_mass')
        self.connect('dof.carrier_mass', 'rna.carrier_mass')
        self.connect('dof.flange_length', 'rna.flange_length')
        self.connect('dof.L_rb', 'rna.L_rb')
        self.connect('dof.gearbox_cm_x', 'rna.gearbox_cm_x')
        self.connect('dof.tower_top_diameter', 'rna.tower_top_diameter')
        self.connect('dof.hss_length', 'rna.hss_length')
        self.connect('dof.machine_rating', 'rna.machine_rating')
        self.connect('dof.crane', 'rna.crane')
        self.connect('dof.shaft_ratio', 'rna.shaft_ratio')





if __name__ == "__main__":
    
    from openmdao.api import Problem, view_model
     
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RNA_usecase())
    prob.setup()
    view_model(prob, outfile='rna_uc1.html')
     
    prob['dof.blade_number'] = 3
    prob['dof.hub_radius'] = 1.5
    prob['dof.hub_height'] = 90.0
    prob['dof.overhang'] = 5.0
    prob['dof.shaft_angle'] = -5.0
    prob['dof.precone'] = 0.0
    prob['dof.wind_speed'] = [10.0]
    prob['dof.shear_exp'] = [0.0]
    prob['dof.rotor_speed'] = [19.1]
    prob['dof.pitch'] = [0.0]
    prob['dof.yaw'] = [0.0]
    prob['dof.BlSpn'] = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    prob['dof.rotor_diameter'] = 100.0
    prob['dof.BlChord'] = [3.4, 3.4, 1.9, 1.3, 1.0, 0.8]
    prob['dof.BlTwist'] = [0.0, 5.9, -2.3, -5.5, -7.0, -7.9]
    prob['dof.BlAFID'] = [1, 2, 2, 2, 2, 2]
    prob['dof.BlMass'] = [750.0, 350.0, 350.0, 350.0, 350.0, 350.0]
    prob['dof.BlInertia'] = [0.10, 0.01, 0.01, 0.01, 0.01, 0.01]
    prob['dof.lifetime'] = 20    
    prob['dof.gear_ratio'] = 96.76
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.shrink_disc_mass'] = 1666.5
    prob['dof.carrier_mass'] = 8000.
    prob['dof.flange_length'] = 0.5
    prob['dof.L_rb'] = 1.912
    prob['dof.gearbox_cm_x'] = 0.0
    prob['dof.tower_top_diameter'] = 3.78
    prob['dof.hss_length'] = 0.
    prob['dof.machine_rating'] = 5000.
    prob['dof.crane'] = 1
    prob['dof.shaft_ratio'] = 0.1
    
    
     
     
    prob.run_model()
     
    print "My Rotor"
    print prob['rna.rotor_power'] # 1580223.
    print prob['rna.cp'] # 0.3131954
    print prob['rna.ct'] # 1.134795
    print prob['rna.tsr'] # 10.34011
    print prob['rna.root_stress_max'] # 1.11918757e+08
    print prob['rna.root_moment_max'] # 6583456.3
    print prob['rna.miners_damage'] # 1.85633554e-17
    print prob['rna.tip_deflection'] # 1.58230231
    print prob['rna.rotor_mass'] # 58500.
    print prob['rna.nacelle_mass'] # 46332.44963208
    print prob['rna.hub_diameter'] # 3.74
    print prob['rna.hub_system_mass'] # 42361.26557694
 
  
    print 'Done in ' + str(time() - start) + ' seconds'
    
           