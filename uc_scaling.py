import numpy as np
from time import time, clock
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil
import matplotlib.pyplot as plt

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model

from hub_adder import HubAdder, NacelleAdder
from rotor import Rotor
from cost import RNACost
from fixed_parameters import num_airfoils, num_nodes, beautify, plots_folder, degree_bezier


#############################################################################
################################  WRAPPER ###################################
#############################################################################
class RNAWrapper(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('in_rotor_diameter', units='m', desc='rotor diameter')
    
        # outputs
        self.add_output('design_tsr', desc='design tip speed ratio')
        self.add_output('blade_number', desc='number of blades')
        self.add_output('rotor_diameter', units='m', desc='rotor diameter')
        self.add_output('hub_radius', units = 'm', desc = 'hub radius')
        self.add_output('root_chord', units = 'm', desc = 'length of root chord') 
        self.add_output('chord_coefficients', units = 'm', desc = 'coefficients of polynomial chord profile', shape=degree_bezier)
        self.add_output('twist_coefficients', units = 'deg', desc = 'coefficients of polynomial twist profile', shape=degree_bezier)       
        self.add_output('span_airfoil_r', units='m', desc='list of blade node radial location at which the airfoils are specified', shape=num_airfoils)
        self.add_output('span_airfoil_id', desc='list of blade node Airfoil ID', shape=num_airfoils)
        self.add_output('adjust_pitch', desc = 'adjust pitch for tip twist to be 0')
        self.add_output('thickness_factor', desc='scaling factor for laminate thickness')
        #self.add_output('wind_speed', units = 'm/s', desc = 'hub wind speed')
        self.add_output('hub_height', units = 'm', desc = 'hub radius')
        self.add_output('precone', units='deg', desc='blade precone angle')
        self.add_output('yaw', units = 'deg', desc = 'rotor yaw misalignment angle')
        self.add_output('overhang', units='m', desc='overhang distance')
        self.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        self.add_output('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        self.add_output('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        self.add_output('machine_rating', units='kW', desc='machine rating')
        self.add_output('gear_ratio', desc='overall gearbox ratio')
        self.add_output('crane', desc='flag for presence of crane')
        self.add_output('shaft_ratio', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
        self.add_output('Np', desc='number of planets in each stage', shape=3)
        self.add_output('shrink_disc_mass', units='kg', desc='Mass of the shrink disc')
        self.add_output('carrier_mass', units='kg', desc='Carrier mass')
        self.add_output('flange_length', units='m', desc='flange length')
        self.add_output('L_rb', units='m', desc='distance between hub center and upwind main bearing')
        self.add_output('gearbox_cm_x', units = 'm', desc = 'distance from tower-top center to gearbox cm--negative for upwind')
        self.add_output('tower_top_diameter', units='m', desc='diameter of tower top')
        self.add_output('hss_length', units = 'm', desc = 'optional high speed shaft length determined by user')

    def compute(self, inputs, outputs):

        rotor_diameter = inputs['in_rotor_diameter']       
        s = rotor_diameter/126.0
        rating = 5000.0 * (s**2)
        span_af =   [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
        span_afid = [0,     1,     2,     3,     4,     5,     6,     7]


        # outputs  
        outputs['design_tsr'] = 7.0
        outputs['blade_number'] = 3
        outputs['rotor_diameter'] = rotor_diameter
        outputs['hub_radius'] = 1.5 * s
        outputs['root_chord'] = 3.542 * s
        outputs['chord_coefficients'] = np.ones(degree_bezier)
        outputs['twist_coefficients'] = np.ones(degree_bezier)
        outputs['span_airfoil_r'] = np.array([x*s for x in span_af]).reshape(num_airfoils)
        outputs['span_airfoil_id'] = span_afid
        outputs['adjust_pitch'] = 1
        outputs['thickness_factor'] = 1.0 #inputs['in_tf']
        outputs['hub_height'] = 90.0
        outputs['precone'] = -2.5    
        outputs['yaw'] = 0.0 
        outputs['overhang'] = 5.0191 * s
        outputs['shaft_angle'] = -5.0
        outputs['cut_in_speed'] = 3.0
        outputs['cut_out_speed'] = 25.0    
        outputs['machine_rating'] = rating  
        outputs['gear_ratio'] = 97.0
        outputs['crane'] = 1
        outputs['shaft_ratio'] = 0.1
        outputs['Np'] =  np.array([3.0,3.0,1.0,])
        outputs['shrink_disc_mass'] = (rating * s**2) /3.0
        outputs['carrier_mass'] = 8000.0
        outputs['flange_length'] = 0.5
        outputs['overhang'] = 5.0191 * s
        outputs['L_rb'] = 1.912 * s
        outputs['gearbox_cm_x'] = 0.1
        outputs['tower_top_diameter'] = 3.87 * s
        outputs['hss_length'] = 1.5 * s

        
#############################################################################
################################  WORKFLOW  #################################
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
                           #promotes_inputs=['rotor_diameter', 'blade_number', 'machine_rating'], \
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
        
        self.connect('blade_mass', 'hub.blade_mass')
        self.connect('MB1_location', 'hub.MB1_location')
        
        self.connect('rotor_mass', 'cost.rotor_mass')
        self.connect('hub_system_mass', 'cost.hub_system_mass')
        self.connect('nacelle_mass', 'cost.nacelle_mass')






#############################################################################
##############################  UNIT TESTING ################################
############################################################################# 
class RNATest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        i.add_output('in_rotor_diameter', units='m', desc='rotor diameter')

        # parameters
        self.add_subsystem('dof', i, promotes=['*'])        
        self.add_subsystem('rna', RNA(), promotes_inputs=['*'])

    def test(self):
    
        start = time()
        
        # workflow setup
        prob = Problem(self)
        prob.setup()
        
        td = []
        flap = []
        grav = []
        mb_mass = []
        rotor_mass = []
        gb_mass = []
        
        # reference turbine
        prob['dof.in_rotor_diameter'] = 126.0
        prob.run_model()
        td_ref = prob['rna.tip_deflection'][0]
        stress_flap_ref = prob['rna.span_stress_flap'][0]
        stress_grav_ref = prob['rna.span_stress_gravity'][0]
        gb_ref = prob['rna.gearbox_mass'][0]
        mb_ref = prob['rna.main_bearing_mass'][0]
        rotor_ref = prob['rna.rotor_mass'][0]
                
        # run for a range of diameters
        d_range = range(100,151)
        for d in d_range:
            prob['dof.in_rotor_diameter'] = d
             
            prob.run_model()
            
            print d, prob['rna.main_bearing_mass'][0], prob['rna.rotor_mass'][0], prob['rna.gearbox_mass'][0], \
            prob['rna.span_stress_gravity'][0], prob['rna.span_stress_flap'][0], prob['rna.tip_deflection'][0]
            
            td.append(prob['rna.tip_deflection'][0])
            flap.append(prob['rna.span_stress_flap'][0])
            grav.append(prob['rna.span_stress_gravity'][0])
            mb_mass.append(prob['rna.main_bearing_mass'][0])
            rotor_mass.append(prob['rna.rotor_mass'][0])
            gb_mass.append(prob['rna.gearbox_mass'][0])
    
    
        d = np.array(d_range)
        td = np.array([x/td_ref for x in td]) # np.array([x/4.9584 for x in td])
        flap = np.array([x/stress_flap_ref for x in flap]) # np.array([x/67923595.1004 for x in flap])
        grav = np.array([x/stress_grav_ref for x in grav]) # np.array([x/35473276.5686 for x in grav])
        mb = np.array([x/mb_ref for x in mb_mass]) # np.array([x/1345.243 for x in mb_mass])
        rotor = np.array([x/rotor_ref for x in rotor_mass]) # np.array([x/51092.6639 for x in rotor_mass])
        gb = np.array([x/gb_ref for x in gb_mass]) # np.array([x/35653.1846 for x in gb_mass])
        
        plt.close('all')    
        plt.xlabel('Rotor diameter [m]')
        plt.ylabel('Normalized wrt reference [-]')
         
        plt.plot(d, td, linestyle='-', label='Tip deflection')
        plt.hold(True)
         
        plt.plot(d, flap, marker='o', label='Flapwise stress at root')
        plt.hold(True)
         
        plt.plot(d, grav, linestyle='--', label='Gravity stress at root')
        plt.hold(True)
        
        #plt.plot(d, mb, linestyle='-.', label='Main bearing mass')
        #plt.hold(True)
         
        plt.plot(d, rotor, linestyle=':', label='Rotor mass')
        plt.hold(True)
         
        plt.plot(d, gb, marker='.', label='Gearbox mass')
        plt.hold(True)
         
        plt.legend(loc='best')
        plt.show()
        #plt.savefig(plots_folder + 'uc_scaling_aerodyn.png')
      
        print 'Done in ' + str(time() - start) + ' seconds'

        
        
   
    
if __name__ == "__main__":
    RNATest().test()        
        
        
        
        