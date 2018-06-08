import numpy as np
from time import time, clock
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil
import matplotlib.pyplot as plt

from openmdao.api import ExplicitComponent, ExecComp, Group, IndepVarComp, Problem, view_model

from hub_adder import HubAdder, NacelleAdder
from rotor import Rotor
from cost import RNACost
from fixed_parameters import num_airfoils, num_nodes, beautify, plots_folder, degree_bezier


#############################################################################
################################  WRAPPER ###################################
#############################################################################
class RNAWrapper(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('in_rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('in_tf', desc='scaling factor for laminate thickness')
        #self.add_input('in_chord_coefficients', desc = 'coefficients of polynomial chord profile', shape=degree_bezier)
        #self.add_input('in_twist_coefficients', desc = 'coefficients of polynomial twist profile', shape=degree_bezier)

    
        # returns
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
        rating = 5000.0 # * (s**2)
        span_af =   [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
        span_afid = [0,     1,     2,     3,     4,     5,     6,     7]


        # outputs  
        outputs['design_tsr'] = 7.0
        outputs['blade_number'] = 3
        outputs['rotor_diameter'] = rotor_diameter
        outputs['hub_radius'] = 1.5 * s
        outputs['root_chord'] = 3.542 * s
        outputs['chord_coefficients'] = np.ones(degree_bezier) # [x*y for x,y in zip(inputs['in_chord_coefficients'], [ 3.4, 3.4, 2.5])] 
        outputs['twist_coefficients'] = np.ones(degree_bezier) # [x*y for x,y in zip(inputs['in_twist_coefficients'], [22.5, 5.4, 2.3])]
        outputs['span_airfoil_r'] = np.array([x*s for x in span_af]).reshape(num_airfoils)
        outputs['span_airfoil_id'] = span_afid
        outputs['adjust_pitch'] = 1
        outputs['thickness_factor'] = inputs['in_tf'] #* (1.5 - 0.5) + 0.5
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
        outputs['L_rb'] = 1.912 * s
        outputs['gearbox_cm_x'] = 0.1
        outputs['tower_top_diameter'] = 3.87 * s
        outputs['hss_length'] = 1.5 * s
        
        #print 'Diameter = ' + beautify(outputs['rotor_diameter'])
        #print 'TF = ' + beautify(outputs['thickness_factor'])
        #print '-'*10
        
        
        


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
        
        self.add_subsystem('mass', ExecComp('rna_mass = hub_mass + nacelle_mass + rotor_mass', \
                                            rna_mass={'units' : 'kg'}, \
                                            hub_mass={'units' : 'kg'}, \
                                            nacelle_mass={'units' : 'kg'}, \
                                            rotor_mass={'units' : 'kg'}), \
                           promotes_outputs=['rna_mass'])
        
        
        self.add_subsystem('cost', RNACost(), \
                           #promotes_inputs=['rotor_diameter', 'blade_number', 'machine_rating'], \
                           promotes_inputs=['machine_rating'], \
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
        
        self.connect('rotor_mass', 'mass.rotor_mass')
        self.connect('hub_system_mass', 'mass.hub_mass')
        self.connect('nacelle_mass', 'mass.nacelle_mass')
        
        self.connect('rotor_mass', 'cost.rotor_mass')
        self.connect('hub_mass', 'cost.hub_mass')
        self.connect('pitch_system_mass', 'cost.pitch_system_mass')
        self.connect('spinner_mass', 'cost.spinner_mass')
        self.connect('low_speed_shaft_mass', 'cost.low_speed_shaft_mass')
        self.connect('main_bearing_mass', 'cost.main_bearing_mass')
        self.connect('gearbox_mass', 'cost.gearbox_mass')
        #self.connect('generator_mass', 'cost.generator_mass')
        self.connect('high_speed_side_mass', 'cost.high_speed_side_mass')
        #self.connect('vs_electronics_mass', 'cost.vs_electronics_mass')
        self.connect('yaw_system_mass', 'cost.yaw_system_mass')
        self.connect('mainframe_mass', 'cost.mainframe_mass')
        #self.connect('electrical_mass', 'cost.electrical_mass')
        #self.connect('hvac_mass', 'cost.hvac_mass')
        self.connect('cover_mass', 'cost.cover_mass')
        #self.connect('controls_mass', 'cost.controls_mass')






#############################################################################
##############################  UNIT TESTING ################################
############################################################################# 
class RNATest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        i.add_output('in_rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('in_tf', desc='scaling factor for laminate thickness')
        #i.add_output('in_chord_coefficients', desc = 'coefficients of polynomial chord profile', shape=degree_bezier)
        #i.add_output('in_twist_coefficients', desc = 'coefficients of polynomial twist profile', shape=degree_bezier)         
        
        # parameters
        self.add_subsystem('dof', i, promotes=['*'])        
        self.add_subsystem('rna', RNA(), promotes_inputs=['*'])

    def test(self):
    
        start = time()
        
        # workflow setup
        prob = Problem(self)
        prob.setup()
        #view_model(prob, outfile='N2/rna.html')
        
        # define inputs 
        prob['in_rotor_diameter'] = 126.0
        prob['in_tf'] = 1.0
        
        prob.run_model()
        
        print 'span_r = ' + beautify(prob['rna.span_r'])
        print 'span_chord = ' + beautify(prob['rna.span_chord'])
        print 'span_twist = ' + beautify(prob['rna.span_twist'])
        print 'pitch = ' + beautify(prob['rna.pitch'])
        print 'rotor_cp = ' + str(prob['rna.rotor_cp'])
        print 'rotor_ct = ' + str(prob['rna.rotor_ct'])
        print 'rotor_thrust = ' + str(prob['rna.rotor_thrust'])
        print 'rated_wind_speed = ' + beautify(prob['rna.rated_wind_speed'])
        print 'rated_tip_speed = ' + beautify(prob['rna.rated_tip_speed'])
        print 'wind_bin = ' + beautify(prob['rna.wind_bin'])
        print 'power_bin = ' + beautify(prob['rna.power_bin'])
        print 'span_stress_max = ' + str(max(prob['rna.span_stress_max']))
        print 'tip_deflection = ' + beautify(prob['rna.tip_deflection'])
        print 'rotor_mass = ' + beautify(prob['rna.rotor_mass'])
        print 'hub_system_mass = ' + beautify(prob['rna.hub_system_mass'])
        print 'nacelle_mass = ' + beautify(prob['rna.nacelle_mass'])
        print 'gb_mass = ' + beautify(prob['rna.gearbox_mass'])
        print 'rna_mass = ' + str(prob['rna.rotor_mass'][0] + prob['rna.hub_system_mass'][0] + prob['rna.nacelle_mass'][0])
        print 'rna_mass = ' + beautify(prob['rna.rna_mass'])
        print 'rna_cost = ' + beautify(prob['rna.cost_rna_total'])

        print 'Done in ' + str(time() - start) + ' seconds'
        
        # plots
        fig = plt.figure()
        
        x1 = fig.add_subplot(121)
        x1.set_title('Chord distribution')
        x1.set_xlabel('r [m]')
        x1.set_ylabel('Chord [m]')
        x1.plot(prob['rna.span_r'], prob['rna.span_chord'])
        
        x2 = fig.add_subplot(122)
        x2.set_title('Twist distribution')
        x2.set_xlabel('r [m]')
        x2.set_ylabel('Twist [deg]')
        x2.plot(prob['rna.span_r'], prob['rna.span_twist'])
        
        plt.show()
        
          

        
        
   
    
if __name__ == "__main__":
    RNATest().test()
    
           