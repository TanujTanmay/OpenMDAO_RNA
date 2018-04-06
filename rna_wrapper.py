import numpy as np

from openmdao.api import ExplicitComponent
from fixed_parameters import num_airfoils, num_nodes, beautify


class RNAWrapper(ExplicitComponent):
    def setup(self):
        # variables
        #self.add_input('in_design_tsr', desc='design tip speed ratio')
        self.add_input('in_rotor_diameter', units='m', desc='rotor diameter')
        #self.add_input('in_machine_rating', units='kW', desc='machine rating')
        #self.add_input('in_thickness_factor', desc='scaling factor for laminate thickness')

    
        # returns
        self.add_output('design_tsr', desc='design tip speed ratio')
        self.add_output('blade_number', desc='number of blades')
        self.add_output('rotor_diameter', units='m', desc='rotor diameter')
        self.add_output('hub_radius', units = 'm', desc = 'hub radius')
        self.add_output('root_chord', units = 'm', desc = 'length of root chord')        
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
        span_af = [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]


        # outputs  
        outputs['design_tsr'] = 7.0 #inputs['in_design_tsr']
        outputs['blade_number'] = 3
        outputs['rotor_diameter'] = rotor_diameter
        outputs['hub_radius'] = 1.5 * s
        outputs['root_chord'] = 3.4 * s
        outputs['span_airfoil_r'] = np.reshape(np.array([x*s for x in span_af]), 8)
        outputs['span_airfoil_id'] = np.array([0,     1,     2,        3,     4,     5,     6,     7])
        outputs['adjust_pitch'] = 1  
        outputs['thickness_factor'] = 1.0 #inputs['in_thickness_factor']      
        outputs['hub_height'] = 90.0
        outputs['precone'] = -2.5
        outputs['yaw'] = 0.0
        outputs['shaft_angle'] = -5.0        
        outputs['cut_in_speed'] = 3.0
        outputs['cut_out_speed'] = 25.0     
        outputs['machine_rating'] = 5000.0  
        outputs['gear_ratio'] = 97.0
        outputs['crane'] = 1
        outputs['shaft_ratio'] = 0.1
        outputs['Np'] =  np.array([3.0,3.0,1.0,])
        outputs['shrink_disc_mass'] = (5000.0 * s**2) /3.0
        outputs['carrier_mass'] = 8000.0
        outputs['flange_length'] = 0.5
        outputs['overhang'] = 5.0191 * s
        outputs['L_rb'] = 1.912
        outputs['gearbox_cm_x'] = 0.1
        outputs['tower_top_diameter'] = 3.87 * s
        outputs['hss_length'] = 1.5

        
        
        
        
        
        