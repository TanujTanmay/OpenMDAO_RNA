from openmdao.api import ExplicitComponent


class RNACost(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('blade_number', desc='number of blades')
        self.add_input('machine_rating', units = 'kW', desc = 'machine rating of turbine')
        self.add_input('blade_mass', units = 'kg', desc='mass of the one blade')
        self.add_input('hub_mass', units='kg',desc='mass of Hub')
        self.add_input('pitch_system_mass', units='kg',desc='mass of Pitch System')
        self.add_input('spinner_mass', units='kg',desc='mass of spinner')
        self.add_input('low_speed_shaft_mass', units='kg', desc='component mass')
        self.add_input('main_bearing_mass', units='kg', desc='component mass')
        self.add_input('gearbox_mass', units='kg', desc='component mass')
        self.add_input('generator_mass', units='kg', desc='component mass')
        self.add_input('high_speed_side_mass', units='kg', desc='component mass')
        self.add_input('vs_electronics_mass', units='kg', desc='component mass')
        self.add_input('yaw_system_mass', units='kg', desc='overall component mass')
        self.add_input('mainframe_mass', units='kg', desc='component mass')
        self.add_input('electrical_mass', units='kg', desc='component mass')
        self.add_input('hvac_mass', units='kg', desc='component mass')
        self.add_input('cover_mass', units='kg', desc='component mass')
        self.add_input('controls_mass', units='kg', desc='component mass')
        
        
        # outputs
        self.add_output('cost_rotor', units='USD', desc='component cost')
        self.add_output('cost_hub', units='USD', desc='component cost')
        self.add_output('cost_pitch', units='USD', desc='component cost')
        self.add_output('cost_spinner', units='USD', desc='component cost')
        self.add_output('cost_lss', units='USD', desc='component cost')
        self.add_output('cost_bearing', units='USD', desc='component cost')
        self.add_output('cost_gearbox', units='USD', desc='component cost')
        self.add_output('cost_generator', units='USD', desc='component cost')
        self.add_output('cost_hss', units='USD', desc='component cost')
        self.add_output('cost_vs_electronics', units='USD', desc='component cost')
        self.add_output('cost_yaw', units='USD', desc='component cost')
        self.add_output('cost_mainframe', units='USD', desc='component cost')
        self.add_output('cost_electrical', units='USD', desc='component cost')
        self.add_output('cost_hvac', units='USD', desc='component cost')
        self.add_output('cost_cover', units='USD', desc='component cost')
        self.add_output('cost_controls', units='USD', desc='component cost')
        self.add_output('cost_rna_total', units='USD', desc='cost of RNA assembly')
        
    def compute(self, inputs, outputs):
        
        rotor_diameter = inputs['rotor_diameter']
        blade_number = inputs['blade_number']
        machine_rating = inputs['machine_rating']
        rotor_radius = rotor_diameter/2.0
        
        BCE = 1.0 # blade material cost escalator
        GDPE = 1.0 # labor cost escalator 
         
        outputs['cost_rotor'] = blade_number *  ((0.4019*rotor_radius**3 - 955.24)*BCE + (2.7445*rotor_radius**2.5025)*GDPE)/(1-0.28)
        outputs['cost_hub'] = inputs['hub_mass'] * 4.25
        outputs['cost_pitch'] = 2.28 * (0.2106 * rotor_diameter**2.6578)
        outputs['cost_spinner'] = inputs['spinner_mass'] * 5.57
        outputs['cost_lss'] = 0.01 * rotor_diameter**2.887
        outputs['cost_bearing'] = 2 * inputs['main_bearing_mass'] * 17.6
        outputs['cost_gearbox'] = 16.45 * machine_rating ** 1.249
        outputs['cost_generator'] = machine_rating * 65.0
        outputs['cost_hss'] = 1.9894 * (machine_rating - 0.1141)
        outputs['cost_vs_electronics'] = machine_rating * 79.0
        outputs['cost_yaw'] = 2.0 * (0.0339 * rotor_diameter**2.964)
        outputs['cost_mainframe'] = 9.489 * rotor_diameter**1.953
        outputs['cost_electrical'] = machine_rating * 40.0
        outputs['cost_hvac'] = machine_rating * 12.0
        outputs['cost_cover'] = 11.537 * machine_rating + 3849.7 
        outputs['cost_controls'] = 55000.0
        
        
        outputs['cost_rna_total'] = outputs['cost_rotor'] + outputs['cost_hub'] + outputs['cost_pitch'] + \
                                    outputs['cost_spinner'] + outputs['cost_lss'] + outputs['cost_bearing'] + \
                                    outputs['cost_gearbox'] + outputs['cost_generator'] + outputs['cost_hss'] + \
                                    outputs['cost_vs_electronics'] + outputs['cost_yaw'] + outputs['cost_mainframe'] + \
                                    outputs['cost_electrical'] + outputs['cost_hvac'] + outputs['cost_cover'] + outputs['cost_controls']
                                    
        
        
        
        
        
        
        