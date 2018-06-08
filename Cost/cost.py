import pandas as pd

from openmdao.api import ExplicitComponent
from fixed_parameters import has_crane, gearbox_stages, \
                        hub_assembly_multiplier, hub_overhead_multiplier, hub_profit_multiplier, hub_transport_multiplier, \
                        nacelle_assembly_multiplier, nacelle_overhead_multiplier, nacelle_profit_multiplier, nacelle_transport_multiplier


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class ComponentCost(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('machine_rating', units = 'kW', desc = 'machine rating of turbine')
        self.add_input('rotor_diameter', units = 'm', desc = 'rotor_diameter')
        self.add_input('blade_mass', units = 'kg', desc='mass of each blade')
        self.add_input('blade_number', desc='number of blades')
        
        self.add_input('hub_mass', units='kg',desc='mass of Hub')
        self.add_input('pitch_mass', units='kg',desc='mass of Pitch System')
        self.add_input('spinner_mass', units='kg',desc='mass of spinner')
        
        self.add_input('lss_mass', units='kg', desc='component mass')
        self.add_input('main_bearing_mass', units='kg', desc='component mass')
        self.add_input('second_bearing_mass', units='kg', desc='component mass')
        self.add_input('gearbox_mass', units='kg', desc='component mass')
        self.add_input('hss_mass', units='kg', desc='component mass')
        self.add_input('generator_mass', units='kg', desc='component mass')
        self.add_input('bedplate_mass', units='kg', desc='component mass')
        self.add_input('platform_mass', units='kg', desc='component mass')
        self.add_input('crane_mass', units='kg', desc='component mass')
        self.add_input('yaw_mass', units='kg', desc='component mass')
        self.add_input('vs_electronics_mass', units='kg', desc='component mass')
        self.add_input('hvac_mass', units='kg', desc='component mass')
        self.add_input('cover_mass', units='kg', desc='component mass')
        self.add_input('transformer_mass', units='kg', desc='component mass')
        
        
        # outputs
        self.add_output('cost_blade', units='USD', desc='Cost of 1 blade')
        
        self.add_output('cost_hub', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_pitch', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_spinner', units='USD', desc='component cost', val=0.0)
        
        self.add_output('cost_lss', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_main_bearing', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_second_bearing', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_gearbox', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_hss', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_generator', units='USD', desc='component cost', val=0.0)
        #self.add_output('cost_bedplate', units='USD', desc='component cost', val=0.0)
        #self.add_output('cost_platform', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_mainframe', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_yaw', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_vs_electronics', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_hvac', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_cover', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_electrical', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_controls', units='USD', desc='component cost', val=0.0)
        self.add_output('cost_transformer', units='USD', desc='component cost', val=0.0)        
        
        self.add_output('cost_blades', units='USD', desc='Cost of all the blades')
        self.add_output('cost_hub_system', units='USD', desc='Hub System cost')
        self.add_output('cost_nacelle', units='USD', desc='Nacelle System cost')
        self.add_output('cost_rna', units='USD', desc='cost of RNA assembly')
        
        
        
    def aggregator_blades(self, outputs, blade_number):
        cost_blades = outputs['cost_blade'] * blade_number
        return cost_blades
        
    def aggregator_hub(self, outputs):    
        hub_component_cost =  outputs['cost_hub'] + outputs['cost_pitch'] + outputs['cost_spinner']
        cost_hub_system =  (1.0 + hub_transport_multiplier + hub_profit_multiplier) * \
                            ((1.0 + hub_overhead_multiplier + hub_assembly_multiplier) * hub_component_cost)
                
        return cost_hub_system        
        
    def aggregator_nacelle(self, outputs):
        nacelle_component_cost = outputs['cost_lss'] + \
                                 outputs['cost_main_bearing'] + \
                                 outputs['cost_second_bearing'] + \
                                 outputs['cost_gearbox'] + \
                                 outputs['cost_hss'] + \
                                 outputs['cost_generator'] + \
                                 outputs['cost_mainframe'] + \
                                 outputs['cost_yaw'] + \
                                 outputs['cost_vs_electronics'] + \
                                 outputs['cost_hvac'] + \
                                 outputs['cost_cover'] + \
                                 outputs['cost_electrical'] + \
                                 outputs['cost_controls'] + \
                                 outputs['cost_transformer']
                                 
        cost_nacelle = (1.0 + nacelle_transport_multiplier + nacelle_profit_multiplier) * \
                        ((1.0 + nacelle_overhead_multiplier + nacelle_assembly_multiplier) * nacelle_component_cost)
                        
        return cost_nacelle                 
        
    def aggregator_rna(self, outputs, blade_number):
        # RNA
        cost_blades = self.aggregator_blades(outputs, blade_number)
        cost_hub = self.aggregator_hub(outputs)
        cost_nacelle = self.aggregator_nacelle(outputs)
        cost_rna = cost_blades + cost_hub + cost_nacelle
        
        return [cost_blades, cost_hub, cost_nacelle, cost_rna]     
        
        
        
        
        

#############################################################################
################  MODEL#1: NREL Cost & Scaling Model ########################
#############################################################################  
class CSM(ComponentCost):
    def compute(self, inputs, outputs):
        
        machine_rating = inputs['machine_rating']
        rotor_diameter = inputs['rotor_diameter']
        blade_number = inputs['blade_number']
        rotor_radius = rotor_diameter/2.0
         
        BCE = 1.0 # blade material cost escalator
        GDPE = 1.0 # labor cost escalator 

        # NREL cost model  
        outputs['cost_blade'] = ((0.4019*rotor_radius**3 - 955.24)*BCE + (2.7445*rotor_radius**2.5025)*GDPE)/(1-0.28)
        
        outputs['cost_hub'] = inputs['hub_mass'] * 4.25
        outputs['cost_pitch'] = 2.28 * (0.2106 * rotor_diameter**2.6578)
        outputs['cost_spinner'] = inputs['spinner_mass'] * 5.57
        
        outputs['cost_lss'] = 0.01 * rotor_diameter**2.887
        outputs['cost_main_bearing'] = 2 * inputs['main_bearing_mass'] * 17.6 # 2 times because housing mass = bearing mass
        outputs['cost_second_bearing'] = 2 * inputs['second_bearing_mass'] * 17.6
        
        # gearbox
        if gearbox_stages == 0:
            # direct-drive
            cost_gearbox = 0.0
            cost_generator = machine_rating * 219.33
            cost_mainframe = 627.28 * rotor_diameter**0.85
        elif gearbox_stages == 1:
            # single-stage
            cost_gearbox = 74.1 * machine_rating ** 1.0
            cost_generator = machine_rating * 54.73
            cost_mainframe = 303.96 * rotor_diameter**1.067
        else:
            # three-stage
            cost_gearbox = 16.45 * machine_rating ** 1.249  
            cost_generator = machine_rating * 65.0
            cost_mainframe = 9.489 * rotor_diameter**1.953
                     
        outputs['cost_gearbox'] = cost_gearbox        
        outputs['cost_hss'] = 1.9894 * (machine_rating - 0.1141)
        outputs['cost_generator'] = cost_generator
        outputs['cost_mainframe'] = cost_mainframe + inputs['platform_mass']*8.7
        outputs['cost_yaw'] = 2.0 * (0.0339 * rotor_diameter**2.964)
        outputs['cost_vs_electronics'] = machine_rating * 79.0
        outputs['cost_hvac'] = machine_rating * 12.0
        outputs['cost_cover'] = 11.537 * machine_rating + 3849.7
        outputs['cost_electrical'] = machine_rating * 40.0
        outputs['cost_controls'] = 55000.0
        outputs['cost_transformer'] = inputs['transformer_mass'] * 18.8 #(3.49e-06*machine_rating**2 - 0.0221*machine_rating + 109.7)*machine_rating
        
        # aggregator
        [outputs['cost_blades'], \
         outputs['cost_hub_system'], \
         outputs['cost_nacelle'], \
         outputs['cost_rna']] = self.aggregator_rna(outputs, blade_number)




#############################################################################
######################  MODEL#2: Calibrated NREL CSM ########################
#############################################################################             
class CSMCalibrated(ComponentCost):
    def ref_cost_mass(self, component):
        RT = pd.read_csv('reference_turbine_cost_mass.csv')
        cost_mass = RT.loc[RT['Component'] == component, 'Cost'].values / RT.loc[RT['Component'] == component, 'Mass'].values
        
        return cost_mass
        
        
    def compute(self, inputs, outputs):
        
        blade_number = inputs['blade_number']
        machine_rating = inputs['machine_rating']
        mainframe_mass = inputs['bedplate_mass'] + inputs['platform_mass'] + inputs['crane_mass']
        
        outputs['cost_blade'] = inputs['blade_mass'] * self.ref_cost_mass('Blade')
        
        outputs['cost_hub'] = inputs['hub_mass'] * self.ref_cost_mass('Hub')
        outputs['cost_pitch'] = inputs['pitch_mass'] * self.ref_cost_mass('Pitch')
        outputs['cost_spinner'] = inputs['spinner_mass'] * self.ref_cost_mass('Spinner')
        
        outputs['cost_lss'] = inputs['lss_mass'] * self.ref_cost_mass('LSS')
        outputs['cost_main_bearing'] = inputs['main_bearing_mass'] * self.ref_cost_mass('Bearing')
        outputs['cost_second_bearing'] = inputs['second_bearing_mass'] * self.ref_cost_mass('Bearing')
        outputs['cost_gearbox'] = inputs['gearbox_mass'] * self.ref_cost_mass('Gearbox')
        outputs['cost_hss'] = inputs['hss_mass'] * self.ref_cost_mass('HSS')
        outputs['cost_generator'] = inputs['generator_mass'] * self.ref_cost_mass('Generator')
        outputs['cost_mainframe'] = mainframe_mass * self.ref_cost_mass('Mainframe')
        outputs['cost_yaw'] = inputs['yaw_system_mass'] * self.ref_cost_mass('Yaw')
        outputs['cost_vs_electronics'] = machine_rating * 79.0
        outputs['cost_hvac'] = machine_rating * 12.0
        outputs['cost_cover'] = inputs['cover_mass'] * self.ref_cost_mass('Cover')
        outputs['cost_electrical'] = machine_rating * 40.0
        outputs['cost_controls'] = 55000.0
        
        # aggregator
        [outputs['cost_rotor'], \
         outputs['cost_hub_system'], \
         outputs['cost_nacelle'], \
         outputs['cost_rna']] = self.aggregator_rna(outputs, blade_number)

        
        
        
        
#############################################################################
######################  MODEL#3: NREL Turbine_CostSE ########################
#############################################################################  
class TurbineCostSE(ComponentCost):
    def compute(self, inputs, outputs):
        
        blade_number = inputs['blade_number']
        machine_rating = inputs['machine_rating']
        
        outputs['cost_blade'] = inputs['blade_mass'] * 14.6
        
        outputs['cost_hub'] = inputs['hub_mass'] * 3.9
        outputs['cost_pitch'] = inputs['pitch_mass'] * 22.1
        outputs['cost_spinner'] = inputs['spinner_mass'] * 11.1
        
        outputs['cost_lss'] = inputs['lss_mass'] * 11.9
        outputs['cost_main_bearing'] = inputs['main_bearing_mass'] * 4.5
        outputs['cost_second_bearing'] = inputs['second_bearing_mass'] * 4.5
        outputs['cost_gearbox'] = inputs['gearbox_mass'] * 12.9
        outputs['cost_hss'] = inputs['hss_mass'] * 6.8
        outputs['cost_generator'] = inputs['generator_mass'] * 12.4
        
        outputs['cost_mainframe'] = inputs['bedplate_mass'] * 2.9 + \
                                    (inputs['platform_mass'] - 3000.0*has_crane) * 17.1 + \
                                    12000.0 * has_crane
        
        outputs['cost_yaw'] = inputs['yaw_system_mass'] * 8.3
        outputs['cost_vs_electronics'] = inputs['vs_electronics_mass'] * 18.8
        outputs['cost_hvac'] = inputs['hvac_mass'] * 124.0
        outputs['cost_cover'] = inputs['cover_mass'] * 5.7
        outputs['cost_electrical'] = machine_rating * 41.85
        outputs['cost_controls'] = machine_rating * 21.15      
        outputs['cost_transformer'] = inputs['transformer_mass'] * 18.8 
        
        # aggregator
        [outputs['cost_rotor'], \
         outputs['cost_hub_system'], \
         outputs['cost_nacelle'], \
         outputs['cost_rna']] = self.aggregator_rna(outputs, blade_number)
        
        