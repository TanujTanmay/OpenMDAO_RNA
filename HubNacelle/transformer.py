from openmdao.api import ExplicitComponent

from fixed_parameters import uptower_transformer
from drivese_utils import size_Transformer



#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class Transformer(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('machine_rating', units='kW', desc='machine rating of the turbine')
        #self.add_input('uptower_transformer', desc = 'uptower or downtower transformer')
        self.add_input('tower_top_diameter', units = 'm', desc = 'tower top diameter for comparision of nacelle CM')
        self.add_input('rotor_mass', units='kg', desc='rotor mass')
        self.add_input('overhang', units='m', desc='rotor overhang distance')
        self.add_input('generator_cm', units='m', desc='center of mass of the generator in [x,y,z]', shape=3)
        self.add_input('rotor_diameter', units='m', desc='rotor diameter of turbine')
        self.add_input('RNA_mass', units='kg', desc='mass of total RNA')
        self.add_input('RNA_cm', units='m', desc='RNA CM along x-axis')
    
        # outputs
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)    
        
        
        
        
        
#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class DriveSE(Transformer):
    def compute(self, inputs, outputs):
        # inputs
        self.machine_rating = inputs['machine_rating']
        self.uptower_transformer = uptower_transformer
        self.tower_top_diameter = inputs['tower_top_diameter']
        self.rotor_mass = inputs['rotor_mass']
        self.overhang = inputs['overhang']
        self.generator_cm = inputs['generator_cm']
        self.rotor_diameter = inputs['rotor_diameter']
        self.RNA_mass = inputs['RNA_mass']
        self.RNA_cm = inputs['RNA_cm']
        
        size_Transformer(self)
        
        # outputs
        outputs['mass'] = self.mass
        outputs['cm'] = self.cm
        outputs['I'] = self.I
        
        
                