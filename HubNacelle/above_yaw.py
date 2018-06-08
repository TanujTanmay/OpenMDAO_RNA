from openmdao.api import ExplicitComponent

from fixed_parameters import has_crane
from drivese_utils import add_AboveYawMass


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class AboveYaw(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('machine_rating', units='kW', desc='machine rating')
        self.add_input('lss_mass', units='kg', desc='component mass')
        self.add_input('main_bearing_mass', units='kg', desc='component mass')
        self.add_input('second_bearing_mass', units='kg', desc='component mass')
        self.add_input('gearbox_mass', units='kg', desc='component mass')
        self.add_input('hss_mass', units='kg', desc='component mass')
        self.add_input('generator_mass', units='kg', desc='component mass')
        self.add_input('bedplate_mass', units='kg', desc='component mass')
        self.add_input('bedplate_length', units='m', desc='component length')
        self.add_input('bedplate_width', units='m', desc='component width')
        self.add_input('transformer_mass', units = 'kg', desc='Transformer mass')
    
        # outputs
        self.add_output('electrical_mass', units='kg', desc='component mass')
        self.add_output('vs_electronics_mass', units='kg', desc='component mass')
        self.add_output('hvac_mass', units='kg', desc='component mass')
        self.add_output('controls_mass', units='kg', desc='component mass')
        self.add_output('platforms_mass', units='kg', desc='component mass')
        self.add_output('crane_mass', units='kg', desc='component mass')
        self.add_output('mainframe_mass', units='kg', desc='component mass')
        self.add_output('cover_mass', units='kg', desc='component mass')
        self.add_output('above_yaw_mass', units='kg', desc='total mass above yaw system')
        self.add_output('length', units='m', desc='component length')
        self.add_output('width', units='m', desc='component width')
        self.add_output('height', units='m', desc='component height')
        
        
#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class DriveSE(AboveYaw):
    def compute(self, inputs, outputs):
        # inputs
        self.machine_rating = inputs['machine_rating']
        self.lss_mass = inputs['lss_mass']
        self.main_bearing_mass = inputs['main_bearing_mass']
        self.second_bearing_mass = inputs['second_bearing_mass']
        self.gearbox_mass = inputs['gearbox_mass']
        self.hss_mass = inputs['hss_mass']
        self.generator_mass = inputs['generator_mass']
        self.bedplate_mass = inputs['bedplate_mass']
        self.bedplate_length = inputs['bedplate_length']
        self.bedplate_width = inputs['bedplate_width']
        self.transformer_mass = inputs['transformer_mass']
        self.crane = has_crane

        add_AboveYawMass(self)
        
        outputs['electrical_mass'] = self.electrical_mass
        outputs['vs_electronics_mass'] = self.vs_electronics_mass
        outputs['hvac_mass'] = self.hvac_mass
        outputs['controls_mass'] = self.controls_mass
        outputs['platforms_mass'] = self.platforms_mass
        outputs['crane_mass'] = self.crane_mass
        outputs['mainframe_mass'] = self.mainframe_mass
        outputs['cover_mass'] = self.cover_mass
        outputs['above_yaw_mass'] = self.above_yaw_mass
        outputs['length'] = self.length
        outputs['width'] = self.width
        outputs['height'] = self.height
    
    
    
            