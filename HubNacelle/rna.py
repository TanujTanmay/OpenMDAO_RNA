from openmdao.api import ExplicitComponent
from drivese_utils import add_RNA



#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class RNA(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('yawMass', units='kg', desc='mass of yaw system')
        self.add_input('lss_mass', units='kg', desc='component mass')
        self.add_input('main_bearing_mass', units='kg', desc='component mass')
        self.add_input('second_bearing_mass', units='kg', desc='component mass')
        self.add_input('gearbox_mass', units='kg', desc='component mass')
        self.add_input('hss_mass', units='kg', desc='component mass')
        self.add_input('generator_mass', units='kg', desc='component mass')
        self.add_input('lss_cm', units='m', desc='component CM', shape=3)
        self.add_input('main_bearing_cm', units='m', desc='component CM', shape=3)
        self.add_input('second_bearing_cm', units='m', desc='component CM', shape=3)
        self.add_input('gearbox_cm', units='m', desc='component CM', shape=3)
        self.add_input('hss_cm', units='m', desc='component CM', shape=3)
        self.add_input('generator_cm', units='m', desc='component CM', shape=3)
        self.add_input('overhang', units='m', desc='nacelle overhang')
        self.add_input('rotor_mass', units='kg', desc='component mass')
        self.add_input('machine_rating', units = 'kW', desc = 'machine rating ')
    
        # outputs
        self.add_output('RNA_mass', units='kg', desc='mass of total RNA')
        self.add_output('RNA_cm', units='m', desc='RNA CM along x-axis', shape=3)
        
        
        
        
#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class DriveSE(RNA):
    def compute(self, inputs, outputs):
        # inputs
        self.yawMass = inputs['yawMass']
        self.lss_mass = inputs['lss_mass']
        self.main_bearing_mass = inputs['main_bearing_mass']
        self.second_bearing_mass = inputs['second_bearing_mass']
        self.gearbox_mass = inputs['gearbox_mass']
        self.hss_mass = inputs['hss_mass']
        self.generator_mass = inputs['generator_mass']
        self.lss_cm = inputs['lss_cm']
        self.main_bearing_cm = inputs['main_bearing_cm']
        self.second_bearing_cm = inputs['second_bearing_cm']
        self.gearbox_cm = inputs['gearbox_cm']
        self.hss_cm = inputs['hss_cm']
        self.generator_cm = inputs['generator_cm']
        self.overhang = inputs['overhang']
        self.rotor_mass = inputs['rotor_mass']
        self.machine_rating = inputs['machine_rating']
        
        add_RNA(self)
        
        # outputs
        outputs['RNA_mass'] = self.RNA_mass
        outputs['RNA_cm'] = self.RNA_cm
        
        
                