from openmdao.api import ExplicitComponent
from drivese_utils import add_Nacelle



#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class Nacelle(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('above_yaw_mass', units='kg', desc='mass above yaw system')
        self.add_input('yawMass', units='kg', desc='mass of yaw system')
        self.add_input('lss_mass', units='kg', desc='component mass')
        self.add_input('main_bearing_mass', units='kg', desc='component mass')
        self.add_input('second_bearing_mass', units='kg', desc='component mass')
        self.add_input('gearbox_mass', units='kg', desc='component mass')
        self.add_input('hss_mass', units='kg', desc='component mass')
        self.add_input('generator_mass', units='kg', desc='component mass')
        self.add_input('bedplate_mass', units='kg', desc='component mass')
        self.add_input('mainframe_mass', units='kg', desc='component mass')
        self.add_input('lss_cm', units='m', desc='component CM', shape=3)
        self.add_input('main_bearing_cm', units='m', desc='component CM', shape=3)
        self.add_input('second_bearing_cm', units='m', desc='component CM', shape=3)
        self.add_input('gearbox_cm', units='m', desc='component CM', shape=3)
        self.add_input('hss_cm', units='m', desc='component CM', shape=3)
        self.add_input('generator_cm', units='m', desc='component CM', shape=3)
        self.add_input('bedplate_cm', units='m', desc='component CM', shape=3)
        self.add_input('lss_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('main_bearing_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('second_bearing_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('gearbox_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('hss_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('generator_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('bedplate_I', units='kg*m**2', desc='component I', shape=3)
        self.add_input('transformer_mass', units='kg', desc='component mass')
        self.add_input('transformer_cm', units='m', desc='component CM', shape=3)
        self.add_input('transformer_I', units='kg*m**2', desc='component I', shape=3)
    
        # returns
        self.add_output('nacelle_mass', units='kg', desc='overall component mass')
        self.add_output('nacelle_cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('nacelle_I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=6)

        
        
        
        
#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class DriveSE(Nacelle):
    def compute(self, inputs, outputs):
        # inputs
        self.above_yaw_mass = inputs['above_yaw_mass']
        self.yawMass = inputs['yawMass']
        self.lss_mass = inputs['lss_mass']
        self.main_bearing_mass = inputs['main_bearing_mass']
        self.second_bearing_mass = inputs['second_bearing_mass']
        self.gearbox_mass = inputs['gearbox_mass']
        self.hss_mass = inputs['hss_mass']
        self.generator_mass = inputs['generator_mass']
        self.bedplate_mass = inputs['bedplate_mass']
        self.mainframe_mass = inputs['mainframe_mass']
        self.lss_cm = inputs['lss_cm']
        self.main_bearing_cm = inputs['main_bearing_cm']
        self.second_bearing_cm = inputs['second_bearing_cm']
        self.gearbox_cm = inputs['gearbox_cm']
        self.hss_cm = inputs['hss_cm']
        self.generator_cm = inputs['generator_cm']
        self.bedplate_cm = inputs['bedplate_cm']
        self.lss_I = inputs['lss_I']
        self.main_bearing_I = inputs['main_bearing_I']
        self.second_bearing_I = inputs['second_bearing_I']
        self.gearbox_I = inputs['gearbox_I']
        self.hss_I = inputs['hss_I']
        self.generator_I = inputs['generator_I']
        self.bedplate_I = inputs['bedplate_I']
        self.transformer_mass = inputs['transformer_mass']
        self.transformer_cm = inputs['transformer_cm']
        self.transformer_I = inputs['transformer_I']
        
        add_Nacelle(self)
        
        # outputs
        outputs['nacelle_mass'] = self.nacelle_mass
        outputs['nacelle_cm'] = self.nacelle_cm
        outputs['nacelle_I'] = self.nacelle_I
        
                