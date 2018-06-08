from time import time
from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model 

from fixed_parameters import drivetrain_design, beautify
from drivese_utils import size_Generator


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class Generator(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('machine_rating', units='kW', desc='machine rating of generator')
        self.add_input('gear_ratio', desc='overall gearbox ratio')
        self.add_input('hss_length', units = 'm', desc='length of high speed shaft and brake')
        self.add_input('hss_cm', units = 'm', desc='cm of high speed shaft and brake', shape=3)
        self.add_input('rotor_speed', units='rpm', desc='Speed of rotor at rated power')
    
        # outputs
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)
         
        




#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class DriveSE(Generator):
    def compute(self, inputs, outputs):
        # inputs
        self.rotor_diameter = inputs['rotor_diameter']
        self.machine_rating = inputs['machine_rating']
        self.gear_ratio = inputs['gear_ratio']
        self.highSpeedSide_length = inputs['hss_length']
        self.highSpeedSide_cm = inputs['hss_cm']
        self.rotor_speed = inputs['rotor_speed']
        
        self.drivetrain_design = drivetrain_design
        size_Generator(self)
        
        # outputs
        outputs['mass'] = self.mass
        outputs['cm'] = self.cm
        outputs['I'] = self.I
        
       
        
        
        


        
        



#############################################################################
##############################  UNIT TESTING ################################
#############################################################################         
class ComponentTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('machine_rating', units='kW', desc='machine rating of generator')
        i.add_output('gear_ratio', desc='overall gearbox ratio')
        i.add_output('hss_length', units = 'm', desc='length of high speed shaft and brake')
        i.add_output('hss_cm', units = 'm', desc='cm of high speed shaft and brake', shape=3)
        i.add_output('rotor_speed', units='rpm', desc='Speed of rotor at rated power')
        
        # sub-components         
        self.add_subsystem('dof', i, promotes=['*'])   
        self.add_subsystem('sys', DriveSE(), promotes_inputs=['*'])
        
        
    def test5(self):
        start = time()
    
        # workflow setup
        prob = Problem(ComponentTest())
        prob.setup()
        #view_model(prob, outfile='lss.html')
        
        # define inputs
        prob['dof.rotor_diameter'] = 126.0
        prob['dof.machine_rating'] = 5000.0
        prob['dof.gear_ratio'] = 96.76 
        prob['dof.hss_length'] = 1.5
        prob['dof.hss_cm'] = [1.606, 0.   , 1.134]
        prob['dof.rotor_speed'] = 12.1
         
        prob.run_model()
        
        # print outputs 
        print "DriveSE Generator"
        beautify('mass', prob['sys.mass'])
        beautify('cm', prob['sys.cm'])
        beautify('I', prob['sys.I'])
      
        print 'Done in ' + str(time() - start) + ' seconds'  


if __name__ == "__main__":
    ComponentTest().test5()
    #ComponentTest().test1_5()
    #print 0.3*(126.0/100.0)**2.0 - 0.1 * (126.0 / 100.0) + 0.4
    #print get_L_rb(126.0, False)
    #print get_My(51092.0,1.95)
    #print get_Mz(51092.0,1.95)
        