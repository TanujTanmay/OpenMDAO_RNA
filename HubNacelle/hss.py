from time import time
from math import pi
from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model 

from fixed_parameters import safety_factor, beautify
from drivese_utils import size_HighSpeedSide


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class HSS(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_torque', units='N*m', desc='rotor torque at rated power')
        self.add_input('gear_ratio', desc='overall gearbox ratio')
        self.add_input('lss_diameter', units='m', desc='low speed shaft outer diameter')
        self.add_input('gearbox_length', units = 'm', desc='gearbox length')
        self.add_input('gearbox_height', units = 'm', desc = 'gearbox height')
        self.add_input('gearbox_cm', units = 'm', desc = 'gearbox cm [x,y,z]', shape=3)
        #self.add_input('hss_length', units = 'm', desc = 'high speed shaft length determined by user. Default 0.5m')
    
        # outputs
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)
        self.add_output('length', units='m', desc='length of high speed shaft')
         
        




#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class DriveSE(HSS):
    def compute(self, inputs, outputs):
        # inputs
        self.rotor_diameter = inputs['rotor_diameter']
        self.rotor_torque = inputs['rotor_torque']*safety_factor
        self.gear_ratio = inputs['gear_ratio']
        self.lss_diameter = inputs['lss_diameter']
        self.gearbox_length = inputs['gearbox_length']
        self.gearbox_height = inputs['gearbox_height']
        self.gearbox_cm = inputs['gearbox_cm']
        self.length_in = 0.0
        
        size_HighSpeedSide(self)
        
        # outputs
        outputs['mass'] = self.mass
        outputs['cm'] = self.cm
        outputs['I'] = self.I
        outputs['length'] = self.length
        
       
        
        
        


        
        



#############################################################################
##############################  UNIT TESTING ################################
#############################################################################         
class ComponentTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('rotor_torque', units='N*m', desc='rotor torque at rated power')
        i.add_output('gear_ratio', desc='overall gearbox ratio')
        i.add_output('lss_diameter', units='m', desc='low speed shaft outer diameter')
        i.add_output('gearbox_length', units = 'm', desc='gearbox length')
        i.add_output('gearbox_height', units = 'm', desc = 'gearbox height')
        i.add_output('gearbox_cm', units = 'm', desc = 'gearbox cm [x,y,z]', shape=3)
        i.add_output('hss_length', units = 'm', desc = 'high speed shaft length determined by user. Default 0.5m')
        
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
        prob['dof.rotor_torque'] = (1.5*5000*1000/0.95)/(12.1*pi/30)
        prob['dof.gear_ratio'] = 96.76
        prob['dof.lss_diameter'] = 1.0184
        prob['dof.gearbox_length'] = 1.512
        prob['dof.gearbox_height'] = 1.89
        prob['dof.gearbox_cm'] = [0.1  , 0.   , 0.756]
        prob['dof.hss_length'] = 1.5
         
        prob.run_model()
        
        # print outputs 
        print "DriveSE HSS"
        beautify('mass', prob['sys.mass'])
        beautify('cm', prob['sys.cm'])
        beautify('I', prob['sys.I'])
        beautify('length', prob['sys.length'])
      
        print 'Done in ' + str(time() - start) + ' seconds'  


if __name__ == "__main__":
    ComponentTest().test5()
    #ComponentTest().test1_5()
    #print 0.3*(126.0/100.0)**2.0 - 0.1 * (126.0 / 100.0) + 0.4
    #print get_L_rb(126.0, False)
    #print get_My(51092.0,1.95)
    #print get_Mz(51092.0,1.95)
        