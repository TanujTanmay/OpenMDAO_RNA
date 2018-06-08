import numpy as np
from time import time
from math import pi
from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model 

from fixed_parameters import safety_factor, beautify


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class Bearing(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('bearing_mass', units = 'kg', desc = 'bearing mass from LSS model')
        self.add_input('lss_diameter', units='m', desc='lss outer diameter at main bearing')
        self.add_input('rotor_torque', units='N*m', desc='lss design torque')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('location', units = 'm', desc = 'x,y,z location from shaft model', shape=3)
    
        # outputs
        self.add_output('mass', units='kg', desc='bearing mass')
        self.add_output('cm', units='m', desc='center of mass of bearing in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for bearing [Ixx, Iyy, Izz] around its center of mass', shape=3)
         
        




#############################################################################
#############################  MODEL#1: MainBearing #########################
#############################################################################        
class MainBearing(Bearing):
    def compute(self, inputs, outputs):
        # inputs
        self.bearing_mass = inputs['bearing_mass']
        self.lss_diameter = inputs['lss_diameter']
        self.lss_design_torque = inputs['rotor_torque']*safety_factor
        self.rotor_diameter = inputs['rotor_diameter']
        self.location = inputs['location']  
        
        self.mass = self.bearing_mass
        self.mass += self.mass*(8000.0/2700.0) #add housing weight
        
        # calculate mass properties
        inDiam  = self.lss_diameter
        depth = (inDiam * 1.5)

        if self.location[0] != 0.0:
            self.cm = self.location

        else:
            cmMB = np.array([0.0,0.0,0.0])
            cmMB = ([- (0.035 * self.rotor_diameter), 0.0, 0.025 * self.rotor_diameter])
            self.cm = cmMB
       
        b1I0 = (self.mass * inDiam ** 2 ) / 4.0 
        self.I = ([b1I0, b1I0 / 2.0, b1I0 / 2.0])
        
        # outputs 
        outputs['mass'] = self.mass
        outputs['cm'] = self.cm
        outputs['I'] = np.reshape(self.I, 3)
        
        
        
#############################################################################
###########################  MODEL#1: SecondBearing   #######################
#############################################################################        
class SecondBearing(Bearing):
    def compute(self, inputs, outputs):
        # inputs
        self.bearing_mass = inputs['bearing_mass']
        self.lss_diameter = inputs['lss_diameter']
        self.lss_design_torque = inputs['rotor_torque']*safety_factor
        self.rotor_diameter = inputs['rotor_diameter']
        self.location = inputs['location']  
        
        self.mass = self.bearing_mass
        self.mass += self.mass*(8000.0/2700.0) #add housing weight
        
        # calculate mass properties
        inDiam  = self.lss_diameter
        depth = (inDiam * 1.5)

        if self.mass > 0 and self.location[0] != 0.0:
            self.cm = self.location
        else:
            self.cm = np.array([0,0,0])
            self.mass = 0.


        b2I0  = (self.mass * inDiam ** 2 ) / 4.0 
        self.I = ([b2I0, b2I0 / 2.0, b2I0 / 2.0])
        
        # outputs 
        outputs['mass'] = self.mass
        outputs['cm'] = self.cm
        outputs['I'] = np.reshape(self.I, 3)

        
        



#############################################################################
##############################  UNIT TESTING ################################
#############################################################################         
class BearingTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('bearing_mass', units = 'kg', desc = 'bearing mass from LSS model')
        i.add_output('lss_diameter', units='m', desc='lss outer diameter at main bearing')
        i.add_output('rotor_torque', units='N*m', desc='lss design torque')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('location', units = 'm', desc = 'x,y,z location from shaft model', shape=3)
        
        # sub-components         
        self.add_subsystem('dof', i, promotes=['*'])   
        self.add_subsystem('sys', MainBearing(), promotes_inputs=['*'])
        
        
    def test5(self):
        start = time()
    
        # workflow setup
        prob = Problem(BearingTest())
        prob.setup()
        #view_model(prob, outfile='lss.html')
        
        # define inputs
        prob['dof.bearing_mass'] = 1489.0804 # 460.0619 #  
        prob['dof.lss_diameter'] = 0.9819 # [0.6873] # 
        prob['dof.rotor_torque'] = ((1.5*5000*1000/0.95)/(12.1*pi/30))
        prob['dof.rotor_diameter'] = 126.0
        prob['dof.location'] = [-1.3637,  0.    , -1.6364] # [-0.6829,  0.    ,  0.665 ] # 
         
        prob.run_model()
        
        # print outputs 
        print "DriveSE Main Bearing"
        beautify('mass', prob['sys.mass'])
        beautify('cm', prob['sys.cm'])
        beautify('I', prob['sys.I'])
      
        print 'Done in ' + str(time() - start) + ' seconds'  
        
        
    def test1_5(self):
        start = time()
    
        # workflow setup
        prob = Problem(BearingTest())
        prob.setup()
        #view_model(prob, outfile='lss.html')
        
        # define inputs
        prob['dof.bearing_mass'] = 352.4511 # 145.0198
        prob['dof.lss_diameter'] = 1.0184 # 0.6927
        prob['dof.rotor_torque'] = (1.5*1500*1000/0.95)/(16.18*pi/30)/97
        prob['dof.rotor_diameter'] = 77.0
        prob['dof.location'] = [-0.8162,  0.    , -0.7352] # [-0.4758,  0.    ,  0.4155]
         
        prob.run_model()
        
        # print outputs 
        print "DriveSE Bearing"
        beautify('mass', prob['sys.mass'])
        beautify('cm', prob['sys.cm'])
        beautify('I', prob['sys.I'])
      
        print 'Done in ' + str(time() - start) + ' seconds'      


if __name__ == "__main__":
    BearingTest().test5()
    #BearingTest().test1_5()
    #print 0.3*(126.0/100.0)**2.0 - 0.1 * (126.0 / 100.0) + 0.4
    #print get_L_rb(126.0, False)
    #print get_My(51092.0,1.95)
    #print get_Mz(51092.0,1.95)
        