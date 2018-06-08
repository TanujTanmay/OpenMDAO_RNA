from time import time
from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model 

from fixed_parameters import beautify


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class Spinner(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
    
        # outputs
        self.add_output('mass', units='kg', desc='overall component mass')
        
        

#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class DriveSE(Spinner):
    def compute(self, inputs, outputs):
        # inputs    
        self.rotor_diameter = inputs['rotor_diameter']
        
        self.mass =18.5 * self.rotor_diameter + (-520.5)   # spinner mass comes from cost and scaling model
                                                            
        outputs['mass'] = self.mass                                                    
           
        
        