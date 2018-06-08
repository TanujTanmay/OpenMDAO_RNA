from openmdao.api import ExplicitComponent
from fixed_parameters import safety_factor


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class Pitch(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('blade_mass', units='kg', desc='mass of one blade')
        self.add_input('rotor_bending_moment', units='N*m', desc='flapwise bending moment at blade root')
        self.add_input('blade_number', desc='number of turbine blades')
    
        # outputs
        self.add_output('mass', units='kg', desc='overall component mass')
        
        

#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class DriveSE(Pitch):
    def compute(self, inputs, outputs):
        # inputs    
        self.blade_mass = inputs['blade_mass']
        self.rotor_bending_moment = inputs['rotor_bending_moment']*safety_factor
        self.blade_number = inputs['blade_number']
        
        # Sunderland method for calculating pitch system masses
        pitchmatldensity = 7860.0                             # density of pitch system material (kg / m^3) - assuming BS1503-622 (same material as LSS)
        pitchmatlstress  = 371000000.0                              # allowable stress of hub material (N / m^2)

        hubpitchFact      = 1.0                                 # default factor is 1.0 (0.54 for modern designs)
        #self.mass =hubpitchFact * (0.22 * self.blade_mass * self.blade_number + 12.6 * self.blade_number * self.rotor_bending_moment * (pitchmatldensity / pitchmatlstress))
                                                            # mass of pitch system based on Sunderland model
        self.mass =hubpitchFact * (0.22 * self.blade_mass * self.blade_number + 12.6 * self.rotor_bending_moment * (pitchmatldensity / pitchmatlstress))
                                                            # mass of pitch system based on Sunderland model
                                                            
        outputs['mass'] = self.mass                                                    
           
 
 
 
        
#############################################################################
#############################  UNIT TESTING #################################
#############################################################################  
if __name__ == "__main__":
    inputs = {'blade_mass' : 17740.0, \
              'rotor_bending_moment' : 6196163.664902505, \
              'blade_number' : 3}
    outputs={}
    DriveSE().compute(inputs, outputs)
    print outputs        