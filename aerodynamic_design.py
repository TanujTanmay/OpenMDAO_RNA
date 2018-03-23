import numpy as np
import pandas as pd
from math import pi, degrees, radians, atan, sin
from time import time

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model
from fixed_parameters import beautify, num_nodes, num_airfoils
from airfoils import AirfoilProperties, ReferenceTurbine


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class AerodynamicDesign(ExplicitComponent):
    def setup(self):        
        # inputs
        self.add_input('design_tsr', desc='design tip speed ratio')
        self.add_input('blade_number', desc='number of blades')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('hub_radius', units = 'm', desc = 'hub radius')
        self.add_input('root_chord', units = 'm', desc = 'length of root chord')
        self.add_input('span_airfoil_r', units='m', desc='list of blade node radial location at which the airfoils are specified', shape=num_airfoils)
        self.add_input('span_airfoil_id', desc='list of blade node Airfoil ID', shape=num_airfoils)
        self.add_input('adjust_pitch', desc = 'adjust pitch for tip twist to be 0')
        
        # outputs
        self.add_output('span_r', units='m', desc='spanwise radial location of blade junctions', shape=num_nodes)
        self.add_output('span_dr', units='m', desc='spanwise blade node thickness', shape=num_nodes)
        self.add_output('span_airfoil', desc='list of blade node airfoil ID', shape=num_nodes)
        self.add_output('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        self.add_output('span_twist', units='deg', desc='list of blade node twist angle', shape=num_nodes)
        self.add_output('pitch', units='deg', desc='blade pitch angle')


    def scale_chord_with_radius(self, mu, radius):
        '''
            This function scales the chord length of the blade with respect to NREL 5MW Offshore wind turbine
            ASSUMING CONSTANT number of blades (3), airfoil distribution
            but VARYING rotor radius
        '''
        ref_radius = ReferenceTurbine.r.iat[-1]
        ref_chord = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['chord'])
        ref_twist = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['twist'])
        
        s = radius/ref_radius # scaling factor
        
        chord = ref_chord * (s**1)
        twist = ref_twist * (s**0)
        
        return [chord, twist]   






class AerodynamicDesignBetz(AerodynamicDesign):  
    '''
        computes the chord and twist distribution based on analytical relationship
        that assumes a=1/3 and the angle of attack at each blade section is optimal
    '''          
    def compute(self, inputs, outputs):
        # inputs
        tsr = inputs['design_tsr']
        b = inputs['blade_number'] 
        rotor_diameter = inputs['rotor_diameter']
        hub_radius = inputs['hub_radius']
        root_chord = inputs['root_chord']
        span_airfoil_r = inputs['span_airfoil_r']
        span_airfoil_id = inputs['span_airfoil_id']
        adjust_pitch = inputs['adjust_pitch']
        
        rotor_radius = rotor_diameter/2.0
        mu_root = hub_radius/rotor_radius
        
        
        # get distinct airfoil data
        airfoils_ = []
        for i in set(span_airfoil_id):
            airfoils_.append(AirfoilProperties(int(i)))
            
        airfoils = pd.DataFrame(airfoils_)  
        airfoils = airfoils.set_index('id')
        
        # divide the blade into annulus
        mu_ = np.linspace(mu_root, 1.0, num_nodes + 1)
        span_mu = np.array([(x+y)/2.0 for x,y in zip(mu_[:-1], mu_[1:])]) # the position of the annulus is taken as its midpoint
        span_r = np.multiply(span_mu, rotor_radius)
        span_dr = np.array([(x-y)*rotor_radius for x,y in zip(mu_[1:], mu_[:-1])]).reshape(num_nodes)        
        
        span_airfoil = np.array([])
        span_chord = np.array([])
        span_twist = np.array([])
        
        span_airfoil_i = 0
        
        for i in range(num_nodes):
            mu = span_mu[i]
            r = span_r[i]
            tsr_r = tsr * mu
            
            # check if we need to move to the next airfoil
            next_airfoil_i = span_airfoil_i + 1 if (span_airfoil_i < num_airfoils - 1) else num_airfoils - 1
            next_airfoil_r = span_airfoil_r[next_airfoil_i]
            span_airfoil_i = span_airfoil_i + 1 if (r >= next_airfoil_r and  span_airfoil_i < num_airfoils - 1) else span_airfoil_i
            airfoil_id = int(span_airfoil_id[span_airfoil_i])
            
            # get the optimal values of that airfoil
            cl_opt = airfoils.loc[airfoil_id, 'cl_opt']
            alpha_opt = airfoils.loc[airfoil_id, 'alpha_opt']
            
            # Manwell et al
            phi_opt = degrees(atan(2.0/(3.0 * tsr_r)))
            
            if cl_opt == 0.0:
                chord = root_chord
                twist = 0.0  
            else:
                chord = (8*pi*rotor_radius*sin(radians(phi_opt)))/(3*b*cl_opt*tsr)
                twist = phi_opt - alpha_opt
            
            span_airfoil = np.append(span_airfoil, airfoil_id)
            span_chord = np.append(span_chord, chord)
            span_twist = np.append(span_twist, twist) 
        
        # pitch the blade to make tip twist angle as 0
        if(adjust_pitch):
            pitch = span_twist[-1]
            span_twist = np.subtract(span_twist, pitch)
        else:
            pitch = 0
        
        # outputs
        outputs['span_r'] = span_r
        outputs['span_dr'] = span_dr
        outputs['span_airfoil'] = span_airfoil
        outputs['span_chord'] = span_chord
        outputs['span_twist'] = span_twist
        outputs['pitch'] = pitch
        
        





class AerodynamicDesignScaling(AerodynamicDesign):   
    '''
        computes the chord and twist distribution by scaling it from NREL 5MW Offshore Reference turbine
    '''  
    def compute(self, inputs, outputs):
        # inputs
        rotor_diameter = inputs['rotor_diameter']
        hub_radius = inputs['hub_radius']
        span_airfoil_r = inputs['span_airfoil_r']
        span_airfoil_id = inputs['span_airfoil_id']
        
        rotor_radius = rotor_diameter/2.0
        mu_root = hub_radius/rotor_radius
        
        # divide the blade into annulus
        mu_ = np.linspace(mu_root, 1.0, num_nodes + 1)
        span_mu = np.array([(x+y)/2.0 for x,y in zip(mu_[:-1], mu_[1:])]) # the position of the annulus is taken as its midpoint
        span_r = np.multiply(span_mu, rotor_radius)
        span_dr = np.array([(x-y)*rotor_radius for x,y in zip(mu_[1:], mu_[:-1])]).reshape(num_nodes)        
        
        span_airfoil = np.array([])
        span_chord = np.array([])
        span_twist = np.array([])
        
        span_airfoil_i = 0
        
        for i in range(num_nodes):
            mu = span_mu[i]
            r = span_r[i]
            
            # check if we need to move to the next airfoil
            next_airfoil_i = span_airfoil_i + 1 if (span_airfoil_i < num_airfoils - 1) else num_airfoils - 1
            next_airfoil_r = span_airfoil_r[next_airfoil_i]
            span_airfoil_i = span_airfoil_i + 1 if (r >= next_airfoil_r and  span_airfoil_i < num_airfoils - 1) else span_airfoil_i
            airfoil_id = int(span_airfoil_id[span_airfoil_i])
            
            # use scaling law
            [chord, twist] = self.scale_chord_with_radius(mu, rotor_radius)
            
            span_airfoil = np.append(span_airfoil, airfoil_id)
            span_chord = np.append(span_chord, chord)
            span_twist = np.append(span_twist, twist) 
        
        # outputs
        outputs['span_r'] = span_r
        outputs['span_dr'] = span_dr
        outputs['span_airfoil'] = span_airfoil
        outputs['span_chord'] = span_chord
        outputs['span_twist'] = span_twist
        outputs['pitch'] = 0






#############################################################################
##############################  UNIT TESTING ################################
#############################################################################        
class AerodynamicDesignTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('design_tsr', desc='design tip speed ratio')
        i.add_output('blade_number', desc='number of blades')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('root_chord', units = 'm', desc = 'length of root chord')
        i.add_output('span_airfoil_r', units='m', desc='list of blade node radial location at which the airfoils are specified', shape=num_airfoils)
        i.add_output('span_airfoil_id', desc='list of blade node Airfoil ID', shape=num_airfoils)
        i.add_output('adjust_pitch', desc = 'adjust pitch for tip twist to be 0')

        # sub-components        
        self.add_subsystem('dof', i, promotes=['*'])   
        self.add_subsystem('design', AerodynamicDesignScaling(), promotes_inputs=['*'])
        

        
 


if __name__ == "__main__":
    start = time()
    
    # workflow setup
    prob = Problem(AerodynamicDesignTest())
    prob.setup()
    view_model(prob, outfile='N2/aero_design.html')
    
    # define inputs
    prob['dof.design_tsr'] = 7.0
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 126.0
    prob['dof.hub_radius'] = 1.5
    prob['dof.root_chord'] = 3.4
    prob['dof.span_airfoil_r'] =  [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
    prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]
    prob['dof.adjust_pitch'] = 1    
     
    prob.run_model()
    
    # print outputs 
    print "Rotor Aerodynamic Design"
    print 'span_r = ' + beautify(prob['design.span_r'])
    print 'span_dr = ' + beautify(prob['design.span_dr'])
    print 'span_airfoil = ' + beautify(prob['design.span_airfoil'])
    print 'span_chord = ' + beautify(prob['design.span_chord'])
    print 'span_twist = ' + beautify(prob['design.span_twist'])
    print 'pitch = ' + beautify(prob['design.pitch'])      
  
    print 'Done in ' + str(time() - start) + ' seconds'
    


 