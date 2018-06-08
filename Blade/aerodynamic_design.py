import numpy as np
import pandas as pd
from math import pi, degrees, radians, atan, sin, factorial
from time import time
from scipy.interpolate import spline
import matplotlib.pyplot as plt

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model
from fixed_parameters import beautify, num_nodes, num_airfoils, num_pegged
from airfoils import AirfoilProperties, ReferenceTurbine
#from sympy.core.tests.test_args import die


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
        self.add_input('chord_coefficients', units = 'm', desc = 'coefficients of polynomial chord profile', shape=num_pegged)
        self.add_input('twist_coefficients', units = 'deg', desc = 'coefficients of polynomial twist profile', shape=num_pegged)
        self.add_input('span_airfoil_r', units='m', desc='list of blade node radial location at which the airfoils are specified', shape=num_airfoils)
        self.add_input('span_airfoil_id', desc='list of blade node Airfoil ID', shape=num_airfoils)
        #self.add_input('adjust_pitch', desc = 'adjust pitch for tip twist to be 0')
        
        # outputs
        self.add_output('span_r', units='m', desc='spanwise radial location of blade junctions', shape=num_nodes)
        self.add_output('span_dr', units='m', desc='spanwise blade node thickness', shape=num_nodes)
        self.add_output('span_airfoil', desc='list of blade node airfoil ID', shape=num_nodes)
        self.add_output('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        self.add_output('span_twist', units='deg', desc='list of blade node twist angle', shape=num_nodes)
        #self.add_output('pitch', units='deg', desc='blade pitch angle')
        
        
    def get_airfoil_id(self, r, span_airfoil_r, span_airfoil_id):
        pass    










#############################################################################
###########  MODEL 1: USE CASE 1 with PEGGED POINTS##########################
#############################################################################
class PeggedNodes(AerodynamicDesign):   
    '''
        computes the chord and twist distribution using pegged points
    '''  
    
    def compute(self, inputs, outputs):
        # inputs
        rotor_diameter = inputs['rotor_diameter']
        hub_radius = inputs['hub_radius']
        chord_coefficients = inputs['chord_coefficients'] # c[0], c[1], ..., c[N-2], c_tip  -----> N = num_pegged
        twist_coefficients = inputs['twist_coefficients'] # t_root, t[1], t[2], ..., t[N-1] -----> N = num_pegged
        span_airfoil_r = inputs['span_airfoil_r']
        span_airfoil_id = inputs['span_airfoil_id']
        
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
        span_airfoil_i = 0
        trans_point = 0
        
        for i in range(num_nodes):
            mu = span_mu[i]
            r = span_r[i]
            
            # check if we need to move to the next airfoil
            next_airfoil_i = span_airfoil_i + 1 if (span_airfoil_i < num_airfoils - 1) else num_airfoils - 1
            next_airfoil_r = span_airfoil_r[next_airfoil_i]
            span_airfoil_i = span_airfoil_i + 1 if (r >= next_airfoil_r and  span_airfoil_i < num_airfoils - 1) else span_airfoil_i
            airfoil_id = int(span_airfoil_id[span_airfoil_i])
            
            span_airfoil = np.append(span_airfoil, airfoil_id)
            
            # get the optimal values of that airfoil
            cl_opt = airfoils.loc[airfoil_id, 'cl_opt']
            
            # check transition point
            if cl_opt != 0.0 and trans_point == 0:
                trans_mu = mu
                trans_point = 1
        
        # Burton's profile
        peg_mu = [mu_root, 0.7, 0.9]
        span_chord =  AerodynamicDesignBurton().chord_profile(chord_coefficients, peg_mu, span_mu)  

        peg_mu = [trans_mu, 0.4, 0.7]
        span_twist =  AerodynamicDesignBurton().twist_profile(twist_coefficients, peg_mu, span_mu)
        
        # outputs
        outputs['span_r'] = span_r
        outputs['span_dr'] = span_dr
        outputs['span_airfoil'] = span_airfoil
        outputs['span_chord'] = span_chord
        outputs['span_twist'] = span_twist
        #outputs['pitch'] = 0      
        
        
        
        

#############################################################################
###########  MODEL 2: Scaled from Reference Turbine #########################
#############################################################################
class Scaling(AerodynamicDesign):   
    '''
        computes the chord and twist distribution by scaling it from NREL 5MW Offshore Reference turbine
    '''  
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
        #outputs['pitch'] = 0 # -0.09182




        
        
        
        
#############################################################################
###################################### TRASH ################################
#############################################################################
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
        
        



class AerodynamicDesignBurton(AerodynamicDesign):  
    '''
        computes the chord and twist distribution based on analytical relationship
        that assumes a=1/3 and the angle of attack at each blade section is optimal
        and redesigned for the ease of manufacturing
    '''      
    
    def chord_profile(self, peg_chord, peg_mu, span_mu):
        # peg_chord = [root_chord, c1, c2]
        # peg_mu = [root_mu, mu1, mu2]
        
        # redesign for manufacturing ease
        mu0 = 0.20
        p0 = int(mu0*num_nodes) - 1 # index of 25% point
        
        m = (peg_chord[1] - peg_chord[2])/(peg_mu[1] - peg_mu[2]) # slope
        y = peg_chord[2] - m*peg_mu[2] # y-intercept
        c0 = m*mu0 + y # chord length at 25% point
        m0 = (c0 - peg_chord[0])/(mu0 - peg_mu[0]) # slope of chord at the transition
        y0 = peg_chord[0] - m0*peg_mu[0] # y-intercept
        
        # blade root transition slope from the reference turbine
        span_chord = []
        for i in range(p0):
            #span_chord[i] = m_tran*span_mu[i] + root_chord
            span_chord.append(m0*span_mu[i] + y0)
            
        span_chord.append(c0)    
        
        for i in range(p0+1, num_nodes):
            #span_chord[i] = m*span_mu[i] + c
            span_chord.append(m*span_mu[i] + y)    
            
        # smooth out the curve
        span_chord[p0] = (span_chord[p0-1]+span_chord[p0]+span_chord[p0+1])/3
        #mu_temp = np.linspace(span_mu.min(), span_mu.max(), 300)
        #span_chord = spline(span_mu, span_chord, mu_temp)
        
        return span_chord
    
    
    def twist_profile(self, peg_twist, peg_mu, span_mu):
        # peg_twist = [trans_twist, t1, t2]
        # peg_mu = [trans_mu, mu1, mu2]
       
        span_twist = []
        
        # zone 1 - root to p1
        for mu in [x for x in span_mu if x <= peg_mu[0]]:
            span_twist.append(peg_twist[0])
        
        # zone 2 - p1 to p2
        m1 = (peg_twist[1] - peg_twist[0])/(peg_mu[1] - peg_mu[0]) # slope
        y1 = peg_twist[0] - m1*peg_mu[0] # y-intercept   
        for mu in [x for x in span_mu if x > peg_mu[0] and x <= peg_mu[1]]:
            span_twist.append(m1*mu + y1)     
        
        # zone 3 - p2 to p3
        m2 = (peg_twist[1] - peg_twist[2])/(peg_mu[1] - peg_mu[2]) # slope
        y2 = peg_twist[2] - m2*peg_mu[2] # y-intercept
        for mu in [x for x in span_mu if x > peg_mu[1] and x <= peg_mu[2]]:
            span_twist.append(m2*mu + y2)  
        
        # zone 4 - p3 to tip
        m3 = (0.0 - peg_twist[2])/(span_mu[-1] - peg_mu[2]) # slope
        y3 = peg_twist[2] - m3*peg_mu[2] # y-intercept
        for mu in [x for x in span_mu if x > peg_mu[2]]:
            span_twist.append(m3*mu + y3)  
        
           
        
            
            
        # smooth out the curve
        #span_chord[p0] = (span_chord[p0-1]+span_chord[p0]+span_chord[p0+1])/3
        #mu_temp = np.linspace(span_mu.min(), span_mu.max(), 300)
        #span_chord = spline(span_mu, span_chord, mu_temp)
        
        return span_twist
    
    
        
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
        trans_point = 0
        
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
                
                if trans_point == 0:
                    trans_mu = mu
                    trans_point = 1
            
            span_airfoil = np.append(span_airfoil, airfoil_id)
            span_chord = np.append(span_chord, chord)
            span_twist = np.append(span_twist, twist) 
        
        # pitch the blade to make tip twist angle as 0
        if(adjust_pitch):
            pitch = span_twist[-1]
            span_twist = np.subtract(span_twist, pitch)
        else:
            pitch = 0
            
        # manufacturing ease
        peg_mu = [mu_root, 0.7, 0.9]
        peg_chord = [np.interp(mu, span_mu, span_chord) for mu in peg_mu]
        span_chord =  self.chord_profile(peg_chord, peg_mu, span_mu)  

        peg_mu = [trans_mu, 0.4, 0.7]
        peg_twist = [np.interp(mu, span_mu, span_twist) for mu in peg_mu]
        span_twist =  self.twist_profile(peg_twist, peg_mu, span_mu) 
        
        #print beautify(peg_chord)
        #print beautify(peg_twist)
        
        
        # outputs
        outputs['span_r'] = span_r
        outputs['span_dr'] = span_dr
        outputs['span_airfoil'] = span_airfoil
        outputs['span_chord'] = span_chord
        outputs['span_twist'] = span_twist
        outputs['pitch'] = pitch











        
        
        
        
        





        
        
        
        




        
class AerodynamicDesignBezier(AerodynamicDesign):   
    '''
        computes the chord and twist distribution using polynomial function
    '''  
    
    def compute(self, inputs, outputs):
        # inputs
        rotor_diameter = inputs['rotor_diameter']
        hub_radius = inputs['hub_radius']
        root_chord = inputs['root_chord']
        chord_coefficients = inputs['chord_coefficients'] # c[0], c[1], ..., c[N-2], c_tip  -----> N = num_pegged
        twist_coefficients = inputs['twist_coefficients'] # t_root, t[1], t[2], ..., t[N-1] -----> N = num_pegged
        span_airfoil_r = inputs['span_airfoil_r']
        span_airfoil_id = inputs['span_airfoil_id']
        
        rotor_radius = rotor_diameter/2.0
        mu_root = hub_radius/rotor_radius
        
        # divide the blade into annulus
        mu_ = np.linspace(mu_root, 1.0, num_nodes + 1)
        span_mu = np.array([(x+y)/2.0 for x,y in zip(mu_[:-1], mu_[1:])]) # the position of the annulus is taken as its midpoint
        span_r = np.multiply(span_mu, rotor_radius)
        span_dr = np.array([(x-y)*rotor_radius for x,y in zip(mu_[1:], mu_[:-1])]).reshape(num_nodes)
        
        # get bezier curves
        # chord
        x = np.array([span_mu[int(x)] for x in np.linspace(0, num_nodes-1, num_pegged+1)])
        y = np.array(root_chord)
        y = np.append(y, chord_coefficients)
        bezier_chord = self.bezier(x,y)
        
        # twist
        #x = np.array([int(x) for x in np.linspace(0, num_nodes-1, num_pegged+1)])
        y = np.array(twist_coefficients)
        y = np.append(y, 0.0)
        bezier_twist = self.bezier(x,y)        
        
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
            
            # use polynomial functions
            chord = np.interp(mu, bezier_chord.ix[:, 'x'], bezier_chord.ix[:, 'y']) if airfoil_id > 1 else root_chord
            twist = np.interp(mu, bezier_twist.ix[:, 'x'], bezier_twist.ix[:, 'y']) if airfoil_id > 1 else 0.0
            
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
        
        
    
    def bezier(self,x,y):  
        n_points = len(x)
        domain = np.linspace(0,1,501)
        
        points = zip(x,y)
        sigma = [factorial(n_points-1)/(factorial(i)*factorial(n_points-i-1)) for i in range(n_points)]    
        
        bezier_matrix = []
        for i in domain:
            bezier_matrix.append([sigma[j]*((1-i)**(n_points-j-1))*(i**j) for j in range(n_points)])

        #bezier_matrix = np.reshape(bezier_matrix, (len(domain), n_points))
        bezier_np = np.dot(np.array(bezier_matrix), np.array(points))    
        bezier_pd = pd.DataFrame(bezier_np, columns=['x', 'y'])
        
        return bezier_pd




#############################################################################
##############################  UNIT TESTING ################################
#############################################################################        
class UnitTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('design_tsr', desc='design tip speed ratio')
        i.add_output('blade_number', desc='number of blades')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('root_chord', units = 'm', desc = 'length of root chord')
        i.add_output('chord_coefficients', units = 'm', desc = 'coefficients of polynomial chord profile', shape=num_pegged)
        i.add_output('twist_coefficients', units = 'deg', desc = 'coefficients of polynomial twist profile', shape=num_pegged)
        i.add_output('span_airfoil_r', units='m', desc='list of blade node radial location at which the airfoils are specified', shape=num_airfoils)
        i.add_output('span_airfoil_id', desc='list of blade node Airfoil ID', shape=num_airfoils)

        # sub-components        
        self.add_subsystem('dof', i, promotes=['*'])   
        self.add_subsystem('design', PeggedNodes(), promotes_inputs=['*'])
        
    def test(self):
        start = time()
    
        # workflow setup
        prob = Problem(UnitTest())
        prob.setup()
        #view_model(prob, outfile='aero_design.html')
        
        # define inputs
        prob['dof.design_tsr'] = 8.0
        prob['dof.blade_number'] = 3
        prob['dof.rotor_diameter'] = 126.0
        prob['dof.hub_radius'] = 1.5
        prob['dof.root_chord'] = 3.542
        prob['dof.chord_coefficients'] = [3.542, 3.01, 2.313]
        prob['dof.twist_coefficients'] = [13.308, 9.0, 3.125]
        prob['dof.span_airfoil_r'] =  [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
        prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]    
         
        prob.run_model()
        
        # print outputs 
        print "Rotor Aerodynamic Design"
        print 'span_r = ' + beautify(prob['design.span_r'])
        print 'span_chord = ' + beautify(prob['design.span_chord'])
        print 'span_dr = ' + beautify(prob['design.span_dr'])
        print 'span_airfoil = ' + beautify(prob['design.span_airfoil'])
        print 'span_chord = ' + beautify(prob['design.span_chord'])
        print 'span_twist = ' + beautify(prob['design.span_twist']) 
        
        print 'Done in ' + str(time() - start) + ' seconds' 
        
        span_chord = [3.5615, 3.9127, 4.2766, 4.5753, 4.6484, 4.5489, 4.3819, 4.2206, 4.0382, 3.8449, 3.6549, 3.4713, 3.2868, 3.1022, 2.9178, 2.7332, 2.5487, 2.3691, 2.1346, 1.4683]
        span_twist = [13.235, 13.235, 13.235, 13.1066, 11.6516, 10.5523, 9.6506, 8.7896, 7.876, 6.937, 6.0226, 5.1396, 4.2562, 3.428, 2.735, 2.1466, 1.5521, 0.9525, 0.3813, 0.0477]

        
        font = {'family' : 'Tahoma', 'size' : 15}
        plt.rc('font', **font)
        fig = plt.figure()
        
        x = np.array(prob['design.span_r'])/63.0
        
        x1 = fig.add_subplot(121)
        x1.set_title('Chord distribution')
        x1.set_xlabel('Normalized radial distance [-]')
        x1.set_ylabel('Chord [m]')
        x1.plot(x, prob['design.span_chord'], marker='^', label='Burton')
        x1.plot(x, span_chord, marker='s', label='Reference')
        x1.axvline(x=0.045, linestyle=':', color='r')
        x1.axvline(x=0.20, linestyle=':', color='c')
        x1.axvline(x=0.70, linestyle=':', color='r')
        x1.axvline(x=0.90, linestyle=':', color='r')
        
        
        x2 = fig.add_subplot(122)
        x2.set_title('Twist distribution')
        x2.set_xlabel('Normalized radial distance [m]')
        x2.set_ylabel('Twist [deg]')
        x2.plot(x, prob['design.span_twist'], marker='^', label='Burton')
        x2.plot(x, span_twist, marker='s', label='Reference')
        x2.axvline(x=0.20, linestyle=':', color='r')
        x2.axvline(x=0.40, linestyle=':', color='r')
        x2.axvline(x=0.70, linestyle=':', color='r')
        
        x1.legend(loc='upper right')
        x2.legend(loc='upper right')
        plt.show()
 


if __name__ == "__main__":
    AerodynamicDesignTest().test()
    


 