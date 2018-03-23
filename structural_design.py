import numpy as np
import pandas as pd
from math import pi, degrees, radians, atan, sin
from time import time

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model
from fixed_parameters import beautify, num_nodes
from airfoils import ReferenceTurbine

#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class StructuralDesign(ExplicitComponent):
    def setup(self):
        '''
            defines the input and output parameters of StructuralDesign discipline
        '''
        # inputs
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('thickness_factor', desc='scaling factor for laminate thickness')
        self.add_input('span_r', units='m', desc='spanwise dimensionless radial location', shape=num_nodes)
        self.add_input('span_dr', units='m', desc='spanwise blade node thickness', shape=num_nodes)
        self.add_input('span_chord', units='m', desc='spanwise chord length', shape=num_nodes)   
        self.add_input('blade_number', desc='number of blades')     
        
        # outputs
        self.add_output('span_thickness', units='m', desc='spanwise blade thickness', shape=num_nodes)
        self.add_output('span_mass', units='kg/m', desc='spanwise blade node mass per unit length', shape=num_nodes)
        self.add_output('span_flap_inertia', units = 'm**4', desc = 'spanwise second moment of flapwise area', shape=num_nodes)
        self.add_output('span_edge_inertia', units = 'm**4', desc = 'spanwise second moment of edgewise area', shape=num_nodes)
        self.add_output('span_flap_stiff', units = 'N*m**2', desc = 'spanwise flapwise stiffness', shape=num_nodes)
        self.add_output('span_edge_stiff', units = 'N*m**2', desc = 'spanwise edgewise stiffness', shape=num_nodes)
        self.add_output('blade_mass', units = 'kg', desc='mass of one blade')
        self.add_output('rotor_mass', units = 'kg', desc='mass of all blades')

    
    def blade_mass(self, span_mass, span_dr):
        return np.dot(span_mass, span_dr)
    
    
    def scale_constant_radius(self, mu, chord, thickness_factor):
        '''
            scales the structural profile of the blade with respect to NREL 5MW Offshore wind turbine
            ASSUMING CONSTANT radius, airfoil distribution and number of blades
            but VARYING chord distribution
        '''
        # get the values of reference turbine
        ref_radius = ReferenceTurbine.r.iat[-1]
        ref_chord = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['chord'])
        ref_thickness = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['thickness'])
        ref_mass = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['mass'])
        ref_flap_inertia = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['flap_inertia'])
        ref_edge_inertia = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['edge_inertia'])
        ref_flap_stiff = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['flap_stiffness'])
        ref_edge_stiff = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['edge_stiffness'])
        
        s = chord/ref_chord # scaling factor
        
        thickness = ref_thickness * (s**1)
        mass = ref_mass * (s**1) * thickness_factor
        flap_inertia = ref_flap_inertia * (s**3) * thickness_factor
        edge_inertia = ref_edge_inertia * (s**3) * thickness_factor
        flap_stiff = ref_flap_stiff * (s**3) * thickness_factor
        edge_stiff = ref_edge_stiff * (s**3) * thickness_factor
        
        return [thickness, mass, flap_inertia, edge_inertia, flap_stiff, edge_stiff]
    
    
    def scale_variable_radius(self, mu, radius):
        '''
            scales the structural profile of the blade with respect to NREL 5MW Offshore wind turbine
            ASSUMING CONSTANT airfoil distribution and number of blades
            but VARYING radius
        '''
        # get the values of reference turbine
        ref_radius = ReferenceTurbine.r.iat[-1]
        ref_chord = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['chord'])
        ref_thickness = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['thickness'])
        ref_mass = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['mass'])
        ref_flap_inertia = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['flap_inertia'])
        ref_edge_inertia = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['edge_inertia'])
        ref_flap_stiff = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['flap_stiffness'])
        ref_edge_stiff = np.interp(mu, ReferenceTurbine['mu'], ReferenceTurbine['edge_stiffness'])
        
        s = radius/ref_radius # scaling factor
        
        thickness = ref_thickness * (s**1)
        mass = ref_mass * (s**2)
        flap_inertia = ref_flap_inertia * (s**4)
        edge_inertia = ref_edge_inertia * (s**4)
        flap_stiff = ref_flap_stiff * (s**4)
        edge_stiff = ref_edge_stiff * (s**4)
        
        return [thickness, mass, flap_inertia, edge_inertia, flap_stiff, edge_stiff]






class StructuralDesignConstantRadius(StructuralDesign):    
    def compute(self, inputs, outputs):
        # inputs
        rotor_diameter = inputs['rotor_diameter']
        thickness_factor = inputs['thickness_factor']
        span_r = inputs['span_r']
        span_dr = inputs['span_dr']
        span_chord = inputs['span_chord']
        blade_number = inputs['blade_number']
        
        
        rotor_radius = rotor_diameter/2.0
        span_mu = [x/rotor_radius for x in span_r]
        span_thickness = np.array([])
        span_mass = np.array([])
        span_flap_inertia = np.array([])
        span_edge_inertia = np.array([])
        span_flap_stiff = np.array([])
        span_edge_stiff = np.array([])  
        
        for i in range(num_nodes):
            [thickness, mass, flap_inertia, edge_inertia, flap_stiff, edge_stiff] = \
                self.scale_constant_radius(span_mu[i], span_chord[i], thickness_factor)
             
            span_thickness = np.append(span_thickness, thickness)
            span_mass = np.append(span_mass, mass)
            span_flap_inertia = np.append(span_flap_inertia, flap_inertia)
            span_edge_inertia = np.append(span_edge_inertia, edge_inertia)
            span_flap_stiff = np.append(span_flap_stiff, flap_stiff)
            span_edge_stiff = np.append(span_edge_stiff, edge_stiff)
        
        blade_mass = self.blade_mass(span_mass, span_dr)
        rotor_mass = blade_mass * blade_number
        
        # outputs
        outputs['span_thickness'] = span_thickness
        outputs['span_mass'] = span_mass
        outputs['span_flap_inertia'] = span_flap_inertia
        outputs['span_edge_inertia'] = span_edge_inertia 
        outputs['span_flap_stiff'] = span_flap_stiff
        outputs['span_edge_stiff'] = span_edge_stiff
        outputs['blade_mass'] = blade_mass
        outputs['rotor_mass'] = rotor_mass
        
        
        


        
class StructuralDesignVariableRadius(StructuralDesign):    
    def compute(self, inputs, outputs):
        # inputs
        rotor_diameter = inputs['rotor_diameter']
        thickness_factor = inputs['thickness_factor']
        span_r = inputs['span_r']
        span_dr = inputs['span_dr']
        span_chord = inputs['span_chord']
        blade_number = inputs['blade_number']
        
        
        rotor_radius = rotor_diameter/2.0
        span_mu = [x/rotor_radius for x in span_r]
        span_thickness = np.array([])
        span_mass = np.array([])
        span_flap_inertia = np.array([])
        span_edge_inertia = np.array([])
        span_flap_stiff = np.array([])
        span_edge_stiff = np.array([]) 
        
        for i in range(num_nodes):
            [thickness, mass, flap_inertia, edge_inertia, flap_stiff, edge_stiff] = \
                self.scale_variable_radius(span_mu[i], rotor_radius)
             
            span_thickness = np.append(span_thickness, thickness)
            span_mass = np.append(span_mass, mass)
            span_flap_inertia = np.append(span_flap_inertia, flap_inertia)
            span_edge_inertia = np.append(span_edge_inertia, edge_inertia)
            span_flap_stiff = np.append(span_flap_stiff, flap_stiff)
            span_edge_stiff = np.append(span_edge_stiff, edge_stiff)
        
        blade_mass = self.blade_mass(span_mass, span_dr)
        rotor_mass = blade_mass * blade_number
        
        # outputs
        outputs['span_thickness'] = span_thickness
        outputs['span_mass'] = span_mass
        outputs['span_flap_inertia'] = span_flap_inertia
        outputs['span_edge_inertia'] = span_edge_inertia 
        outputs['span_flap_stiff'] = span_flap_stiff
        outputs['span_edge_stiff'] = span_edge_stiff 
        outputs['blade_mass'] = blade_mass
        outputs['rotor_mass'] = rotor_mass  
        





#############################################################################
##############################  UNIT TESTING ################################
#############################################################################
class StructuralDesignTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('thickness_factor', desc='scaling factor for laminate thickness')
        i.add_output('span_r', units='m', desc='spanwise dimensionless radial location', shape=num_nodes)
        i.add_output('span_dr', units='m', desc='spanwise blade node thickness', shape=num_nodes)
        i.add_output('span_chord', units='m', desc='spanwise chord length', shape=num_nodes) 
        i.add_output('blade_number', desc='number of blades')

        # sub-components        
        self.add_subsystem('dof', i, promotes=['*'])   
        self.add_subsystem('design', StructuralDesignVariableRadius(), promotes_inputs=['*'])
                
        
if __name__ == "__main__":
    start = time()
    
    # workflow setup
    prob = Problem(StructuralDesignTest())
    prob.setup()
    view_model(prob, outfile='N2/structural_design.html')
    
    # define inputs
    span_r = [3.0375, 6.1125, 9.1875, 12.2625, 15.3375, 18.4125, 21.4875, 24.5625, 27.6375, 30.7125, 33.7875, 36.8625, 39.9375, 43.0125, 46.0875, 49.1625, 52.2375, 55.3125, 58.3875, 61.4625]
    span_dr = [3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075]
    span_chord = [3.5615, 3.9127, 4.2766, 4.5753, 4.6484, 4.5489, 4.3819, 4.2206, 4.0382, 3.8449, 3.6549, 3.4713, 3.2868, 3.1022, 2.9178, 2.7332, 2.5487, 2.3691, 2.1346, 1.4683]

    prob['dof.rotor_diameter'] = 126.0
    prob['dof.thickness_factor'] = 1.0
    prob['dof.span_r'] = span_r
    prob['dof.span_dr'] = span_dr
    prob['dof.span_chord'] = span_chord 
    prob['dof.blade_number'] = 3 
     
    prob.run_model()
    
    # print outputs 
    print "Rotor Structural Design"
    print 'span_thickness = ' + beautify(prob['design.span_thickness'])
    print 'span_mass = ' + beautify(prob['design.span_mass'])
    print 'span_flap_inertia = ' + beautify(prob['design.span_flap_inertia'], 0)
    print 'span_edge_inertia = ' + beautify(prob['design.span_edge_inertia'], 0)  
    print 'span_flap_stiff = ' + beautify(prob['design.span_flap_stiff'], 0)
    print 'span_edge_stiff = ' + beautify(prob['design.span_edge_stiff'], 0) 
    print 'blade_mass = ' + beautify(prob['design.blade_mass'])   
    print 'rotor_mass = ' + beautify(prob['design.rotor_mass'])  
  
    print 'Done in ' + str(time() - start) + ' seconds'      
       