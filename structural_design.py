import numpy as np
import pandas as pd
from math import pi, degrees, radians, atan, sin
from time import time

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model
from fixed_parameters import beautify, num_nodes, num_airfoils
from airfoils import AirfoilProperties, BladeScaling, ReferenceTurbine


class StructuralDesign(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('thickness_factor', desc='scaling factor for laminate thickness')
        self.add_input('span_r', units='m', desc='spanwise dimensionless radial location', shape=num_nodes)
        self.add_input('span_chord', units='m', desc='spanwise chord length', shape=num_nodes)        
        
        # outputs
        self.add_output('span_thickness', units='m', desc='spanwise blade thickness', shape=num_nodes)
        self.add_output('span_mass', units='kg/m', desc='spanwise blade node mass per unit length', shape=num_nodes)
        self.add_output('span_flap_inertia', units = 'm**4', desc = 'spanwise second moment of flapwise area', shape=num_nodes)
        self.add_output('span_edge_inertia', units = 'm**4', desc = 'spanwise second moment of edgewise area', shape=num_nodes)
        self.add_output('span_flap_stiff', units = 'N*m**2', desc = 'spanwise flapwise stiffness', shape=num_nodes)
        self.add_output('span_edge_stiff', units = 'N*m**2', desc = 'spanwise edgewise stiffness', shape=num_nodes)







class StructuralDesignScaling(StructuralDesign):    
    
    def scale_constant_radius(mu, chord, thickness_factor):
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
    
    
    def scale_variable_radius(mu, radius):
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
    
        
    def compute(self, inputs, outputs):
        # inputs
        rotor_diameter = inputs['rotor_diameter']
        span_r = inputs['span_r']
        span_chord = inputs['span_chord']
        thickness_factor = inputs['thickness_factor']
        
        rotor_radius = rotor_diameter/2.0
        span_mu = [x/rotor_radius for x in span_r]
        span_thickness = np.array([])
        span_mass = np.array([])
        span_flap_inertia = np.array([])
        span_edge_inertia = np.array([])
        span_flap_stiff = np.array([])
        span_edge_stiff = np.array([])
        
        ref_turbine = ReferenceTurbine()  
        
        for i in range(num_nodes):
            [thickness, mass, flap_inertia, edge_inertia, \
             flap_stiff, edge_stiff] = BladeScaling(ref_turbine, span_mu[i], span_chord[i], thickness_factor, rotor_radius)
            span_thickness = np.append(span_thickness, thickness)
            span_mass = np.append(span_mass, mass)
            span_flap_inertia = np.append(span_flap_inertia, flap_inertia)
            span_edge_inertia = np.append(span_edge_inertia, edge_inertia)
            span_flap_stiff = np.append(span_flap_stiff, flap_stiff)
            span_edge_stiff = np.append(span_edge_stiff, edge_stiff)
        
        outputs['span_thickness'] = span_thickness
        outputs['span_mass'] = span_mass
        outputs['span_flap_inertia'] = span_flap_inertia
        outputs['span_edge_inertia'] = span_edge_inertia 
        outputs['span_flap_stiff'] = span_flap_stiff
        outputs['span_edge_stiff'] = span_edge_stiff