from math import pi, radians, sin, cos
import numpy as np
import pandas as pd
from time import time
import matplotlib.pyplot as plt

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model
from fixed_parameters import num_nodes, g, beautify, plot_graphs


class BladeMass(ExplicitComponent):
    def setup(self):
        # inputs        
        self.add_input('blade_number', desc = 'number of blades')
        self.add_input('span_dr', units='m', desc='list of blade node thickness', shape=num_nodes)
        self.add_input('span_mass', units='kg/m', desc='list of blade node mass per unit length', shape=num_nodes)        
        
        # outputs
        self.add_output('blade_mass', units = 'kg', desc='mass of the one blade')
        self.add_output('rotor_mass', units = 'kg', desc='mass of the rotor (all blades)')
        
        
    def compute(self, inputs, outputs):
        
        span_dr =   inputs['span_dr']
        span_mass =   inputs['span_mass']  
        blade_number =   inputs['blade_number']   
        
        outputs['blade_mass'] = np.dot(span_mass, span_dr)
        outputs['rotor_mass'] = outputs['blade_mass'] * blade_number
        
        

class BladeState(ExplicitComponent):
    def setup(self):
        
        # inputs
        self.add_input('rotor_thrust',  units='N', desc='rotor thrust')
        self.add_input('rotor_mass', units = 'kg', desc='mass of the rotor (all blades)')
        self.add_input('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        self.add_input('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        self.add_input('span_dr', units='m', desc='spanwise blade node thickness', shape=num_nodes)
        self.add_input('span_chord', units='m', desc='spanwise chord distribution', shape=num_nodes)
        self.add_input('span_thickness', units='m', desc='spanwise blade thickness', shape=num_nodes)
        self.add_input('span_mass', units='kg/m', desc='spanwise blade node mass per unit length', shape=num_nodes)
        self.add_input('span_flap_inertia', units = 'm**4', desc = 'spanwise second moment of area', shape=num_nodes)
        self.add_input('span_edge_inertia', units = 'm**4', desc = 'spanwise second moment of area', shape=num_nodes)
        self.add_input('span_flap_stiff', units = 'N*m**2', desc = 'spanwise flapwise stiffness', shape=num_nodes)
        #self.add_input('span_edge_stiff', units = 'N*m**2', desc = 'spanwise edgewise stiffness', shape=num_nodes)
        self.add_input('span_fx', units = 'N/m', desc = 'spanwise force normal to rotation', shape=num_nodes)
        self.add_input('span_fy', units = 'N/m', desc = 'spanwise force tangential to rotation', shape=num_nodes)        
     
        # outputs
        self.add_output('span_moment_flap', units = 'N*m', desc='spanwise flapping moment', shape=num_nodes)
        self.add_output('span_moment_edge', units = 'N*m', desc='spanwise edge-wise moment', shape=num_nodes)
        self.add_output('span_stress_flap', units = 'Pa', desc='spanwise flapping stress', shape=num_nodes)
        self.add_output('span_stress_edge', units = 'Pa', desc='spanwise edgewise stress', shape=num_nodes)
        self.add_output('span_stress_gravity', units = 'Pa', desc='spanwise gravity loading', shape=num_nodes)
        self.add_output('tip_deflection', units = 'm', desc='tip deflection from neutral point')
        self.add_output('root_bending_moment', units = 'N*m', desc='bending moment vector at blade root', shape=3)
        self.add_output('root_force', units = 'N', desc='force vector at the hub', shape=3)
 
 
    def compute(self, inputs, outputs):
        
        rotor_thrust = inputs['rotor_thrust'] 
        rotor_mass =  inputs['rotor_mass'] 
        shaft_angle = radians(inputs['shaft_angle'])
        span_r = inputs['span_r']
        span_dr = inputs['span_dr']
        span_chord = inputs['span_chord']
        span_thickness = inputs['span_thickness']
        span_mass = inputs['span_mass']
        span_flap_inertia = inputs['span_flap_inertia']
        span_edge_inertia = inputs['span_edge_inertia']
        span_flap_stiff = inputs['span_flap_stiff']
        span_fx = inputs['span_fx']
        span_fy = inputs['span_fy']
         
        zeroes = [0.0] * num_nodes
        
        nodes = pd.DataFrame(data={ 'r' : span_r, \
                                    'dr' : span_dr, \
                                    'chord' : span_chord, \
                                    'thick': span_thickness, \
                                    'mass' : span_mass, \
                                    'I_flap' : span_flap_inertia, \
                                    'I_edge' : span_edge_inertia, \
                                    'EI_flap' : span_flap_stiff, \
                                    'Fx' : span_fx, \
                                    'Fy' : span_fy, \
                                    'M_flap' : zeroes, \
                                    'M_edge' : zeroes, \
                                    'M_grav' : zeroes, \
                                    'S_flap' : zeroes, \
                                    'S_edge' : zeroes, \
                                    'S_grav' : zeroes, \
                                    'M_EI' : zeroes, \
                                    'dy_dx': zeroes, \
                                    'y': zeroes
                                } 
                            )
               
               
        for i in range(num_nodes): # loop through each i, call NODE_index
            for j in range(i, num_nodes): # j is the i larger than NODE_index
                nodes.loc[i, 'M_flap'] += (nodes.loc[j, 'Fx'] * nodes.loc[j, 'dr'] + \
                                            nodes.loc[j, 'mass'] * nodes.loc[j, 'dr'] * g * sin(shaft_angle)) * \
                                            (nodes.loc[j, 'r'] - nodes.loc[i, 'r'])
                nodes.loc[i, 'M_edge'] += nodes.loc[j, 'Fy'] * nodes.loc[j, 'dr'] * (nodes.loc[j, 'r'] - nodes.loc[i, 'r'])
                nodes.loc[i, 'M_grav'] += nodes.loc[j, 'mass'] * nodes.loc[j, 'dr'] * g * cos(shaft_angle) * \
                                            (nodes.loc[j, 'r'] - nodes.loc[i, 'r'])                                    
           
            nodes.loc[i, 'S_flap'] = nodes.loc[i, 'M_flap'] * 0.5 * nodes.loc[i, 'thick'] / nodes.loc[i, 'I_flap']
            nodes.loc[i, 'S_edge'] = nodes.loc[i, 'M_edge'] * 0.75 * nodes.loc[i, 'chord'] / nodes.loc[i, 'I_edge']
            nodes.loc[i, 'S_grav'] = nodes.loc[i, 'M_grav'] * 0.75 * nodes.loc[i, 'chord'] / nodes.loc[i, 'I_edge']
            nodes.loc[i, 'M_EI']   = nodes.loc[i, 'M_flap'] / nodes.loc[i, 'EI_flap']
            
            if i > 0:
                nodes.loc[i, 'dy_dx'] = nodes.loc[i-1, 'dy_dx'] + \
                                        (nodes.loc[i, 'M_EI'] + nodes.loc[i-1, 'M_EI'])/2.0 * nodes.loc[i, 'dr']
                nodes.loc[i, 'y'] = nodes.loc[i-1, 'y'] + \
                                        (nodes.loc[i, 'dy_dx'] + nodes.loc[i-1, 'dy_dx'])/2.0 * nodes.loc[i, 'dr']          
            
        
        #print nodes
        
        # Plot and save
        if plot_graphs:
            params = ['thick', 'mass', 'EI_flap', 'M_flap', 'M_edge', 'M_grav', 'S_flap', 'S_edge', 'S_grav']
            for param in params:
                # save the plots
                plt.close('all')
                plt.plot(np.array(nodes['r']), np.array(nodes[param]))
                plt.xlabel('Span [m]')
                plt.ylabel(param)
                #plt.show()
                plt.savefig('Plots/' + param + '.png')
            
        outputs['span_moment_flap'] = np.array(nodes['M_flap'])    
        outputs['span_moment_edge'] = np.array(nodes['M_edge'])  
        outputs['span_stress_flap'] = np.array(nodes['S_flap'])  
        outputs['span_stress_edge'] = np.array(nodes['S_edge'])  
        outputs['span_stress_gravity'] = np.array(nodes['S_grav'])              
        outputs['tip_deflection'] = nodes.loc[num_nodes-1, 'y']  
        
        outputs['root_bending_moment'] = [nodes.loc[0, 'M_edge'] + nodes.loc[0, 'M_grav'], \
                                           0, \
                                           0]
        
#                                           [nodes.loc[0, 'M_edge'] + nodes.loc[0, 'M_grav'], \
#                                           nodes.loc[0, 'M_flap'], \
#                                           0]
        
        hub_cd = 0.42 # Applied Fluid Dynamics Handbook, Blevins: https://www.lmnoeng.com/Drag/index.php
        hub_drag = 0.0 # hub_cd * 0.5 * rho_air *  (wind_speed**2) * (pi * hub_radius**2)
        fx = rotor_thrust + rotor_mass * g * sin(shaft_angle) + hub_drag
        fy = 0 # cancels for all blades
        fz = -1 * rotor_mass * g * cos(shaft_angle)
            
        outputs['root_force'] = [fx, fy, fz]
        
        
        
        




class RotorStructure(Group):
    def setup(self):        
        self.add_subsystem('mass', BladeMass(), \
                           promotes_inputs = ['blade_number', 'span_dr', 'span_mass'], \
                           promotes_outputs=['blade_mass', 'rotor_mass'])
        
        self.add_subsystem('state', BladeState(), \
                           promotes_inputs = ['rotor_thrust', 'shaft_angle', \
                                              'span_r', 'span_dr', 'span_chord', 'span_thickness', \
                                              'span_mass', 'span_flap_inertia', 'span_edge_inertia', \
                                              'span_flap_stiff', 'span_fx', 'span_fy'], \
                           promotes_outputs=['*'])
        
        self.connect('rotor_mass', 'state.rotor_mass')



class RotorStructureTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('blade_number', desc = 'number of blades')
        i.add_output('rotor_thrust',  units='N', desc='rotor thrust')
        i.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        i.add_output('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        i.add_output('span_dr', units='m', desc='list of blade node thickness', shape=num_nodes)
        i.add_output('span_chord', units='m', desc='spanwise chord distribution', shape=num_nodes)
        i.add_output('span_thickness', units='m', desc='spanwise blade thickness', shape=num_nodes)
        i.add_output('span_mass', units='kg/m', desc='spanwise blade node mass per unit length', shape=num_nodes)
        i.add_output('span_flap_inertia', units = 'm**4', desc = 'spanwise second moment of area', shape=num_nodes)
        i.add_output('span_edge_inertia', units = 'm**4', desc = 'spanwise second moment of area', shape=num_nodes)
        i.add_output('span_flap_stiff', units = 'N*m**2', desc = 'spanwise flapwise stiffness', shape=num_nodes)
        i.add_output('span_fx', units = 'N/m', desc = 'spanwise force normal to rotation', shape=num_nodes)
        i.add_output('span_fy', units = 'N/m', desc = 'spanwise force tangential to rotation', shape=num_nodes)        
        
        # sub-components
        self.add_subsystem('dof', i) 
        self.add_subsystem('struc', RotorStructure()) 
        
        # connections
        self.connect('dof.blade_number', 'struc.blade_number')
        self.connect('dof.rotor_thrust', 'struc.rotor_thrust')
        self.connect('dof.shaft_angle', 'struc.shaft_angle')
        self.connect('dof.span_r', 'struc.span_r')
        self.connect('dof.span_dr', 'struc.span_dr')
        self.connect('dof.span_chord', 'struc.span_chord')
        self.connect('dof.span_thickness', 'struc.span_thickness')
        self.connect('dof.span_mass', 'struc.span_mass')
        self.connect('dof.span_flap_inertia', 'struc.span_flap_inertia')
        self.connect('dof.span_edge_inertia', 'struc.span_edge_inertia')
        self.connect('dof.span_flap_stiff', 'struc.span_flap_stiff')
        self.connect('dof.span_fx', 'struc.span_fx')
        self.connect('dof.span_fy', 'struc.span_fy')
        






if __name__ == "__main__":

    
    blade_number = 3
    rotor_thrust = 370177.59
    shaft_angle = 6.0
    span_r = [6.5, 11.5, 16.5, 21.5, 26.5, 31.5, 36.5, 41.5, 46.5, 51.5]
    span_dr = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
    span_chord = [3.4, 3.4, 4.2, 3.2, 1.8, 1.5, 1.3, 1.1, 1.0, 0.9]
    span_thickness = [2.5, 1.4, 1.4, 0.9, 0.4, 0.3, 0.3, 0.2, 0.2, 0.2]
    span_mass = [344.0, 227.7, 295.5, 189.9, 59.0, 42.8, 29.2, 22.4, 16.3, 11.9]
    span_flap_inertia = [5.8E+02, 7.1E+01, 1.8E+01, 4.2E+00, 1.6E-01, 5.4E-02, 1.4E-02, 4.1E-03, 2.1E-03, 1.4E-03]
    span_edge_inertia = [1.1E+02, 5.9E+01, 9.5E+01, 3.3E+01, 2.6E+00, 1.4E+00, 8.0E-01, 5.3E-01, 3.1E-01, 1.9E-01]
    span_flap_stiff = [4.0E+09, 1.2E+09, 1.6E+09, 5.2E+08, 3.5E+07, 1.2E+07, 4.1E+06, 2.3E+06, 1.3E+06, 5.7E+05]
    span_fx = [0.0, 0.0, 1532.4, 1985.5, 2480.0, 2913.0, 3377.5, 3714.4, 4191.7, 4484.1]
    span_fy = [0.0, 0.0, 345.6, 346.8, 345.2, 342.9, 339.7, 335.5, 326.4, 280.7]
     
    start = time()
    
    # workflow setup
    prob = Problem(RotorStructureTest())
    prob.setup()
    view_model(prob, outfile='rotor_structure_siemens.html')
    
    # define inputs
    prob['dof.blade_number'] = blade_number
    prob['dof.rotor_thrust'] = rotor_thrust
    prob['dof.shaft_angle'] = shaft_angle
    prob['dof.span_r'] = span_r
    prob['dof.span_dr'] = span_dr
    prob['dof.span_chord'] = span_chord
    prob['dof.span_thickness'] = span_thickness
    prob['dof.span_mass'] = span_mass
    prob['dof.span_flap_inertia'] = span_flap_inertia
    prob['dof.span_edge_inertia'] = span_edge_inertia
    prob['dof.span_flap_stiff'] = span_flap_stiff
    prob['dof.span_fx'] = span_fx
    prob['dof.span_fy'] = span_fy    
     
    prob.run_model()
    
    # print outputs  
    print "Rotor Structure"
    print 'blade_mass = ' + beautify(prob['struc.blade_mass'])
    print 'rotor_mass = ' + beautify(prob['struc.rotor_mass'])
    print 'span_moment_flap = ' + beautify(prob['struc.span_moment_flap'])
    print 'span_moment_edge = ' + beautify(prob['struc.span_moment_edge'])
    print 'span_stress_flap = ' + beautify(prob['struc.span_stress_flap'])
    print 'span_stress_edge = ' + beautify(prob['struc.span_stress_edge'])
    print 'span_stress_gravity = ' + beautify(prob['struc.span_stress_gravity'])
    print 'tip_deflection = ' + beautify(prob['struc.tip_deflection'])
    print 'root_bending_moment = ' + beautify(prob['struc.root_bending_moment'])
    print 'root_force = ' + beautify(prob['struc.root_force'])
     
    print 'Done in ' + str(time() - start) + 'seconds'  

        
    
    
    
    
            