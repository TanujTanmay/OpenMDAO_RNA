import numpy as np
import pandas as pd
from math import pi, degrees, radians, atan, sin
from time import time

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model
from fixed_parameters import num_airfoils, airfoil_folder, airfoils_db, num_nodes, beautify
from airfoils import AirfoilProperties, ReferenceTurbine, BladeScaling



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
        
    def compute(self, inputs, outputs):
        
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
        
        mu_ = np.linspace(mu_root, 1.0, num_nodes+1)
        
        span_mu = np.array([(x+y)/2.0 for x,y in zip(mu_[:-1], mu_[1:])])
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
            cl_opt = airfoils.loc[airfoil_id, 'cl_opt']
            alpha_opt = airfoils.loc[airfoil_id, 'alpha_opt']
            
            phi_opt = degrees(atan(2.0/(3.0 * tsr_r)))
            #print 'cl_opt = ' + str(cl_opt*2)
            
            if cl_opt == 0.0:
                chord = root_chord
                twist = 0.0  
            else:
                chord = (8*pi*rotor_radius*sin(radians(phi_opt)))/(3*b*cl_opt*tsr)
                twist = phi_opt - alpha_opt
            
            span_airfoil = np.append(span_airfoil, airfoil_id)
            span_chord = np.append(span_chord, chord)
            span_twist = np.append(span_twist, twist) 
            
#             # update the chord length of cylindrical section as the chord length of first airfoil section
#             if cylinder_correction == 0 and cl_opt > 0:
#                 span_chord[span_chord == 0] = chord
#                 cylinder_correction = 1

        
        # pitch the blade to make tip twist angle as 0
        if(adjust_pitch):
            pitch = span_twist[-1]
            span_twist = np.subtract(span_twist, pitch)
        else:
            pitch = 0
            
#         span_chord = [3.55, 3.95, 4.30, 4.57, 4.65, 4.56, 4.38, 4.22, 4.03, 3.84, 3.65, 3.47, 3.29, 3.09, 2.91, 2.72, 2.55, 2.37, 2.12, 1.38]
#         span_twist = [13.2, 13.2, 13.2, 13.1, 11.5, 10.6, 9.68, 8.79, 7.84, 6.92, 6.05, 5.18, 4.26, 3.39, 2.75, 2.00, 1.58, 0.95, 0.35, 0.02]    
#         pitch = -0.09
#         s = rotor_diameter/126.0
#         span_chord = np.array([x*s for x in span_chord])
#         span_chord = np.resize(span_chord, num_nodes)
        
        
        outputs['span_r'] = span_r
        outputs['span_dr'] = span_dr
        outputs['span_airfoil'] = span_airfoil
        outputs['span_chord'] = span_chord
        outputs['span_twist'] = span_twist
        outputs['pitch'] = pitch
        
        
        
           
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
            
    def compute(self, inputs, outputs):
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



class RotorDesign(Group):
    def setup(self):
             
        self.add_subsystem('aero', AerodynamicDesign(), \
                           promotes_inputs = ['*'], \
                           promotes_outputs=['*']
                           )
        
        self.add_subsystem('struc', StructuralDesign(), \
                           promotes_inputs = ['rotor_diameter', 'thickness_factor'], \
                           promotes_outputs=['*'])
        
        
        self.connect('span_r', 'struc.span_r')  
        self.connect('span_chord', 'struc.span_chord') 


        
class RotorDesignTest(Group):
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
        i.add_output('thickness_factor', desc='scaling factor for laminate thickness')

        # sub-components        
        self.add_subsystem('dof', i)   
        self.add_subsystem('design', RotorDesign())
               
        # connections
        self.connect('dof.design_tsr', 'design.design_tsr')
        self.connect('dof.blade_number', 'design.blade_number')
        self.connect('dof.rotor_diameter', 'design.rotor_diameter')
        self.connect('dof.hub_radius', 'design.hub_radius')
        self.connect('dof.root_chord', 'design.root_chord')
        self.connect('dof.span_airfoil_r', 'design.span_airfoil_r')
        self.connect('dof.span_airfoil_id', 'design.span_airfoil_id')
        self.connect('dof.adjust_pitch', 'design.adjust_pitch')
        self.connect('dof.thickness_factor', 'design.thickness_factor')
        

        
 


if __name__ == "__main__":
    start = time()
    
    # workflow setup
    prob = Problem(RotorDesignTest())
    prob.setup()
    view_model(prob, outfile='rotor_design_siemens.html')
    
    # define inputs
    prob['dof.design_tsr'] = 9.0
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 108.0
    prob['dof.hub_radius'] = 4.0
    prob['dof.root_chord'] = 3.4
    prob['dof.span_airfoil_r'] = [4.0, 16.2, 25.0]
    prob['dof.span_airfoil_id'] = [0, 1, 2]
    prob['dof.adjust_pitch'] = 1    
    prob['dof.thickness_factor'] = 1.0
     
    prob.run_model()
    
    # print outputs 
    print "Rotor Design"
    print 'span_r = ' + beautify(prob['design.span_r'])
    print 'span_dr = ' + beautify(prob['design.span_dr'])
    print 'span_airfoil = ' + beautify(prob['design.span_airfoil'])
    print 'span_chord = ' + beautify(prob['design.span_chord'])
    print 'span_twist = ' + beautify(prob['design.span_twist'])
    print 'pitch = ' + beautify(prob['design.pitch']) 
    print 'span_thickness = ' + beautify(prob['design.span_thickness'])
    print 'span_mass = ' + beautify(prob['design.span_mass'])
    print 'span_flap_inertia = ' + beautify(prob['design.span_flap_inertia'], 0)
    print 'span_edge_inertia = ' + beautify(prob['design.span_edge_inertia'], 0)  
    print 'span_flap_stiff = ' + beautify(prob['design.span_flap_stiff'], 0)
    print 'span_edge_stiff = ' + beautify(prob['design.span_edge_stiff'], 0)       
  
    print 'Done in ' + str(time() - start) + ' seconds'
    


 