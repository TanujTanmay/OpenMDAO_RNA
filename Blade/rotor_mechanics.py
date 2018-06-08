import numpy as np
import pandas as pd
from time import time
from math import pi, radians, sin, cos
import matplotlib.pyplot as plt

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model
from fixed_parameters import num_nodes, beautify, plot_graphs, plots_folder, \
                             g, E_blade
        
        
        
#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class RotorMechanics(ExplicitComponent):
    def setup(self):        
        # inputs
        self.add_input('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        self.add_input('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        self.add_input('span_dr', units='m', desc='spanwise blade node thickness', shape=num_nodes)
        self.add_input('span_chord', units='m', desc='spanwise chord distribution', shape=num_nodes)
        self.add_input('span_thickness', units='m', desc='spanwise blade thickness', shape=num_nodes)
        self.add_input('span_mass', units='kg/m', desc='spanwise blade node mass per unit length', shape=num_nodes)
        self.add_input('span_flap_stiff', units = 'N*m**2', desc = 'spanwise flapwise stiffness', shape=num_nodes)
        self.add_input('span_edge_stiff', units = 'N*m**2', desc = 'spanwise edgewise stiffness', shape=num_nodes)
        self.add_input('span_fx', units = 'N/m', desc = 'spanwise force normal to rotation', shape=num_nodes)
        self.add_input('span_fy', units = 'N/m', desc = 'spanwise force tangential to rotation', shape=num_nodes)        
     
        # outputs
        self.add_output('root_moment_flap', units = 'N*m', desc='flapping moment at blade root')
        self.add_output('span_moment_flap', units = 'N*m', desc='spanwise flapping moment', shape=num_nodes)
        self.add_output('span_moment_edge', units = 'N*m', desc='spanwise edge-wise moment', shape=num_nodes)
        self.add_output('span_moment_gravity', units = 'N*m', desc='spanwise edge-wise moment', shape=num_nodes)
        self.add_output('span_stress_flap', units = 'Pa', desc='spanwise flapping stress', shape=num_nodes)
        self.add_output('span_stress_edge', units = 'Pa', desc='spanwise edgewise stress', shape=num_nodes)
        self.add_output('span_stress_gravity', units = 'Pa', desc='spanwise gravity loading', shape=num_nodes)
        self.add_output('span_stress_max', units = 'Pa', desc='maximum loading')
        self.add_output('tip_deflection', units = 'm', desc='tip deflection from neutral point')
 







class  Analytical(RotorMechanics):
    def compute(self, inputs, outputs):
        # inputs
        shaft_angle = radians(abs(inputs['shaft_angle']))
        span_r = inputs['span_r']
        span_dr = inputs['span_dr']
        span_chord = inputs['span_chord']
        span_thickness = inputs['span_thickness']
        span_mass = inputs['span_mass']
        span_flap_stiff = inputs['span_flap_stiff']
        span_edge_stiff = inputs['span_edge_stiff']
        span_fx = inputs['span_fx']
        span_fy = inputs['span_fy']
        
        # to store information at each blade node 
        zeroes = [0.0] * num_nodes
        nodes = pd.DataFrame(data={ 'r' : span_r, \
                                    'dr' : span_dr, \
                                    'chord' : span_chord, \
                                    'thick': span_thickness, \
                                    'mass' : span_mass, \
                                    'EI_flap' : span_flap_stiff, \
                                    'EI_edge' : span_edge_stiff, \
                                    'I_flap' : [x/E_blade for x in span_flap_stiff], \
                                    'I_edge' : [x/E_blade for x in span_edge_stiff], \
                                    'Fx' : span_fx, \
                                    'Fy' : span_fy, \
                                    'M_flap' : zeroes, \
                                    'M_edge' : zeroes, \
                                    'M_grav' : zeroes, \
                                    'S_flap' : zeroes, \
                                    'S_edge' : zeroes, \
                                    'S_grav' : zeroes, \
                                    'S_max' : zeroes, \
                                    'M_EI' : zeroes, \
                                    'dy_dx': zeroes, \
                                    'y': zeroes
                                } 
                            )
               
        # analytical calculations       
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
            nodes.loc[i, 'S_max'] = nodes.loc[i, 'S_flap'] + nodes.loc[i, 'S_edge'] + nodes.loc[i, 'S_grav']
            nodes.loc[i, 'M_EI']   = nodes.loc[i, 'M_flap'] / nodes.loc[i, 'EI_flap']
            
            if i > 0:
                nodes.loc[i, 'dy_dx'] = nodes.loc[i-1, 'dy_dx'] + \
                                        (nodes.loc[i, 'M_EI'] + nodes.loc[i-1, 'M_EI'])/2.0 * nodes.loc[i, 'dr']
                nodes.loc[i, 'y'] = nodes.loc[i-1, 'y'] + \
                                        (nodes.loc[i, 'dy_dx'] + nodes.loc[i-1, 'dy_dx'])/2.0 * nodes.loc[i, 'dr']          
            
        
        #print nodes
        
        # plot and save
        if plot_graphs:
            params = ['thick', 'mass', 'EI_flap', 'M_flap', 'M_edge', 'M_grav', 'S_flap', 'S_edge', 'S_grav']
            for param in params:
                plt.close('all')
                plt.plot(np.array(nodes['r']), np.array(nodes[param]))
                plt.xlabel('Span [m]')
                plt.ylabel(param)
                #plt.show()
                plt.savefig(plots_folder + param + '.png')
        
        # outputs    
        outputs['root_moment_flap'] = nodes.loc[0, 'M_flap']
        outputs['span_moment_flap'] = np.array(nodes['M_flap'])    
        outputs['span_moment_edge'] = np.array(nodes['M_edge'])  
        outputs['span_moment_gravity'] = np.array(nodes['M_grav'])
        outputs['span_stress_flap'] = np.array(nodes['S_flap'])  
        outputs['span_stress_edge'] = np.array(nodes['S_edge'])  
        outputs['span_stress_gravity'] = np.array(nodes['S_grav']) 
        outputs['span_stress_max'] = np.array(nodes['S_max']).max()              
        outputs['tip_deflection'] = nodes.loc[num_nodes-1, 'y']  

        #print 'Tip deflection = ' + str(nodes.loc[num_nodes-1, 'y'])
        #print '-'*10
        
        
        







#############################################################################
##############################  UNIT TESTING ################################
#############################################################################
class RotorMechanicsTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        i.add_output('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        i.add_output('span_dr', units='m', desc='list of blade node thickness', shape=num_nodes)
        i.add_output('span_chord', units='m', desc='spanwise chord distribution', shape=num_nodes)
        i.add_output('span_thickness', units='m', desc='spanwise blade thickness', shape=num_nodes)
        i.add_output('span_mass', units='kg/m', desc='spanwise blade node mass per unit length', shape=num_nodes)
        i.add_output('span_flap_stiff', units = 'N*m**2', desc = 'spanwise flapwise stiffness', shape=num_nodes)
        i.add_output('span_edge_stiff', units = 'N*m**2', desc = 'spanwise flapwise stiffness', shape=num_nodes)
        i.add_output('span_fx', units = 'N/m', desc = 'spanwise force normal to rotation', shape=num_nodes)
        i.add_output('span_fy', units = 'N/m', desc = 'spanwise force tangential to rotation', shape=num_nodes)        
        
        # sub-components
        self.add_subsystem('dof', i, promotes=['*']) 
        self.add_subsystem('mech', Analytical(), promotes_inputs=['*']) 


if __name__ == "__main__":
    
    # test parameters
    shaft_angle = -5.0
    span_r = [3.0375, 6.1125, 9.1875, 12.2625, 15.3375, 18.4125, 21.4875, 24.5625, 27.6375, 30.7125, 33.7875, 36.8625, 39.9375, 43.0125, 46.0875, 49.1625, 52.2375, 55.3125, 58.3875, 61.4625]
    span_dr = [3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075]
    span_chord = [3.5615, 3.9127, 4.2766, 4.5753, 4.6484, 4.5489, 4.3819, 4.2206, 4.0382, 3.8449, 3.6549, 3.4713, 3.2868, 3.1022, 2.9178, 2.7332, 2.5487, 2.3691, 2.1346, 1.4683]
    span_thickness = [3.5614, 3.3798, 2.7023, 2.1252, 1.7103, 1.5501, 1.414, 1.2486, 1.0955, 0.9645, 0.8542, 0.767, 0.664, 0.6017, 0.5409, 0.4944, 0.4608, 0.42, 0.3861, 0.2655]
    span_mass = [697.7655, 679.6009, 413.8751, 410.6307, 384.1854, 346.2471, 333.3628, 320.0997, 298.2267, 275.5935, 252.3555, 227.5372, 193.881, 166.5232, 146.1523, 126.5607, 100.8119, 85.5755, 62.0602, 17.4553]
    span_flap_stiff = [1.8371E+10, 1.3871E+10, 5.9623E+09, 4.7197E+09, 3.0085E+09, 2.2642E+09, 1.9079E+09, 1.5313E+09, 1.1402E+09, 7.8028E+08, 5.2445E+08, 3.4265E+08, 2.1866E+08, 1.3169E+08, 9.8637E+07, 7.4358E+07, 5.2445E+07, 3.6103E+07, 2.0647E+07, 1.5998E+06]
    span_edge_stiff = [1.8403E+10, 1.7684E+10, 8.7006E+09, 7.1049E+09, 6.3176E+09, 4.7961E+09, 4.3343E+09, 3.9345E+09, 3.4919E+09, 2.9340E+09, 2.5390E+09, 1.9896E+09, 1.5029E+09, 1.1941E+09, 8.9622E+08, 6.8351E+08, 4.6979E+08, 3.6768E+08, 2.1335E+08, 2.0813E+07]
    span_fx = [0.0, 152.8413, 692.3132, 1231.7851, 1771.257, 2077.937, 2385.88, 2693.823, 3167.965, 3613.9953, 4060.0257, 4506.056, 4882.4425, 5258.829, 5635.2155, 6011.602, 5169.536, 4327.47, 3485.404, 2222.305]
    span_fy = [-3.5, -103.8047, 162.8256, 429.456, 696.0863, 702.4778, 701.8906, 701.3033, 720.8832, 725.5557, 730.2281, 734.9006, 732.7446, 730.5885, 728.4325, 726.2764, 558.9978, 391.7192, 224.4406, -26.4774]

     
    start = time()
    
    # workflow setup
    prob = Problem(RotorMechanicsTest())
    prob.setup()
    view_model(prob, outfile='N2/rotor_structure.html')
    
    # define inputs
    prob['dof.shaft_angle'] = shaft_angle
    prob['dof.span_r'] = span_r
    prob['dof.span_dr'] = span_dr
    prob['dof.span_chord'] = span_chord
    prob['dof.span_thickness'] = span_thickness
    prob['dof.span_mass'] = span_mass
    prob['dof.span_flap_stiff'] = span_flap_stiff
    prob['dof.span_edge_stiff'] = span_edge_stiff
    prob['dof.span_fx'] = span_fx
    prob['dof.span_fy'] = span_fy    
     
    prob.run_model()
    
    # print outputs  
    print "Rotor Mechanics"
    print 'span_moment_flap = ' + beautify(prob['mech.span_moment_flap'])
    print 'span_moment_edge = ' + beautify(prob['mech.span_moment_edge'])
    print 'span_moment_gravity = ' + beautify(prob['mech.span_moment_gravity'])
    print 'span_stress_flap = ' + beautify(prob['mech.span_stress_flap'])
    print 'span_stress_edge = ' + beautify(prob['mech.span_stress_edge'])
    print 'span_stress_gravity = ' + beautify(prob['mech.span_stress_gravity'])
    print 'span_stress_max = ' + beautify(prob['mech.span_stress_max'])
    print 'tip_deflection = ' + beautify(prob['mech.tip_deflection'])
     
    print 'Done in ' + str(time() - start) + 'seconds'  

        
    
    
    
    
            