import numpy as np
from time import time

from openmdao.api import Problem, view_model 
from Blade.fixed_parameters import num_nodes, params, beautify
from rna import RNATest









def NREL():

    start = time()
    
    # workflow setup
    prob = Problem(RNATest())
    prob.setup()
    #view_model(prob, outfile='rna.html')
    
    # define inputs 
    prob['dof.design_tsr'] = 7.6
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 126.0
    prob['dof.hub_radius'] = 1.5 
    prob['dof.root_chord'] = 3.542
    prob['dof.chord_coefficients'] = [3.542, 3.01, 2.313]
    prob['dof.twist_coefficients'] = [13.308, 9.0, 3.125]
    prob['dof.span_airfoil_r'] =  [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
    prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]
    prob['dof.pitch'] = 0.0
    prob['dof.thickness_factor'] = [1.0]*num_nodes
    prob['dof.hub_height'] = 90.0
    prob['dof.precone'] = -2.5
    prob['dof.yaw'] = 0.0
    prob['dof.overhang'] = 5.0191
    prob['dof.shaft_angle'] = -5.0
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 5000.0
    prob['dof.drive_train_efficiency'] = 0.95
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.Np'] = [3,3,1]
    prob['dof.gear_ratio'] = 96.76
    prob['dof.tower_top_diameter'] = 3.78
    
    prob.run_model()
    
    for p in params:
        beautify(p, prob['rna.'+p])    
    
    print 'Done in ' + str(time() - start) + ' seconds'
          


def SWT():

    start = time()
    
    # workflow setup
    prob = Problem(RNATest())
    prob.setup()
    #view_model(prob, outfile='rna.html')
    
    # define inputs 
    prob['dof.design_tsr'] = 7.4
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 108.0
    prob['dof.hub_radius'] = 2.3
    prob['dof.root_chord'] = 3.4
    prob['dof.chord_coefficients'] = np.array([3.542, 3.01, 2.313])*108/126.
    prob['dof.twist_coefficients'] = np.array([13.308, 9.0, 3.125])
    prob['dof.span_airfoil_r'] =  np.array([01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05])*108/126.
    prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]
    prob['dof.pitch'] = 0.0
    prob['dof.thickness_factor'] = [0.79]*num_nodes
    prob['dof.hub_height'] = 80.0
    prob['dof.precone'] = -2.5
    prob['dof.yaw'] = 0.0
    prob['dof.overhang'] = 4.3
    prob['dof.shaft_angle'] = -6.0
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 2300.0
    prob['dof.drive_train_efficiency'] = 0.95
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.Np'] = [3,3,1]
    prob['dof.gear_ratio'] = 91.
    prob['dof.tower_top_diameter'] = 3.78 * 108/126.
    
    prob.run_model()
    
    for p in params:
        beautify(p, prob['rna.'+p])

    
    
    print 'Done in ' + str(time() - start) + ' seconds'
    
    
def DTU():

    start = time()
    
    # workflow setup
    prob = Problem(RNATest())
    prob.setup()
    #view_model(prob, outfile='rna.html')
    
    # rated RPM = 9.6; Rated wind speed = 11.4
    
    # define inputs 
    prob['dof.design_tsr'] = 7.86
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 178.3
    prob['dof.hub_radius'] = 2.8
    prob['dof.root_chord'] = 3.542*178.3/126.
    prob['dof.chord_coefficients'] = [3.542, 3.01, 2.313]
    prob['dof.twist_coefficients'] = [13.308, 9.0, 3.125]
    prob['dof.span_airfoil_r'] =  np.array([01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05])*178.3/126.
    prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]
    prob['dof.pitch'] = 0.0
    prob['dof.thickness_factor'] = [1.14]*num_nodes
    prob['dof.hub_height'] = 119.
    prob['dof.precone'] = -2.5
    prob['dof.yaw'] = 0.0
    prob['dof.overhang'] = 7.07
    prob['dof.shaft_angle'] = -5.0
    prob['dof.cut_in_speed'] = 4.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 10000.0
    prob['dof.drive_train_efficiency'] = 0.95
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.Np'] = [3,3,1]
    prob['dof.gear_ratio'] = 50.
    prob['dof.tower_top_diameter'] = 3.78 * 178.3/126.
    
    prob.run_model()
    
    for p in params:
        beautify(p, prob['rna.'+p])

    
    
    print 'Done in ' + str(time() - start) + ' seconds'    
            
        
   
    
if __name__ == "__main__":
    NREL()
    #SWT()
    #DTU()