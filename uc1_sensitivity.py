import numpy as np
import pandas as pd
from time import time
import matplotlib.pyplot as plt

from openmdao.api import Problem, view_model 
from Blade.fixed_parameters import num_nodes, params, beautify
from rna import RNATest
      
def driver(b=3, d=126.0, tsr=7.6, tf=1.0, cm=1.0, p_rated=5000., eta_dt=0.95, r_gb=96.76):
    
    # workflow setup
    prob = Problem(RNATest())
    prob.setup()
    #view_model(prob, outfile='rna.html')
    
    s=d/126.0
    
    # define inputs 
    prob['dof.design_tsr'] = tsr
    prob['dof.blade_number'] = b
    prob['dof.rotor_diameter'] = d
    prob['dof.hub_radius'] = 1.5 * s
    prob['dof.root_chord'] = 3.542 * s
    prob['dof.chord_coefficients'] = np.array([3.542, 3.01, 2.313])*s*cm
    prob['dof.twist_coefficients'] = np.array([13.308, 9.0, 3.125])
    prob['dof.span_airfoil_r'] =  np.array([01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05])*s
    prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]
    prob['dof.pitch'] = 0.0
    prob['dof.thickness_factor'] = [tf]*num_nodes
    prob['dof.hub_height'] = 90.0 * s
    prob['dof.precone'] = -2.5
    prob['dof.yaw'] = 0.0
    prob['dof.overhang'] = 5.0191 * s
    prob['dof.shaft_angle'] = -5.0
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = p_rated
    prob['dof.drive_train_efficiency'] = eta_dt
    prob['dof.gearbox_cm_x'] = 0.1 * s
    prob['dof.Np'] = [3,3,1]
    prob['dof.gear_ratio'] = r_gb
    prob['dof.tower_top_diameter'] = 3.78 * s
    
    prob.run_model()
    
    res = {'blades' : b, \
           'diameter' : d, \
           'tsr' : tsr, \
           'thickness_factor' : tf, \
           'chord_multiplier' : cm, \
           'p_rated' : p_rated, \
           'eta_dt' : eta_dt, \
           'r_gb' : r_gb}
    
    for p in params:
        res[p] = prob['rna.'+p]
        #beautify(p, prob['rna.'+p])    
 

    return res    



def sensitivity():
    
    ################################################
    ############## Setup Plots #####################
    ################################################
    font = {'family' : 'Tahoma', 'size' : 15}
    plt.rc('font', **font)
    f1= plt.figure(1)
    
    x1 = f1.add_subplot(111)
    #x1.set_title('Chord distribution')
    x1.set_xlabel('Gearbox ratio [-]')
    x1.set_ylabel('Mass [kg]')
    
    
    ################################################
    ############## Reference Run ###################
    ################################################
    ref = driver()
    gb_mass = ref['nacelle.gearbox_mass']
    nacelle_mass = ref['nacelle_mass']
    
    
    
        
    
#     ################################################
#     ##### Tip Speed Ratio / Number Blades ##########
#     ################################################
#     range_tsr = np.linspace(5,15,11)
#     result=[]
#     for tsr in range_tsr:
#         for b in [2,3,4]:
#             res = driver(b=b, d=126.0, tsr=tsr, tf=1.0, cm=1.0, p_rated=5000., eta_dt=0.95, r_gb=96.76)
#             result.append(res)
#             
#     result = pd.DataFrame(result)
#     
#     # 3 blades
#     res3 = result[result['blades'] == 3]
#     plt.plot(res3['tsr'], res3['rotor_cp'], label='3 blades')
#     plt.legend()
#     plt.show()
    
    
    ################################################
    ################# Gearbox Ratio ################
    ################################################
    range = np.linspace(50,100,11)
    result=[]
    for r_gb in range:
        res = driver(b=3, d=126.0, tsr=7.6, tf=1.0, cm=1.0, p_rated=5000., eta_dt=0.95, r_gb=r_gb)
        result.append(res)
            
    result = pd.DataFrame(result)
    
    plt.plot(result['r_gb'], np.array(result['nacelle_mass'])/nacelle_mass, linestyle='-', marker='^', label='Nacelle mass')
    plt.plot(result['r_gb'], np.array(result['nacelle.gearbox_mass'])/gb_mass, linestyle='-', marker='s',  label='Gearbox mass')
    plt.legend()
    plt.show()    
            
        
        







    
            
        
   
    
if __name__ == "__main__":
    sensitivity()