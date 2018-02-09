from math import pi
from rainflow import rainflow
import numpy as np
import pandas as pd
from time import time, clock
import datetime as dt

from openmdao.api import ExplicitComponent, Group, IndepVarComp

num_nodes = 6
num_times = 20
T_max = 10 # time of simulation [s]

E = 200.0e9 # Young's modulus of steel [Pa]
UTS = 900.0e6 # ultimate tensile strength of steel [Pa]
SN = 9.0 # slope (inverse negative) of SN Curve


class RotorUltimateStress(ExplicitComponent):
    def setup(self):
        # variables
        
        for node_index in range(num_nodes): # loop through each node_index, call NODE_index
            node_number = node_index + 1
            var_name = 'B1N' + str(node_number) + 'Fx'
            self.add_input(var_name, units = 'N/m', desc = 'time series of force per unit length normal to the plane', shape=num_times)
            
        self.add_input('BlSpn', units = 'm', desc = 'list of blade node radial location', shape=num_nodes)
        self.add_input('root_area_moment', units = 'm**4', desc = 'second moment of area at blade root')
        self.add_input('root_chord', units = 'm', desc = 'chord length at blade root')
        
     
        # outputs
        self.add_output('root_stress_max', units = 'Pa', desc='maximum stress at the blade root')
 
 
    def compute(self, inputs, outputs):
         
#         for i in range(2, num_nodes+1):
#             var_name = 'B1N' + str(i) + 'Fx'
#             command = var_name + "= inputs['" + var_name + "']"
#             exec(command)
        
        B1N1Fx =   inputs['B1N1Fx']
        B1N2Fx =   inputs['B1N2Fx']  
        B1N3Fx =   inputs['B1N3Fx']  
        B1N4Fx =   inputs['B1N4Fx']  
        B1N5Fx =   inputs['B1N5Fx']  
        B1N6Fx =   inputs['B1N6Fx']  
        
        BlSpn =   inputs['BlSpn']  
        root_chord = inputs['root_chord']
        root_area_moment = inputs['root_area_moment']
        
        diameter = root_chord
        root_stress_max = 0    

        for t in range(len(B1N2Fx)): # loop through each time step
            d_moment = [] # list of flapwise moment due to Fx at every node at the blade root at the gien time step
            
            for far_node_index in range(len(BlSpn)): 
                far_node_number = far_node_index + 1
                var_name = 'B1N' + str(far_node_number) + 'Fx'
                d_moment.append(eval(var_name)[t] * (BlSpn[far_node_index] - BlSpn[0]))
            
            # moment at blade root at time = t 
            root_moment = np.trapz(d_moment, BlSpn)
            
            # stress at blade root at time = t 
            root_stress = root_moment * (diameter/2) / root_area_moment
            
            root_stress_max = root_stress if root_stress > root_stress_max else root_stress_max
            
            
        
        outputs['root_stress_max'] = root_stress_max
        
        
        
        
class RotorFatigue(ExplicitComponent):
    def setup(self):
        
        # variables
        for node_index in range(num_nodes): # loop through each node_index, call NODE_index
            node_number = node_index + 1
            var_name = 'B1N' + str(node_number) + 'Fx'
            self.add_input(var_name, units = 'N/m', desc = 'time series of force per unit length normal to the plane', shape=num_times)
            
        self.add_input('BlSpn', units = 'm', desc = 'list of blade node radial location', shape=num_nodes)
        self.add_input('root_area_moment', units = 'm**4', desc = 'second moment of area at blade root')
        self.add_input('root_chord', units = 'm', desc = 'chord length at blade root')
        self.add_input('lifetime', units = 'year', desc = 'lifetime of the wind turbine')
     
        # outputs
        self.add_output('miners_damage', desc='miners fatigue damage')
 
 
    def compute(self, inputs, outputs):
         
        B1N1Fx =   inputs['B1N1Fx'] 
        B1N2Fx =   inputs['B1N2Fx']  
        B1N3Fx =   inputs['B1N3Fx']  
        B1N4Fx =   inputs['B1N4Fx']  
        B1N5Fx =   inputs['B1N5Fx']  
        B1N6Fx =   inputs['B1N6Fx']  
        
        BlSpn =   inputs['BlSpn']  
        root_chord = inputs['root_chord']
        root_area_moment = inputs['root_area_moment']
        lifetime = inputs['lifetime']
        
        diameter = root_chord        
        root_stress = [] # time series of flapwise stress at the blade root
        
        for t in range(len(B1N2Fx)): # loop through each time step
            d_moment = [] # list of flapwise moment due to Fx at every node at the blade root at the gien time step
            
            for far_node_index in range(len(BlSpn)): 
                far_node_number = far_node_index + 1
                var_name = 'B1N' + str(far_node_number) + 'Fx'
                d_moment.append(eval(var_name)[t] * (BlSpn[far_node_index] - BlSpn[0]))
            
            # moment at blade root at time = t 
            root_moment = np.trapz(d_moment, BlSpn)
            
            # append the stress at blade root at time = t to the time series
            root_stress.append(root_moment * (diameter/2) / root_area_moment)
        
        res = rainflow(np.array(root_stress))
        h = np.histogram(res[0], bins=10)
        
        load_range = [(a+b)/2.0 for a,b in zip(h[1][:-1], h[1][1:])]
        n = [x*(3600.0/T_max)*24*365*lifetime for x in h[0]]
        N = [(UTS/x)**SN for x in load_range]
        
        damage_list = [float(a/b) for a,b in zip(n, N)] 
        outputs['miners_damage'] = sum(damage_list)
        
        



class RotorTipDeflection(ExplicitComponent):
    def setup(self):
        # variables
        for node_index in range(num_nodes): # loop through each node_index, call NODE_index
            node_number = node_index + 1
            var_name = 'B1N' + str(node_number) + 'Fx'
            self.add_input(var_name, units = 'N/m', desc = 'time series of force per unit length normal to the plane', shape=num_times)
            
        self.add_input('BlSpn', units = 'm', desc = 'list of blade node radial location', shape=num_nodes)
        self.add_input('BlInertia', units = 'm**4', desc = 'list of blade node area moment of inertia', shape=num_nodes)
     
        # outputs
        self.add_output('tip_deflection', units = 'm', desc='maximum tip deflection from netrual point')
 
 
    def compute(self, inputs, outputs):
         
        B1N1Fx =   inputs['B1N1Fx']  
        B1N2Fx =   inputs['B1N2Fx']  
        B1N3Fx =   inputs['B1N3Fx']  
        B1N4Fx =   inputs['B1N4Fx']  
        B1N5Fx =   inputs['B1N5Fx']  
        B1N6Fx =   inputs['B1N6Fx']  
        
        BlSpn =   inputs['BlSpn']  
        BlInertia = inputs['BlInertia']
        
        
        num_nodes = len(BlSpn)
        zeroes = [0.0] * num_nodes
        
        nodal_param = pd.DataFrame(data={  'x' : BlSpn, \
                                           'Fx' : zeroes, \
                                           'Mx' : zeroes, \
                                           'Ix' : BlInertia, \
                                           'M_EI' : zeroes, \
                                           'dy_dx': zeroes, \
                                           'y': zeroes
                                        } 
                                    )
        
        max_tip_deflection = 0
        
        for t in range(len(B1N2Fx)): # loop through each time step
               
               
            for node_index in range(len(BlSpn)): # loop through each node_index, call NODE_index
                node_number = node_index + 1
                #print 'Node Number: ' + str(node_number)
                nodal_param.loc[node_index, 'Fx'] = eval('B1N' + str(node_number) + 'Fx')[t]
                d_moment = []
                   
                for far_node_index in range(node_index, len(BlSpn)): # j is the node_index larger than NODE_index
                    far_node_number = far_node_index + 1
                    #print '\t Moments at Nodes: ' + str(far_node_number)
                    d_moment.append(eval('B1N' + str(far_node_number) + 'Fx')[t] * (BlSpn[far_node_index] - BlSpn[node_index]))
               
                # moment at the given node at time = t 
                #print '\t Integrating d_moment: ' + ', '.join(str(x) for x in BlSpn[node_index:])
                nodal_param.loc[node_index, 'Mx'] = np.trapz(d_moment, BlSpn[node_index:])
                nodal_param.loc[node_index, 'M_EI'] = nodal_param.loc[node_index, 'Mx'] / (E * nodal_param.loc[node_index, 'Ix'])
                
                d2y_dx2 = []
                for near_node_index in range(node_index+1): # j is the node_index smaller than NODE_index
                    near_node_number = near_node_index + 1
                    #print '\t d2y_dx2 at Nodes: ' + str(near_node_number)
                    d2y_dx2.append(nodal_param.loc[near_node_index, 'M_EI'])
                
                # dy_dx at the given node at time = t     
                #print '\t Integrating d2y_dx2: ' + ', '.join(str(x) for x in BlSpn[:node_index+1])    
                nodal_param.loc[node_index, 'dy_dx'] = np.trapz(d2y_dx2, BlSpn[:node_index+1])    
                
                
                dy_dx = []
                for near_node_index in range(node_index+1): # j is the node_index smaller than NODE_index
                    near_node_number = near_node_index + 1
                    #print '\t dy_dx at Nodes: ' + str(near_node_number)
                    dy_dx.append(nodal_param.loc[near_node_index, 'dy_dx'])
                
                # dy_dx at the given node at time = t     
                #print '\t Integrating dy_dx: ' + ', '.join(str(x) for x in BlSpn[:node_index+1])    
                nodal_param.loc[node_index, 'y'] = np.trapz(dy_dx, BlSpn[:node_index+1])            
                
                
            tip_deflection = nodal_param.loc[len(BlSpn)-1, 'y']               
            max_tip_deflection = tip_deflection if  tip_deflection > max_tip_deflection else max_tip_deflection
                  
       
        outputs['tip_deflection'] = max_tip_deflection




class RotorStructure(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        for node_index in range(num_nodes): # loop through each node_index, call NODE_index
            node_number = node_index + 1
            var_name = 'B1N' + str(node_number) + 'Fx'
            i.add_output(var_name, units = 'N/m', desc = 'time series of force per unit length normal to the plane', shape=num_times)
            
        i.add_output('BlSpn', units = 'm', desc = 'list of blade node radial location', shape=num_nodes)  
        i.add_output('BlInertia', units = 'm**4', desc = 'list of blade node area moment of inertia', shape=num_nodes) 
        i.add_output('root_chord', units = 'm', desc = 'chord length at blade root')
        i.add_output('lifetime', units = 'year', desc = 'lifetime of the wind turbine')
        
        
        # systems
        self.add_subsystem('dof', i) 
        self.add_subsystem('ultimate', RotorUltimateStress()) 
        self.add_subsystem('fatigue', RotorFatigue()) 
        self.add_subsystem('deflection', RotorTipDeflection()) 
        
        # connections
        for node_index in range(num_nodes): # loop through each node_index, call NODE_index
            node_number = node_index + 1
            var_name = 'B1N' + str(node_number) + 'Fx'
            self.connect('dof.' + var_name, ['ultimate.' + var_name, 'fatigue.' + var_name, 'deflection.' + var_name])
        
        self.connect('dof.BlSpn', ['ultimate.BlSpn', 'fatigue.BlSpn', 'deflection.BlSpn'])
        self.connect('dof.BlInertia', 'deflection.BlInertia')
        self.connect('dof.BlInertia', ['ultimate.root_area_moment', 'fatigue.root_area_moment'], src_indices=[0])
        self.connect('dof.root_chord', ['ultimate.root_chord', 'fatigue.root_chord'])
        self.connect('dof.lifetime', 'fatigue.lifetime')
        
        

if __name__ == "__main__":
    B1N1Fx =   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    B1N2Fx =   [1481.973, 1491.605, 1492.307, 1483.669, 1473.638, 1471.138, 1478.922, 1489.682, 1493.135, 1486.955, 1476.132, 1470.594, 1476.231, 1487.067, 1493.15, 1489.598, 1478.828, 1471.098, 1473.722, 1483.812]
    B1N3Fx =   [3045.719, 3086.007, 3089.144, 3052.499, 3011.242, 3002.288, 3032.618, 3077.384, 3092.834, 3065.605, 3020.642, 3000.433, 3021.033, 3066.064, 3092.902, 3077.008, 3032.156, 3002.156, 3011.546, 3053.0]  
    B1N4Fx =   [4719.358, 4781.151, 4786.083, 4729.77, 4663.076, 4647.987, 4698.679, 4767.713, 4791.899, 4749.696, 4678.917, 4644.906, 4679.574, 4750.396, 4792.007, 4767.133, 4697.933, 4647.769, 4663.588, 4730.536]
    B1N5Fx =   [6435.854, 6514.452, 6520.774, 6449.026, 6365.034, 6346.128, 6409.771, 6497.263, 6528.244, 6474.302, 6384.919, 6342.269, 6385.74, 6475.193, 6528.382, 6496.522, 6408.832, 6345.854, 6365.679, 6449.991] 
    B1N6Fx =   [6949.423, 7048.738, 7056.761, 6966.001, 6860.686, 6837.125, 6916.653, 7026.944, 7066.248, 6997.901, 6885.523, 6832.323, 6886.552, 6999.025, 7066.424, 7026.007, 6915.476, 6836.784, 6861.492, 6967.224] 
    
    BlSpn =   [0.0, 10.0, 20.0, 30.0, 40.0, 50.0] 
    BlInertia = [0.10, 0.01, 0.01, 0.01, 0.01, 0.01]
    root_chord = 3.4
    lifetime = 20
    
    from openmdao.api import Problem, view_model
     
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RotorStructure())
    prob.setup()
    view_model(prob, outfile='rotor_struc.html')
    
    prob['dof.B1N1Fx'] = B1N1Fx
    prob['dof.B1N2Fx'] = B1N2Fx
    prob['dof.B1N3Fx'] = B1N3Fx
    prob['dof.B1N4Fx'] = B1N4Fx
    prob['dof.B1N5Fx'] = B1N5Fx
    prob['dof.B1N6Fx'] = B1N6Fx
    prob['dof.BlSpn'] = BlSpn
    prob['dof.BlInertia'] = BlInertia
    prob['dof.root_chord'] = root_chord
    prob['dof.lifetime'] = lifetime
    
    prob.run_model()
     
    print "My Rotor"
    print prob['ultimate.root_stress_max'] 
    print prob['fatigue.miners_damage'] 
    print prob['deflection.tip_deflection'] 
    
    print 'Done in ' + str(time() - start) + 'seconds'
    
        
    
    
    
    
            