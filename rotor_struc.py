from math import pi
from rainflow import rainflow
import numpy as np
import pandas as pd

from openmdao.api import ExplicitComponent, Group, IndepVarComp
num_nodes = 6




class RotorUltimateStress(ExplicitComponent):
    def setup(self):
        # variables
        
        for i in range(2, num_nodes+1):
            var_name = 'B1N' + str(i) + 'Fx'
            self.add_input(var_name, units = 'N/m', desc = 'time series of force per unit length normal to the plane', shape=20)
            
        self.add_input('BlSpn', units = 'm', desc = 'list of blade node radial location', shape=num_nodes)
        self.add_input('root_chord', units = 'm', desc = 'chord length at blade root')
        self.add_input('root_thickness', units = 'm', desc = 'skin thickness at blade root')
     
        # outputs
        self.add_output('flap_stress_max', units = 'Pa', desc='maxium flapwise stress at the blade root')
 
 
    def compute(self, inputs, outputs):
         
#         for i in range(2, num_nodes+1):
#             var_name = 'B1N' + str(i) + 'Fx'
#             command = var_name + "= inputs['" + var_name + "']"
#             exec(command)
        
        B1N2Fx =   inputs['B1N2Fx']  
        B1N3Fx =   inputs['B1N3Fx']  
        B1N4Fx =   inputs['B1N4Fx']  
        B1N5Fx =   inputs['B1N5Fx']  
        B1N6Fx =   inputs['B1N6Fx']  
        
        BlSpn =   inputs['BlSpn']  
        root_chord = inputs['root_chord']
        root_thickness = inputs['root_thickness']
        
        r_root = BlSpn[1]        
        diameter_out = root_chord
        diameter_in = diameter_out - 2.0*root_thickness
        I_b = (pi/64.0) * (diameter_out**4 - diameter_in**4)
        
        root_stress = [] # time series of flapwise stress at the blade root
        
        for t in range(len(B1N2Fx)):
            moment_t = [] # flapwise moment at the blade root at a given time step
            
            for i in range(1, len(BlSpn)):
                var_name = 'B1N' + str(i+1) + 'Fx'
                moment_t.append(eval(var_name)[t] * (BlSpn[i] - r_root))
            
            # moment at blade root at time = t 
            root_moment = np.trapz(moment_t, BlSpn[1:])
            
            # stress at blade root at time = t
            root_stress.append(root_moment * (diameter_out/2) / I_b)
        
        outputs['flap_stress_max'] = max(root_stress)
        
        
        
        
class RotorFatigue(ExplicitComponent):
    def setup(self):
        # variables
        
        for i in range(2, num_nodes+1):
            var_name = 'B1N' + str(i) + 'Fx'
            self.add_input(var_name, units = 'N/m', desc = 'time series of force per unit length normal to the plane', shape=20)
            
        self.add_input('BlSpn', units = 'm', desc = 'list of blade node radial location', shape=num_nodes)
        self.add_input('root_chord', units = 'm', desc = 'chord length at blade root')
        self.add_input('root_thickness', units = 'm', desc = 'skin thickness at blade root')
        self.add_input('lifetime', units = 'year', desc = 'lifetime of the wind turbine')
     
        # outputs
        self.add_output('miners_damage', desc='miners fatigue damage')
 
 
    def compute(self, inputs, outputs):
         
        
        B1N2Fx =   inputs['B1N2Fx']  
        B1N3Fx =   inputs['B1N3Fx']  
        B1N4Fx =   inputs['B1N4Fx']  
        B1N5Fx =   inputs['B1N5Fx']  
        B1N6Fx =   inputs['B1N6Fx']  
        
        BlSpn =   inputs['BlSpn']  
        root_chord = inputs['root_chord']
        root_thickness = inputs['root_thickness']
        lifetime = inputs['lifetime']
        
        r_root = BlSpn[1]        
        diameter_out = root_chord
        diameter_in = diameter_out - 2.0*root_thickness
        I_b = (pi/64.0) * (diameter_out**4 - diameter_in**4)
        
        root_stress = [] # time series of flapwise stress at the blade root
        
        for t in range(len(B1N2Fx)):
            moment_t = [] # flapwise moment at the blade root at a given time step
            
            for i in range(1, len(BlSpn)):
                var_name = 'B1N' + str(i+1) + 'Fx'
                moment_t.append(eval(var_name)[t] * (BlSpn[i] - r_root))
            
            # moment at blade root at time = t 
            root_moment = np.trapz(moment_t, BlSpn[1:])
            
            # stress at blade root at time = t
            root_stress.append(root_moment * (diameter_out/2) / I_b)
        
        res = rainflow(np.array(root_stress))
        h = np.histogram(res[0], bins=10)
        
        load_range = [(a+b)/2.0 for a,b in zip(h[1][:-1], h[1][1:])]
        n = [x*360*24*365*lifetime for x in h[0]]
        N = [(900.0e6/x)**3.0 for x in load_range]
        
        damage_list = [float(a/b) for a,b in zip(n, N)] 
        outputs['miners_damage'] = sum(damage_list)
        
        



class RotorTipDeflection(ExplicitComponent):
    def setup(self):
        # variables
        
        for i in range(2, num_nodes+1):
            var_name = 'B1N' + str(i) + 'Fx'
            self.add_input(var_name, units = 'N/m', desc = 'time series of force per unit length normal to the plane', shape=20)
            
        self.add_input('BlSpn', units = 'm', desc = 'list of blade node radial location', shape=num_nodes)
        self.add_input('BlInertia', units = 'm**4', desc = 'list of blade node area moment of inertia', shape=num_nodes)
     
        # outputs
        self.add_output('tip_deflection', units = 'm', desc='tip deflection from netrual point')
 
 
    def compute(self, inputs, outputs):
         
        
        B1N2Fx =   inputs['B1N2Fx']  
        B1N3Fx =   inputs['B1N3Fx']  
        B1N4Fx =   inputs['B1N4Fx']  
        B1N5Fx =   inputs['B1N5Fx']  
        B1N6Fx =   inputs['B1N6Fx']  
        
        BlSpn =   inputs['BlSpn']  
        BlInertia = inputs['BlInertia']
        
        
        for i in range(len(BlSpn)):
            root_stress = [] # time series of flapwise stress at the blade root
        
        for t in range(len(B1N2Fx)):
            
            
            for node in range(2, len(BlSpn)+1):
                var_name = 'B1N' + str(node) + 'Fx'
                moment_t = [] # flapwise moment at that at a given time step
                
                for j in range(node, len(BlSpn)+1):
                    var_name = 'B1N' + str(j) + 'Fx'
                    moment_t.append(eval(var_name)[t] * (BlSpn[j] - BlSpn[node]))
            
                # moment at the given node at time = t 
                root_moment = np.trapz(moment_t, BlSpn[node-1:])
            
            
        

        outputs['tip_deflection'] = sum(damage_list)




def test_damage():
    B1N2Fx =   [1481.973, 1491.605, 1492.307, 1483.669, 1473.638, 1471.138, 1478.922, 1489.682, 1493.135, 1486.955, 1476.132, 1470.594, 1476.231, 1487.067, 1493.15, 1489.598, 1478.828, 1471.098, 1473.722, 1483.812]
    B1N3Fx =   [3045.719, 3086.007, 3089.144, 3052.499, 3011.242, 3002.288, 3032.618, 3077.384, 3092.834, 3065.605, 3020.642, 3000.433, 3021.033, 3066.064, 3092.902, 3077.008, 3032.156, 3002.156, 3011.546, 3053.0]  
    B1N4Fx =   [4719.358, 4781.151, 4786.083, 4729.77, 4663.076, 4647.987, 4698.679, 4767.713, 4791.899, 4749.696, 4678.917, 4644.906, 4679.574, 4750.396, 4792.007, 4767.133, 4697.933, 4647.769, 4663.588, 4730.536]
    B1N5Fx =   [6435.854, 6514.452, 6520.774, 6449.026, 6365.034, 6346.128, 6409.771, 6497.263, 6528.244, 6474.302, 6384.919, 6342.269, 6385.74, 6475.193, 6528.382, 6496.522, 6408.832, 6345.854, 6365.679, 6449.991] 
    B1N6Fx =   [6949.423, 7048.738, 7056.761, 6966.001, 6860.686, 6837.125, 6916.653, 7026.944, 7066.248, 6997.901, 6885.523, 6832.323, 6886.552, 6999.025, 7066.424, 7026.007, 6915.476, 6836.784, 6861.492, 6967.224] 
    
    BlSpn =   [0.0, 10.0, 20.0, 30.0, 40.0, 50.0] 
    root_chord = 3.4
    root_thickness = 0.1
    
    r_root = BlSpn[1]        
    diameter_out = root_chord
    diameter_in = diameter_out - 2.0*root_thickness
    I_b = (pi/64.0) * (diameter_out**4 - diameter_in**4)
    
    root_stress = [] # time series of flapwise stress at the blade root
    
    for t in range(len(B1N2Fx)):
        moment_t = [] # flapwise moment at the blade root at a given time step
        
        for i in range(1, len(BlSpn)):
            var_name = 'B1N' + str(i+1) + 'Fx'
            moment_t.append(eval(var_name)[t] * (BlSpn[i] - r_root))
        
        # moment at blade root at time = t 
        root_moment = np.trapz(moment_t, BlSpn[1:])
        
        # stress at blade root at time = t
        root_stress.append(root_moment * (diameter_out/2) / I_b)
    
#     print root_stress
#     print max(root_stress)
#     
#     print B1N2Fx
#     fx = [B1N2Fx, B1N3Fx, B1N4Fx]
#     print len(fx)

    res = rainflow(np.array(root_stress))
    h = np.histogram(res[0], bins=10)
    
    load_range = [(a+b)/2.0 for a,b in zip(h[1][:-1], h[1][1:])]
    n = [x*360*24*365*20 for x in h[0]]
    N = [(900.0e6/x)**3.0 for x in load_range]
    
    damage_list = [a/b for a,b in zip(n, N)]
    
    print I_b
    print n
    print N
    print sum(damage_list)
    
        
        
        

if __name__ == "__main__":
    B1N2Fx =   [1481.973, 1491.605, 1492.307, 1483.669, 1473.638, 1471.138, 1478.922, 1489.682, 1493.135, 1486.955, 1476.132, 1470.594, 1476.231, 1487.067, 1493.15, 1489.598, 1478.828, 1471.098, 1473.722, 1483.812]
    B1N3Fx =   [3045.719, 3086.007, 3089.144, 3052.499, 3011.242, 3002.288, 3032.618, 3077.384, 3092.834, 3065.605, 3020.642, 3000.433, 3021.033, 3066.064, 3092.902, 3077.008, 3032.156, 3002.156, 3011.546, 3053.0]  
    B1N4Fx =   [4719.358, 4781.151, 4786.083, 4729.77, 4663.076, 4647.987, 4698.679, 4767.713, 4791.899, 4749.696, 4678.917, 4644.906, 4679.574, 4750.396, 4792.007, 4767.133, 4697.933, 4647.769, 4663.588, 4730.536]
    B1N5Fx =   [6435.854, 6514.452, 6520.774, 6449.026, 6365.034, 6346.128, 6409.771, 6497.263, 6528.244, 6474.302, 6384.919, 6342.269, 6385.74, 6475.193, 6528.382, 6496.522, 6408.832, 6345.854, 6365.679, 6449.991] 
    B1N6Fx =   [6949.423, 7048.738, 7056.761, 6966.001, 6860.686, 6837.125, 6916.653, 7026.944, 7066.248, 6997.901, 6885.523, 6832.323, 6886.552, 6999.025, 7066.424, 7026.007, 6915.476, 6836.784, 6861.492, 6967.224] 
    
    BlSpn =   [0.0, 10.0, 20.0, 30.0, 40.0, 50.0] 
    BlInertia = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
    
    num_nodes = len(BlSpn)
    ones = [1.0] * num_nodes
    
    
    nodes = pd.DataFrame({'x'  : BlSpn}, \
                       {'Fx' : ones}, \
                       {'Mx' : ones}, \
                       {'Ix' : BlInertia}, \
                       {'M_EI' : ones})
    
    print nodes
    
#     for t in range(len(B1N2Fx)):
#         
#         
#         for node in range(2, len(BlSpn)+1):
#             var_name = 'B1N' + str(node) + 'Fx'
#             moment_t = [] # flapwise moment at that at a given time step
#             
#             for j in range(node, len(BlSpn)+1):
#                 var_name = 'B1N' + str(j) + 'Fx'
#                 moment_t.append(eval(var_name)[t] * (BlSpn[j] - BlSpn[node]))
#         
#             # moment at the given node at time = t 
#             root_moment = np.trapz(moment_t, BlSpn[node-1:])
#         
#         
#     
# 
#     outputs['tip_deflection'] = sum(damage_list)

        
    
    
    
    
            