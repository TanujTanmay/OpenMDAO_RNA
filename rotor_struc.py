from math import pi
from rainflow import rainflow
import numpy as np

from openmdao.api import ExplicitComponent, Group, IndepVarComp

class RotorUltimateStress(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('blade_radius', units = 'm', desc = 'radius of the blade')
        self.add_input('r_root', units = 'm', desc = 'radial position of the blade root')
        self.add_input('cd_max', desc='maximum darg coefficient at blade root')
        self.add_input('chord_length', units = 'm', desc = 'chord length at blade root')
        self.add_input('thickness', units = 'm', desc = 'skin thickness at blade root')
        self.add_input('u_rated', units = 'm/s', desc = 'rated wind speed')
     
        # outputs
        self.add_output('flap_stress_max', units = 'Pa', desc='maxium flapwise stress at the blade root')
 
 
    def compute(self, inputs, outputs):
         
        blade_radius = inputs['blade_radius']
        r_root = inputs['r_root']
        cd_max = inputs['cd_max']
        chord_length = inputs['chord_length']
        thickness = inputs['thickness']
        u_rated = inputs['u_rated']
        
        flap_moment = 0.5 * 1.225 * cd_max * (u_rated**2) * chord_length * ((0.5 * blade_radius**2) - \
                                                                            (blade_radius * r_root) + \
                                                                            (0.5 * r_root**2))
        
        diameter_out = chord_length
        diameter_in = diameter_out - 2.0*thickness
        I_b = (pi/64.0) * (diameter_out**4 - diameter_in**4)
        
        outputs['flap_stress_max'] = flap_moment * (diameter_out/2) / I_b
        
        
        
        
class RotorFatigue(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('blade_radius', units = 'm', desc = 'radius of the blade')
        self.add_input('r_root', units = 'm', desc = 'radial position of the blade root')
        self.add_input('cd_root', desc = 'time series of drag coefficient at blade root')
        self.add_input('chord_length', units = 'm', desc = 'chord length at blade root')
        self.add_input('thickness', units = 'm', desc = 'skin thickness at blade root')
        self.add_input('u_rated', units = 'm/s', desc = 'rated wind speed')
     
        # outputs
        self.add_output('miners_damage', desc='miners damage')
 
 
    def compute(self, inputs, outputs):
         
        blade_radius = inputs['blade_radius']
        r_root = inputs['r_root']
        cd_root = inputs['cd_root']
        chord_length = inputs['chord_length']
        thickness = inputs['thickness']
        u_rated = inputs['u_rated']
        
        diameter_out = chord_length
        diameter_in = diameter_out - 2.0*thickness
        I_b = (pi/64.0) * (diameter_out**4 - diameter_in**4)
        
        stress_factor = 0.5 * 1.225 * (u_rated**2) * chord_length * ((0.5 * blade_radius**2) - \
                                                                            (blade_radius * r_root) + \
                                                                            (0.5 * r_root**2)) \
                        * (diameter_out/2) / I_b                                                    
        
        flap_stress_timeseries = [stress_factor * x for x in cd_root]
        
        
        
        outputs['flap_stress_max'] = flap_moment * r_root/I_b       
        
        
        
        

if __name__ == "__main__":
    blade_radius = 50.0
    r_root = 10.0
    cd_root = [0.0108,    0.0105,    0.0105,    0.01074,    0.01088,    0.01094,    0.0108,    0.01052,    0.0105,    0.0106,    0.01082,    0.01096,    0.01082,    0.01059,    0.0105,    0.01052,    0.0108,    0.01095,    0.01088,    0.01074]
    chord_length = 3.4
    thickness = 0.1
    u_rated = 10.0
    
    diameter_out = chord_length
    diameter_in = diameter_out - 2.0*thickness
    I_b = (pi/64.0) * (diameter_out**4 - diameter_in**4)
    
    stress_factor = 0.5 * 1.225 * (u_rated**2) * chord_length * ((0.5 * blade_radius**2) - \
                                                                        (blade_radius * r_root) + \
                                                                        (0.5 * r_root**2)) \
                    * (diameter_out/2) / I_b                                                    
    
    flap_stress_timeseries = [stress_factor * x for x in cd_root]
    
    print rainflow(np.array(flap_stress_timeseries))
    
    
    
    
            