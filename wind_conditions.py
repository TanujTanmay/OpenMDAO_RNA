from math import log

from openmdao.api import ExplicitComponent
from fixed_parameters import z0, h_ref, wind_shear

class HubWindSpeed(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('pot_wind_speed', units = 'm/s', desc = 'potential wind speed at reference height of 10m')
        self.add_input('hub_height', units = 'm', desc = 'hub height')
        
        self.add_output('wind_speed', units='m/s', desc='wind speed at hub height')
        
    def compute(self, inputs, outputs):
         
        pot_wind_speed = inputs['pot_wind_speed']
        hub_height = inputs['hub_height']
        
        # reference height to 60m (log law)
        u_60 = pot_wind_speed * log(60/z0) / log(h_ref/z0)
            
        # 60m to hub height (exponential law)
        outputs['wind_speed'] = u_60 * (hub_height/60) ** wind_shear  