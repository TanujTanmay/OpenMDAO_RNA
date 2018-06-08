from time import time
from math import pi
import numpy as np
import matplotlib.pyplot as plt

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model    
from fixed_parameters import num_bins, beautify, plot_graphs, plots_folder, rho_air  




#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class PowerCurve(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('design_tsr', desc='design tip speed ratio')
        self.add_input('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        self.add_input('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        self.add_input('swept_area', units='m**2', desc='rotor swept area')
        self.add_input('machine_rating', units='kW', desc='machine rating')
        self.add_input('drive_train_efficiency', desc='efficiency of aerodynamic to electrical conversion')
        self.add_input('rotor_cp', desc='rotor power coefficient')
        self.add_input('rotor_ct', desc='rotor thrust coefficient')
        
        # outputs
        self.add_output('rated_wind_speed', units = 'm/s', desc = 'rated wind speed')
        self.add_output('rated_tip_speed', units = 'm/s', desc = 'rated tip speed')
        self.add_output('wind_bin', units = 'm/s', desc='list of wind speeds', shape=num_bins)
        self.add_output('elec_power_bin', units='kW', desc='list of electrical power output', shape=num_bins)
        self.add_output('aero_power_bin', units='kW', desc='list of aerodynamic power output', shape=num_bins)
        self.add_output('thrust_bin', units = 'N', desc='list of rotor thrust', shape=num_bins)
        self.add_output('cp_bin', desc='list of power coefficients', shape=num_bins)
        self.add_output('ct_bin', desc='list of thrust coefficients', shape=num_bins)
        
        
    def compute(self, inputs, outputs):
        # inputs
        design_tsr = inputs['design_tsr'] 
        cut_in_speed = inputs['cut_in_speed'] 
        cut_out_speed = inputs['cut_out_speed']  
        swept_area = inputs['swept_area']
        machine_rating = inputs['machine_rating'] 
        eta_dt = inputs['drive_train_efficiency']
        rotor_cp = inputs['rotor_cp']   
        rotor_ct = inputs['rotor_ct']  
        
        rated_wind_speed = (machine_rating * 1000.0 / (rotor_cp * 0.5 * rho_air * swept_area * eta_dt))**(1.0/3.0)
        rated_tip_speed  = design_tsr * rated_wind_speed
        wind_bin = np.linspace(0, 30.0, num_bins) 
        aero_power_bin, elec_power_bin, thrust_bin, cp_bin, ct_bin = [], [], [], [], []
       
        # aerodynamic calculations
        for v in wind_bin:
            if v < cut_in_speed or v > cut_out_speed:
                power, thrust, cp, ct = 0., 0., 0., 0.
            elif v < rated_wind_speed:
                power  = rotor_cp * 0.5 * rho_air * swept_area * (v**3) / 1000.0  
                thrust = rotor_ct * 0.5 * rho_air * swept_area * (v**2)   
                cp = rotor_cp
                ct = rotor_ct           
            else:
                power = machine_rating/eta_dt    
                cp = (machine_rating * 1000.0)/(0.5 * rho_air * swept_area * (v**3) * eta_dt)
                
                # => cp = 4a(1-a)^2 
                # => 4a^3 - 8a^2 + 4a - cp = 0
                a = min(np.roots([4, -8, 4, -1*cp]))
                ct = 4*a*(1-a)
                thrust = ct * 0.5 * rho_air * swept_area * (rated_wind_speed**2)
            
            aero_power_bin.append(power)        # aerodynamic power
            elec_power_bin.append(power*eta_dt) # electrical power    
            thrust_bin.append(thrust)
            cp_bin.append(cp)
            ct_bin.append(ct)
        
        if plot_graphs:
            plots = {'Electrical Power (kW)'  : elec_power_bin, \
                     'Aerodynamic Power (kW)' : aero_power_bin, \
                     'Thrust (N)' : thrust_bin, \
                     'C_p (-)' : cp_bin, \
                     'C_t (-)' : ct_bin}
            for key, value in plots.items():
                plt.close('all')
                plt.plot(wind_bin, value)
                plt.xlabel('Wind Speed (m/s)')
                plt.ylabel(key)
                plt.show()
                #plt.savefig(plots_folder + key + '.png')

        
        outputs['rated_wind_speed'] = rated_wind_speed     
        outputs['rated_tip_speed'] = rated_tip_speed   
        outputs['wind_bin'] = wind_bin
        outputs['elec_power_bin'] = np.array(elec_power_bin).reshape(num_bins)
        outputs['aero_power_bin'] = aero_power_bin
        outputs['thrust_bin'] = thrust_bin    
        outputs['cp_bin'] = cp_bin
        outputs['ct_bin'] = ct_bin   
        
        
        
        
        
        
#############################################################################
##############################  UNIT TESTING ################################
#############################################################################         
class PowerCurveTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('design_tsr', desc='design tip speed ratio')
        i.add_output('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        i.add_output('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        i.add_output('swept_area', units='m**2', desc='rotor swept area')
        i.add_output('machine_rating', units='kW', desc='machine rating')
        i.add_output('drive_train_efficiency', desc='efficiency of aerodynamic to electrical conversion')
        i.add_output('rotor_cp', desc='rotor power coefficient')
        i.add_output('rotor_ct', desc='rotor thrust coefficient')
        
        # sub-components         
        self.add_subsystem('dof', i, promotes=['*'])   
        self.add_subsystem('pc', PowerCurve(), promotes_inputs=['*'])
        
        
        


if __name__ == "__main__":
    start = time()
    
    # workflow setup
    prob = Problem(PowerCurveTest())
    prob.setup()
    view_model(prob, outfile='N2/power_curve.html')
    
    # define inputs
    prob['dof.design_tsr'] = 7.0
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.swept_area'] = 12445.26
    prob['dof.machine_rating'] = 5000.0
    prob['dof.drive_train_efficiency'] = 0.95
    prob['dof.rotor_cp'] = 0.467403
    prob['dof.rotor_ct'] = 0.7410045
    
     
    prob.run_model()
    
    # print outputs 
    print "Rotor Power Curve"
    print 'rated_wind_speed = ' + beautify(prob['pc.rated_wind_speed'])
    print 'wind_bin = ' + beautify(prob['pc.wind_bin'])
    print 'elec_power_bin = ' + beautify(prob['pc.elec_power_bin'])
    print 'aero_power_bin = ' + beautify(prob['pc.aero_power_bin'])
    print 'thrust_bin = ' + beautify(prob['pc.thrust_bin'])
    print 'cp_bin = ' + beautify(prob['pc.cp_bin'])
    print 'ct_bin = ' + beautify(prob['pc.ct_bin'])
  
    print 'Done in ' + str(time() - start) + ' seconds'    
     
    


         
