import numpy as np
from time import time
from math import pi
import matplotlib.pyplot as plt

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model
from fixed_parameters import num_nodes, num_bins, rho_air, beautify, plot_graphs
from bem import bem_rotor


class AnnulusAero(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('wind_speed', units = 'm/s', desc = 'wind speed')
        self.add_input('blade_number', desc='number of blades')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        self.add_input('hub_radius', units = 'm', desc = 'hub radius')
        self.add_input('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        self.add_input('span_dr', units='m', desc='list of blade annulus thickness', shape=num_nodes)
        self.add_input('span_airfoil', desc='list of blade node Airfoil ID', shape=num_nodes)
        self.add_input('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        self.add_input('span_twist', units='deg',desc='list of blade node twist angle', shape=num_nodes)
        self.add_input('pitch', units = 'deg', desc = 'blade pitch angle')
        
     
        # outputs
        self.add_output('span_power', units='W', desc='list of spanwise power', shape=num_nodes)
        self.add_output('span_thrust', units='N', desc='list of spanwise thrust', shape=num_nodes)
        self.add_output('span_fx', units='N/m', desc='list of spanwise normal force to the plane of rotation', shape=num_nodes)
        self.add_output('span_fy', units='N/m', desc='list of spanwise tangential force to the plane of rotation', shape=num_nodes)
        
 
 
    def compute(self, inputs, outputs):
        
        wind_speed = inputs['wind_speed'] 
        blade_number = inputs['blade_number']
        rotor_diameter = inputs['rotor_diameter']
        rotor_speed = inputs['rotor_speed']
        hub_radius = inputs['hub_radius']
        span_r = inputs['span_r']
        span_dr = inputs['span_dr']
        span_chord = inputs['span_chord']
        span_twist = inputs['span_twist']
        span_airfoil = inputs['span_airfoil']
        pitch = inputs['pitch']
        
        rotor_radius = rotor_diameter/2.0
        tsr = (rotor_speed * 2 * pi / 60.0) * rotor_radius/wind_speed
        
        result = bem_rotor(wind_speed, blade_number, rotor_radius, hub_radius, tsr, pitch, \
                span_r, span_dr, span_chord, span_twist, span_airfoil, is_prandtl=1, is_glauert=1)
        
        
        
        # Plot and save
        if plot_graphs:
            params = ['chord', 'twist', 'alpha', 'phi', 'aA', 'aT', 'cl', 'cd', 'cp', 'ct']
            for param in params:
                # save the plots
                plt.close('all')
                plt.plot(np.array(result['r']), np.array(result[param]))
                plt.xlabel('Span [m]')
                plt.ylabel(param)
                #plt.show()
                plt.savefig('Plots/' + param + '.png')
        
        
        outputs['span_power'] = np.array(result['power'])
        outputs['span_thrust'] = np.array(result['thrust'])
        outputs['span_fx'] = np.array(result['fx'])
        outputs['span_fy'] = np.array(result['fy'])
        
        


class RotorAero(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('wind_speed', units = 'm/s', desc = 'wind speed')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        self.add_input('hub_radius', units = 'm', desc = 'hub radius')
        self.add_input('span_power', units='W', desc='list of spanwise power', shape=num_nodes)
        self.add_input('span_thrust',  units='N', desc='list of spanwise thrust', shape=num_nodes)
        
        
        # outputs
        self.add_output('rotor_tsr', desc='rotor tip speed ratio')
        self.add_output('swept_area', units='m**2', desc='rotor swept area')
        self.add_output('rotor_cp', desc='rotor power coefficient')
        self.add_output('rotor_cq', desc='rotor torque coefficient')
        self.add_output('rotor_ct',  desc='rotor thrust coefficient')
        self.add_output('rotor_power', units='W', desc='rotor power')
        self.add_output('rotor_torque', units='N*m', desc='rotor torque')
        self.add_output('rotor_thrust',  units='N', desc='rotor thrust')
        
        
    def compute(self, inputs, outputs):
        
        wind_speed = inputs['wind_speed']
        rotor_diameter = inputs['rotor_diameter']
        rotor_speed = inputs['rotor_speed']
        hub_radius = inputs['hub_radius']
        span_power = inputs['span_power']
        span_thrust = inputs['span_thrust']
        
        rotor_radius = rotor_diameter/2.0
        swept_area = pi * (rotor_radius**2 - hub_radius**2)
        omega = rotor_speed * (2*pi/60.0)
        tsr = omega*rotor_radius/wind_speed
        
        power = np.sum(span_power)
        torque = power/omega
        thrust = np.sum(span_thrust)
        
        cp = power/(0.5 * rho_air * (wind_speed ** 3) * swept_area)
        cq = cp/tsr
        ct = thrust/(0.5 * rho_air * (wind_speed ** 2) * swept_area) 
        
        outputs['rotor_tsr'] = tsr
        outputs['swept_area'] = swept_area
        outputs['rotor_cp'] = cp
        outputs['rotor_cq'] = cq
        outputs['rotor_ct'] = ct
        outputs['rotor_power'] = power
        outputs['rotor_torque'] = torque
        outputs['rotor_thrust'] = thrust
        
        
        

        
class PowerCurve(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        self.add_input('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('machine_rating', units='kW', desc='machine rating')
        self.add_input('rotor_cp', desc='rotor power coefficient')
        self.add_input('rotor_ct', desc='rotor thrust coefficient')
        
        # outputs
        self.add_output('rated_wind_speed', units = 'm/s', desc = 'rated wind speed')
        self.add_output('wind_bin', units = 'm/s', desc='list of wind speeds', shape=num_bins)
        self.add_output('power_bin', units='kW', desc='list of power output', shape=num_bins)
        self.add_output('thrust_bin', units = 'N', desc='list of rotor thrust', shape=num_bins)
        
        
    def compute(self, inputs, outputs):
        
        cut_in_speed = inputs['cut_in_speed'] 
        cut_out_speed = inputs['cut_out_speed'] 
        rotor_diameter = inputs['rotor_diameter'] 
        machine_rating = inputs['machine_rating'] 
        rotor_cp = inputs['rotor_cp']   
        rotor_ct = inputs['rotor_ct']  
        
        rotor_radius = rotor_diameter/2.0
        wind_bin = range(1,num_bins+1)    
        power_bin = []
        thrust_bin = []
        
        rated_speed = (machine_rating * 1000.0 / (rotor_cp * 0.5 * rho_air * pi * rotor_radius**2))**(1.0/3.0)
        
        
        for v in wind_bin:
            if v < cut_in_speed or v > cut_out_speed:
                power = 0
                thrust = 0
            elif v < rated_speed:
                power  = rotor_cp * 0.5 * rho_air * pi * (rotor_radius**2) * (v**3) / 1000.0  
                thrust = rotor_ct * 0.5 * rho_air * pi * (rotor_radius**2) * (v**2)              
            else:
                power = machine_rating    
                cp = (machine_rating * 1000.0)/(0.5 * rho_air * pi * (rotor_radius**2) * (v**3))
                
                # cp = 4a(1-a)^2 => 4a^3 - 8a^2 + 4a - cp = 0
                a = min(np.roots([4, -8, 4, -1*cp]))
                ct = 4*a*(1-a)
                thrust = ct * 0.5 * rho_air * pi * (rotor_radius**2) * (rated_speed**2)
            
            power_bin.append(power)        
            thrust_bin.append(thrust)
        
        if plot_graphs:
            # plot power curve
            plt.close('all')
            plt.plot(wind_bin, power_bin)
            plt.xlabel('Wind Speed [m/s]')
            plt.ylabel('Power [kW]')
            #plt.show()
            plt.savefig('Plots/power_curve.png')
            
            # plot thrust curve
            plt.close('all')
            plt.plot(wind_bin, thrust_bin)
            plt.xlabel('Wind Speed [m/s]')
            plt.ylabel('Thrust [N]')
            #plt.show()
            plt.savefig('Plots/thrust_curve.png')
        
        outputs['rated_wind_speed'] = rated_speed        
        outputs['wind_bin'] = wind_bin
        outputs['power_bin'] = power_bin
        outputs['thrust_bin'] = thrust_bin    

        
        
        
        
        
class RotorAerodynamics(Group):
    def setup(self):
             
        self.add_subsystem('annulus', AnnulusAero(), \
                           promotes_inputs = ['wind_speed', 'blade_number', 'rotor_diameter', 'rotor_speed', 'hub_radius', \
                                              'span_r', 'span_dr', 'span_airfoil', 'span_chord', 'span_twist', 'pitch'], \
                           promotes_outputs=['span_fx', 'span_fy']
                           )
        
        self.add_subsystem('rotor', RotorAero(), \
                           promotes_inputs = ['wind_speed', 'rotor_diameter', 'rotor_speed', 'hub_radius'], \
                           promotes_outputs=['*'])
        
        self.add_subsystem('curve', PowerCurve(), \
                           promotes_inputs = ['cut_in_speed', 'cut_out_speed', 'rotor_diameter', 'machine_rating'], \
                           promotes_outputs=['rated_wind_speed', 'wind_bin', 'power_bin', 'thrust_bin'])
        
        
        self.connect('annulus.span_power', 'rotor.span_power')  
        self.connect('annulus.span_thrust', 'rotor.span_thrust') 
        self.connect('rotor_cp', 'curve.rotor_cp')
        self.connect('rotor_ct', 'curve.rotor_ct') 
        
        
        
           
        





        
class RotorAerodynamicsTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('wind_speed', units = 'm/s', desc = 'hub height wind speed')
        i.add_output('blade_number', desc='number of blades')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('span_r', units='m', desc='spanwise radial location of blade junctions', shape=num_nodes)
        i.add_output('span_dr', units='m', desc='spanwise blade node thickness', shape=num_nodes)
        i.add_output('span_airfoil', desc='list of blade node airfoil ID', shape=num_nodes)
        i.add_output('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        i.add_output('span_twist', units='deg', desc='list of blade node twist angle', shape=num_nodes)
        i.add_output('pitch', units='deg', desc='blade pitch angle')
        i.add_output('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        i.add_output('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        i.add_output('machine_rating', units='kW', desc='machine rating')
        
        # sub-components         
        self.add_subsystem('dof', i)   
        self.add_subsystem('aero', RotorAerodynamics())
               
        # connections
        self.connect('dof.wind_speed', 'aero.wind_speed')
        self.connect('dof.blade_number', 'aero.blade_number')
        self.connect('dof.rotor_diameter', 'aero.rotor_diameter')
        self.connect('dof.rotor_speed', 'aero.rotor_speed')
        self.connect('dof.hub_radius', 'aero.hub_radius')
        self.connect('dof.span_r', 'aero.span_r')
        self.connect('dof.span_dr', 'aero.span_dr')
        self.connect('dof.span_airfoil', 'aero.span_airfoil')
        self.connect('dof.span_chord', 'aero.span_chord')
        self.connect('dof.span_twist', 'aero.span_twist')
        self.connect('dof.pitch', 'aero.pitch')
        self.connect('dof.cut_in_speed', 'aero.cut_in_speed')
        self.connect('dof.cut_out_speed', 'aero.cut_out_speed')
        self.connect('dof.machine_rating', 'aero.machine_rating')
        
        
        


if __name__ == "__main__":
    start = time()
    
    # workflow setup
    prob = Problem(RotorAerodynamicsTest())
    prob.setup()
    view_model(prob, outfile='rotor_aero_siemens.html')
    
    # define inputs
    prob['dof.wind_speed'] = 9.0 
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 108.0
    prob['dof.rotor_speed'] = 14.3
    prob['dof.hub_radius'] = 4.0
    prob['dof.span_r'] = [ 6.5, 11.5, 16.5, 21.5, 26.5, 31.5, 36.5, 41.5, 46.5, 51.5]
    prob['dof.span_dr'] = [5., 5., 5., 5., 5., 5., 5., 5., 5., 5.]
    prob['dof.span_airfoil'] = [0., 0., 1., 1., 2., 2., 2., 2., 2., 2.]
    prob['dof.span_chord'] = [3.4, 3.4, 4.2, 3.2, 1.8, 1.5, 1.3, 1.1, 1.0, 0.9]
    prob['dof.span_twist'] = [4.0, 4.0, 12.4, 9.3, 4.1, 2.8, 1.8, 1.1, 0.5, 0.0]
    prob['dof.pitch'] = -4.0
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 2300.0
    
     
    prob.run_model()
    
    # print outputs 
    print "Rotor Aerodynamics"
    print 'rotor_tsr = ' + beautify(prob['aero.rotor_tsr'])
    print 'swept_area = ' + beautify(prob['aero.swept_area'])
    print 'rotor_cp = ' + beautify(prob['aero.rotor_cp'])
    print 'rotor_cq = ' + beautify(prob['aero.rotor_cq'])
    print 'rotor_ct = ' + beautify(prob['aero.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['aero.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['aero.rotor_torque'])
    print 'rotor_thrust = ' + beautify(prob['aero.rotor_thrust'])
    print 'span_fx = ' + beautify(prob['aero.span_fx'])
    print 'span_fy = ' + beautify(prob['aero.span_fy'])
    print 'rated_wind_speed = ' + beautify(prob['aero.rated_wind_speed'])
    print 'wind_bin = ' + beautify(prob['aero.wind_bin'])
    print 'power_bin = ' + beautify(prob['aero.power_bin'])
    print 'thrust_bin = ' + beautify(prob['aero.thrust_bin'])
  
    print 'Done in ' + str(time() - start) + ' seconds'    
     
    


 