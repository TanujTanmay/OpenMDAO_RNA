from time import time
import numpy as np
import matplotlib.pyplot as plt

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model, \
                        ScipyOptimizer, ExecComp, DirectSolver
from rna import RNAAeroDyn
from fixed_parameters import num_airfoils, num_nodes, beautify


class InitializeVariables(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('in_design_tsr', desc='design tip speed ratio')
        self.add_input('in_rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('in_thickness_factor', desc='scaling factor for laminate thickness')
        self.add_input('in_wind_speed', units = 'm/s', desc = 'hub wind speed')
    
        # returns
        self.add_output('design_tsr', desc='design tip speed ratio')
        self.add_output('blade_number', desc='number of blades')
        self.add_output('rotor_diameter', units='m', desc='rotor diameter')
        self.add_output('hub_radius', units = 'm', desc = 'hub radius')
        self.add_output('root_chord', units = 'm', desc = 'length of root chord')        
        self.add_output('span_airfoil_r', units='m', desc='list of blade node radial location at which the airfoils are specified', shape=num_airfoils)
        self.add_output('span_airfoil_id', desc='list of blade node Airfoil ID', shape=num_airfoils)
        self.add_output('adjust_pitch', desc = 'adjust pitch for tip twist to be 0')
        self.add_output('thickness_factor', desc='scaling factor for laminate thickness')
        self.add_output('wind_speed', units = 'm/s', desc = 'hub wind speed')
        self.add_output('hub_height', units = 'm', desc = 'hub radius')
        self.add_output('precone', units='deg', desc='blade precone angle')
        self.add_output('yaw', units = 'deg', desc = 'rotor yaw misalignment angle')
        self.add_output('overhang', units='m', desc='overhang distance')
        self.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        self.add_output('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        self.add_output('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        self.add_output('machine_rating', units='kW', desc='machine rating')
        self.add_output('gear_ratio', desc='overall gearbox ratio')
        self.add_output('crane', desc='flag for presence of crane')
        self.add_output('shaft_ratio', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
        self.add_output('Np', desc='number of planets in each stage', shape=3)
        self.add_output('shrink_disc_mass', units='kg', desc='Mass of the shrink disc')
        self.add_output('carrier_mass', units='kg', desc='Carrier mass')
        self.add_output('flange_length', units='m', desc='flange length')
        self.add_output('L_rb', units='m', desc='distance between hub center and upwind main bearing')
        self.add_output('gearbox_cm_x', units = 'm', desc = 'distance from tower-top center to gearbox cm--negative for upwind')
        self.add_output('tower_top_diameter', units='m', desc='diameter of tower top')
        self.add_output('hss_length', units = 'm', desc = 'optional high speed shaft length determined by user')

    def compute(self, inputs, outputs):

        rotor_diameter = inputs['in_rotor_diameter']
        
        s = rotor_diameter/126.0
        span_af = [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]


        # outputs  
        outputs['machine_rating'] = 5000.0/0.944
        outputs['blade_number'] = 3
        outputs['rotor_diameter'] = rotor_diameter
        outputs['hub_radius'] = 1.5 * s
        outputs['root_chord'] = 3.4 * s
        outputs['span_airfoil_r'] = np.reshape(np.array([x*s for x in span_af]), 8)
        outputs['span_airfoil_id'] = np.array([0,     1,     2,        3,     4,     5,     6,     7])
        outputs['adjust_pitch'] = 1        
        outputs['hub_height'] = 90.0
        outputs['precone'] = -2.5
        outputs['yaw'] = 0.0
        outputs['shaft_angle'] = -5.0        
        outputs['cut_in_speed'] = 3.0
        outputs['cut_out_speed'] = 25.0        
        outputs['gear_ratio'] = 97.0
        outputs['crane'] = 1
        outputs['shaft_ratio'] = 0.1
        outputs['Np'] =  np.array([3.0,3.0,1.0,])
        outputs['shrink_disc_mass'] = (5000.0 * s**2) /3.0
        outputs['carrier_mass'] = 8000.0
        outputs['flange_length'] = 0.5
        outputs['overhang'] = 5.0191
        outputs['L_rb'] = 1.912
        outputs['gearbox_cm_x'] = 0.1
        outputs['tower_top_diameter'] = 3.87
        outputs['hss_length'] = 1.5

        outputs['design_tsr'] = inputs['in_design_tsr']
        outputs['thickness_factor'] = inputs['in_thickness_factor']
        outputs['wind_speed'] = inputs['in_wind_speed']
        


class Sizing(Group):
    def setup(self):
        
        self.add_subsystem('vars', InitializeVariables(), \
                           promotes_inputs=['*'], \
                           promotes_outputs=['*'])
        
        self.add_subsystem('rna', RNAAeroDyn(), \
                           promotes_inputs=['*'], \
                           promotes_outputs=['*'])

        
        
        
                
class SizingTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        i.add_output('in_design_tsr', desc='design tip speed ratio')
        i.add_output('in_rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('in_thickness_factor', desc='scaling factor for laminate thickness')
        i.add_output('in_wind_speed', units = 'm/s', desc = 'hub wind speed')
        
        # parameters
        self.add_subsystem('dof', i, promotes=['*'])        
        self.add_subsystem('rna', Sizing(), promotes=['*'])

        # optimization
#         self.add_subsystem('obj_cmp', ExecComp('obj = x**2 + z[1] + y1 + exp(-y2)',
#                            z=np.array([0.0, 0.0]), x=0.0),
#                            promotes=['x', 'z', 'y1', 'y2', 'obj'])


      
    
    
if __name__ == "__main__":

    from openmdao.api import Problem, view_model
     
    start = time()
    
    # get and set values of the variables using Problem
    prob = Problem()
    prob.model = SizingTest()
    prob.setup()
    
    prob.driver = ScipyOptimizer()
    prob.driver.options['optimizer'] = 'SLSQP'
    prob.driver.options['maxiter'] = 100
    prob.driver.options['tol'] = 1e-8
    prob.model.add_design_var('in_design_tsr', lower=6, upper=12)
    prob.model.add_design_var('in_rotor_diameter', lower=110, upper=135)
    prob.model.add_design_var('in_thickness_factor', lower=0.5, upper=1.5)
    prob.model.add_design_var('in_wind_speed', lower=11.1, upper=11.1)
    prob.model.add_objective('cost_rna_total')
    
    prob.setup()
    prob.set_solver_print(level=2)

     
    # Ask OpenMDAO to finite-difference across the model to compute the gradients for the optimizer
    prob.model.approx_totals()
    
    
    prob.run_driver()
     
    print "NREL 5 MW"
    print 'rated_wind_speed = ' + beautify(prob['rna.rated_wind_speed'])
    print 'swept_area = ' + beautify(prob['rna.swept_area'])
    print 'rotor_cp = ' + str(prob['rna.rotor_cp'])
    print 'rotor_cq = ' + str(prob['rna.rotor_cq'])
    print 'rotor_ct = ' + str(prob['rna.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['rna.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['rna.rotor_torque'])
    print 'rotor_thrust = ' + beautify(prob['rna.rotor_thrust'])
    print 'pitch = ' + beautify(prob['rna.pitch'])
    print 'span_r = ' + beautify(prob['rna.span_r'])
    print 'span_airfoil = ' + beautify(prob['rna.span_airfoil'])
    print 'span_dr = ' + beautify(prob['rna.span_dr'])
    print 'span_chord = ' + beautify(prob['rna.span_chord'])
    print 'span_twist = ' + beautify(prob['rna.span_twist'])
    print 'span_mass = ' + beautify(prob['rna.span_mass'])
    print 'hub_diameter = ' + beautify(prob['rna.hub_diameter'])
    print 'span_fx = ' + beautify(prob['rna.span_fx'])
    print 'span_fy = ' + beautify(prob['rna.span_fy'])
    print 'span_moment_flap = ' + beautify(prob['rna.span_moment_flap'])
    print 'span_moment_edge = ' + beautify(prob['rna.span_moment_edge'])
    print 'span_stress_flap = ' + beautify(prob['rna.span_stress_flap'])
    print 'span_stress_edge = ' + beautify(prob['rna.span_stress_edge'])
    print 'span_stress_gravity = ' + beautify(prob['rna.span_stress_gravity'])
    print 'tip_deflection = ' + beautify(prob['rna.tip_deflection'])
    print 'hub_system_mass = ' + beautify(prob['rna.hub_system_mass'])
    print 'nacelle_mass = ' + beautify(prob['rna.nacelle_mass'])
    print 'mb_mass = ' + beautify(prob['rna.main_bearing_mass'])
    print 'gb_mass = ' + beautify(prob['rna.gearbox_mass'])
    print 'rotor_mass = ' + beautify(prob['rna.rotor_mass'])
    print 'cost_rna_total = ' + beautify(prob['rna.cost_rna_total'])
    print 'rotor_moment = ' + beautify(prob['rna.rotor_moment'])
    print 'rotor_force = ' + beautify(prob['rna.rotor_force'])

    
    
  
    print 'Done in ' + str(time() - start) + ' seconds'
    
    
    



        