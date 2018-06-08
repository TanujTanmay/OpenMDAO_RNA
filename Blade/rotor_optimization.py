import numpy as np
from time import time

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model, \
                        ScipyOptimizer, ExecComp, DirectSolver
from fixed_parameters import num_airfoils, degree_bezier, num_nodes, beautify, UTS
from rotor import Rotor
from rna import RNA

class RotorOptimizer(Group):
    def setup(self):
        i = IndepVarComp()
        
        # variables
        i.add_output('in_design_tsr', desc='design tip speed ratio')
        i.add_output('in_tf', desc='scaling factor for laminate thickness')
        i.add_output('in_chord_coefficients', desc = 'coefficients of polynomial chord profile', shape=degree_bezier)
        i.add_output('in_twist_coefficients', desc = 'coefficients of polynomial twist profile', shape=degree_bezier)
    
        
        # parameters
        self.add_subsystem('dof', i, promotes=['*'])        
        self.add_subsystem('rna', RNA(), promotes=['*'])
        
        self.add_subsystem('obj', ExecComp('f = -1 * cp / 0.4967'))
        #self.add_subsystem('obj', ExecComp('f = -1 * (cp/(rm)) / (0.4967/51092.66)', rm={'units' : 'kg'}))
        #self.add_subsystem('obj', ExecComp('f = -1 * (cp/(rm+hm+nm)) / (0.4967/217493.9)', rm={'units' : 'kg'}, hm={'units' : 'kg'}, nm={'units' : 'kg'}))
        #self.add_subsystem('obj', ExecComp('f = -1 * (cp/crt) / (0.4967/3202937.51)', crt={'units' : 'USD'}))
        
        self.add_subsystem('c1', ExecComp('tip = (y/5.6964)-1.0', tip={'units' : 'm'}, y={'units' : 'm'}))#, d={'units' : 'm'}))
        self.add_subsystem('c2', ExecComp('max_s = s/(450.0*10**6) - 1.0', s={'units' : 'Pa'},  max_s={'units' : 'Pa'}))

        
        self.connect('rotor_cp', 'obj.cp')
        #self.connect('rotor_mass', 'obj.rm')
        #self.connect('hub_system_mass', 'obj.hm')
        #self.connect('nacelle_mass', 'obj.nm')
        #self.connect('cost_rna_total', 'obj.crt')
        
        self.connect('tip_deflection', 'c1.y')
        self.connect('span_stress_max', 'c2.s')


if __name__ == "__main__":
    
    start = time()
    
    # coefficients
    c = np.ones(degree_bezier)
    t = np.ones(degree_bezier)
    
    # get and set values of the variables using Problem
    prob = Problem()
    prob.model = RotorOptimizer()
    prob.setup()
    #view_model(prob)
    
    prob.driver = ScipyOptimizer()
    prob.driver.options['optimizer'] = 'SLSQP'
    prob.driver.options['maxiter'] = 100
    prob.driver.options['tol'] = 1e-4
    
    prob.model.add_design_var('in_design_tsr', lower=0.85, upper=2.0)
    prob.model.add_design_var('in_tf', lower=0.5, upper=1.5)
    prob.model.add_design_var('in_chord_coefficients', lower=0.50*c, upper=1.50*c)
    prob.model.add_design_var('in_twist_coefficients', lower=0.50*t, upper=1.50*t)
    prob.model.add_objective('obj.f')
    prob.model.add_constraint('c1.tip', equals=0)
    prob.model.add_constraint('c2.max_s', upper=0)

    
    
    prob.setup()
    #view_model(prob, 'opt.html')
    
    prob['in_design_tsr'] = 1.0
    prob['in_tf'] = 1.0
    prob['in_chord_coefficients'] = c
    prob['in_twist_coefficients'] = t
    
    prob.run_model()
    
    prob.set_solver_print(level=2)
    
    # Ask OpenMDAO to finite-difference across the model to compute the gradients for the optimizer
    prob.model.approx_totals()
    
    
    prob.run_driver()
    
    print('minimum found at')
    print prob['in_design_tsr'][0]
    print prob['in_tf'][0] 
    print beautify(prob['in_chord_coefficients'])
    print beautify(prob['in_twist_coefficients'])
    ##
    print 'span_r = ' + beautify(prob['span_r'])
    print 'span_chord = ' + beautify(prob['span_chord'])
    print 'span_twist = ' + beautify(prob['span_twist'])
    print 'pitch = ' + beautify(prob['pitch'])
    print 'rotor_cp = ' + str(prob['rotor_cp'])
    print 'rotor_ct = ' + str(prob['rotor_ct'])
    print 'rated_wind_speed = ' + beautify(prob['rated_wind_speed'])
    print 'wind_bin = ' + beautify(prob['wind_bin'])
    print 'power_bin = ' + beautify(prob['power_bin'])
    print 'span_stress_max = ' + str(max(prob['span_stress_max']))
    print 'tip_deflection = ' + beautify(prob['tip_deflection'])
    print 'rotor_mass = ' + beautify(prob['rotor_mass'])
    print 'hub_system_mass = ' + beautify(prob['hub_system_mass'])
    print 'nacelle_mass = ' + beautify(prob['nacelle_mass'])
    print 'gb_mass = ' + beautify(prob['gearbox_mass'])
    print 'rna_mass = ' + str(prob['rotor_mass'][0] + prob['hub_system_mass'][0] + prob['nacelle_mass'][0])
    print 'rna_cost = ' + beautify(prob['cost_rna_total'])
    
    print 'Done in ' + str(time() - start) + ' seconds'

