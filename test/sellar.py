from __future__ import division, print_function
from openmdao.api import ExplicitComponent, Group, IndepVarComp
from numpy import exp
import sha
from openmdao.solvers.nonlinear.nonlinear_block_gs import NonlinearBlockGS

class Disc1(ExplicitComponent):
    def setup(self):
        self.add_input('x')
        self.add_input('z', shape=2)
        self.add_input('y2')
        self.add_output('y1')
        
        self.declare_partials(of='*', wrt='*', method='fd')
        
    def __init__(self):
        ''' Initialize above yaw mass adder component
        '''
    
        super(Disc1 , self).__init__()    
        
    def compute(self, inputs, outputs):
        x = inputs['x']
        y2 = inputs['y2']
        z1 = inputs['z'][0]
        z2 = inputs['z'][1]
        outputs['y1'] = z1**2 + z2 + x - 0.2*y2
        
class Disc2(ExplicitComponent):
    def setup(self):
        self.add_input('z', shape=2)
        self.add_input('y1')
        self.add_output('y2')
        
        self.declare_partials(of='*', wrt='*', method='fd')
        
    def compute(self, inputs, outputs):
        y1 = inputs['y1']
        z1 = inputs['z'][0]
        z2 = inputs['z'][1]
        outputs['y2'] = y1**.5 + z1 + z2       
        
        
        
class Constraint1(ExplicitComponent):
    def setup(self):
        self.add_input('y1')
        self.add_output('g1')
        
        #self.declare_partials(of='*', wrt='*', method='fd')
        
    def compute(self, inputs, outputs):
        y1 = inputs['y1']
        outputs['g1'] = 3.16-y1    
        
        
class Constraint2(ExplicitComponent):
    def setup(self):
        self.add_input('y2')
        self.add_output('g2')
        
        #self.declare_partials(of='*', wrt='*', method='fd')
        
    def compute(self, inputs, outputs):
        y2 = inputs['y2']
        outputs['g2'] = y2 - 24.0     
        
        
        
class Objective(ExplicitComponent):
    def setup(self):
        self.add_input('x')
        self.add_input('z', shape=2)
        self.add_input('y1')
        self.add_input('y2')
        self.add_output('f')
        
        #self.declare_partials(of='*', wrt='*', method='fd')
        
    def compute(self, inputs, outputs):
        x = inputs['x']
        y1 = inputs['y1']
        y2 = inputs['y2']
        z1 = inputs['z'][0]
        outputs['f'] = x**2 + z1 + y1 + exp(-y2)               



      
class GaussSiedel(Group):
    def setup(self):
        self.add_subsystem('d1', Disc1())
        self.add_subsystem('d2', Disc2())
        
        #self.add_subsystem('d1', Disc1(), promotes_inputs=['x', 'z'], promotes_outputs=['y1'])
        #self.add_subsystem('d2', Disc2(), promotes_inputs=['x', 'z'], promotes_outputs=['y2'])
        
        self.connect('d1.y1', 'd2.y1')
        self.connect('d2.y2', 'd1.y2')
        
        self.nonlinear_solver = NonlinearBlockGS()
        
        
class dof(IndepVarComp):    
    def setup(self):
        self.add_output('x', val=2.0)
        self.add_output('z', val=[-1., -1.])
        
    def compute(self, inputs, outputs):
        pass    

        
class Sellar(Group):
    def setup(self):
        i = IndepVarComp()
        i.add_output('x')
        i.add_output('z', shape=2) 
        
        
        self.add_subsystem('dof', i)
        self.add_subsystem('cycle', GaussSiedel())
        self.add_subsystem('obj', Objective())
        self.add_subsystem('c1', Constraint1())
        self.add_subsystem('c2', Constraint2())
        
#         self.add_subsystem('dof', i, promotes=['*'])
#         self.add_subsystem('cycle', GaussSiedel(), promotes=['*'])
#         self.add_subsystem('obj', Objective(), promotes=['*'])
#         self.add_subsystem('c1', Constraint1())
#         self.add_subsystem('c2', Constraint2())
        
        self.connect('dof.x', ['cycle.d1.x', 'obj.x'])
        self.connect('dof.z', ['obj.z', 'cycle.d1.z', 'cycle.d2.z'])        
        self.connect('cycle.d1.y1', ['obj.y1', 'c1.y1']) 
        self.connect('cycle.d2.y2', ['obj.y2', 'c2.y2']) 
          
      
if __name__ == "__main__":
    from openmdao.api import Problem, ScipyOptimizer
    
    # get and set values of the variables using Problem
    prob = Problem(Sellar())
    prob.setup()
    prob['dof.x'] = 2.
    prob['dof.z'] = [-1., -1.]
    prob.run_model()
    print(prob['cycle.d1.y1'])
    
    
    # optimizer
    opt = Problem(Sellar())
    opt.driver = ScipyOptimizer()
    opt.driver.options['optimizer'] = 'COBYLA'
    # prob.driver.options['maxiter'] = 100
    opt.driver.options['tol'] = 1e-8
    
    opt.model.add_design_var('dof.x', lower=0, upper=10)
    opt.model.add_design_var('dof.z', lower=0, upper=10)
    opt.model.add_objective('obj.f')
    opt.model.add_constraint('c1.g1', upper=0.)   # inequality
    opt.model.add_constraint('c2.g2', upper=0.)   # inequality
    #opt.model.add_constraint('constraint.g', equals=0.)             # equality
     
    opt.setup()
    opt.set_solver_print(level=0)
    opt.model.approx_totals()
    opt.run_driver()
    print(opt['dof.x'], opt['dof.z'], opt['obj.f'], opt['c1.g1'])
    
    
    