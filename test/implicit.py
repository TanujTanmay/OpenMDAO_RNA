from __future__ import division, print_function
from openmdao.api import ExplicitComponent

class Paraboloid(ExplicitComponent):
    def setup(self):
        self.add_input('x')
        self.add_input('y')
        self.add_output('f_xy')
        
        #self.declare_partials(of='*', wrt='*', method='fd')
        
    def compute(self, inputs, outputs):
        x = inputs['x']
        y = inputs['y']
        outputs['f_xy'] = (x-3.0)**2 + x*y + (y+4.0)**2 - 3.0
        
        
        
class ParaboloidConstraint(ExplicitComponent):
    def setup(self):
        self.add_input('x')
        self.add_input('y')
        self.add_output('g')
        
        #self.declare_partials(of='*', wrt='*', method='fd')
        
    def compute(self, inputs, outputs):
        x = inputs['x']
        y = inputs['y']
        outputs['g'] = x+y       
      
      
      
if __name__ == "__main__":
    from openmdao.api import Problem, Group, IndepVarComp, ScipyOptimizer
    
    # Design variable
    i = IndepVarComp()
    i.add_output('x', 3.0)
    i.add_output('y', -4.0)    

    # Define OpenMDAO hierarchy and connect various components
    
    # with Promote
    model = Group()
    model.add_subsystem('dof', i, promotes=['x', 'y'])
    model.add_subsystem('paraboloid', Paraboloid(), promotes=['x', 'y'])
    model.add_subsystem('constraint', ParaboloidConstraint(), promotes=['x', 'y'])
    
#     # without Promote
#     model = Group()
#     model.add_subsystem('dof', i)
#     model.add_subsystem('paraboloid', Paraboloid())
#     model.add_subsystem('constraint', ParaboloidConstraint())
#     model.connect('dof.x', ['paraboloid.x', 'constraint.x'])
#     model.connect('dof.y', ['paraboloid.y', 'constraint.y'])
    
    
    # get and set values of the variables using Problem
    prob = Problem(model)
    prob.setup()
    prob.run_model()
    print(prob['paraboloid.f_xy'])
    
    prob['dof.x'] = 5.0
    prob['dof.y'] = 2.0
    prob.run_model()
    print(prob['paraboloid.f_xy'])
    
    # optimization problem
#     opt = Problem(model)
#     opt.driver = ScipyOptimizer()
#     opt.driver.options['optimizer'] = 'COBYLA'
#      
#     opt.model.add_design_var('dof.x', lower=-50, upper=50)
#     opt.model.add_design_var('dof.y', lower=-50, upper=50)
#     opt.model.add_objective('paraboloid.f_xy')
#     opt.model.add_constraint('constraint.g', lower=0., upper=10.)   # inequality
#     #opt.model.add_constraint('constraint.g', equals=0.)             # equality
#      
#     opt.setup()
#     opt.run_driver()
#     print(opt['paraboloid.f_xy'], opt['dof.x'], opt['dof.y'])
#     
    
    
    