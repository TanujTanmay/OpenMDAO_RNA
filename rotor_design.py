from time import time

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model
from fixed_parameters import beautify, num_nodes, num_airfoils, model_aerodynamic_design, model_structural_design
from aerodynamic_design import AerodynamicDesignBetz, AerodynamicDesignScaling
from structural_design import StructuralDesignConstantRadius, StructuralDesignVariableRadius



#############################################################################
################################  WORKFLOWS #################################
#############################################################################
class RotorDesign(Group):
    '''
        this setup is used to design the rotor when the following holds true:
        - aerodynamic profile is determined analytically using Betz law
        - structural profile is scaled using the new chord distribution
    '''
    def setup(self):             
        self.add_subsystem('aero', globals()[model_aerodynamic_design](), \
                           promotes_inputs = ['*'], \
                           promotes_outputs=['*']
                           )
        
        self.add_subsystem('struc', globals()[model_structural_design](), \
                           promotes_inputs = ['*'], \
                           promotes_outputs=['*'])







#############################################################################
##############################  UNIT TESTING ################################
#############################################################################         
class RotorDesignTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('design_tsr', desc='design tip speed ratio')
        i.add_output('blade_number', desc='number of blades')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('root_chord', units = 'm', desc = 'length of root chord')
        i.add_output('span_airfoil_r', units='m', desc='list of blade node radial location at which the airfoils are specified', shape=num_airfoils)
        i.add_output('span_airfoil_id', desc='list of blade node Airfoil ID', shape=num_airfoils)
        i.add_output('adjust_pitch', desc = 'adjust pitch for tip twist to be 0')
        i.add_output('thickness_factor', desc='scaling factor for laminate thickness')

        # sub-components        
        self.add_subsystem('dof', i, promotes=['*'])   
        self.add_subsystem('design', RotorDesign(), promotes_inputs=['*'])
        
    def test(self):
        start = time()
        
        # workflow setup
        prob = Problem(RotorDesignTest())
        prob.setup()
        view_model(prob, outfile='N2/rotor_design.html')
        
        # define inputs
        prob['dof.design_tsr'] = 7.0
        prob['dof.blade_number'] = 3
        prob['dof.rotor_diameter'] = 126.0
        prob['dof.hub_radius'] = 1.5
        prob['dof.root_chord'] = 3.4
        prob['dof.span_airfoil_r'] =  [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
        prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]
        prob['dof.adjust_pitch'] = 1    
        prob['dof.thickness_factor'] = 1.0
         
        prob.run_model()
        
        # print outputs 
        print "Rotor Design"
        print 'span_r = ' + beautify(prob['design.span_r'])
        print 'span_dr = ' + beautify(prob['design.span_dr'])
        print 'span_airfoil = ' + beautify(prob['design.span_airfoil'])
        print 'span_chord = ' + beautify(prob['design.span_chord'])
        print 'span_twist = ' + beautify(prob['design.span_twist'])
        print 'pitch = ' + beautify(prob['design.pitch']) 
        print 'span_thickness = ' + beautify(prob['design.span_thickness'])
        print 'span_mass = ' + beautify(prob['design.span_mass'])
        print 'span_flap_inertia = ' + beautify(prob['design.span_flap_inertia'], 0)
        print 'span_edge_inertia = ' + beautify(prob['design.span_edge_inertia'], 0)  
        print 'span_flap_stiff = ' + beautify(prob['design.span_flap_stiff'], 0)
        print 'span_edge_stiff = ' + beautify(prob['design.span_edge_stiff'], 0)
        print 'blade_mass = ' + beautify(prob['design.blade_mass']) 
        print 'rotor_mass = ' + beautify(prob['design.rotor_mass'])      
      
        print 'Done in ' + str(time() - start) + ' seconds'            
        
 


if __name__ == "__main__":
    RotorDesignTest().test()
    


 