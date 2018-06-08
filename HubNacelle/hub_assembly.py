from time import time
from math import pi
from openmdao.api import Group, ExecComp, IndepVarComp, Problem, view_model 

import hub, pitch, spinner, hub_aerodynamics
from fixed_parameters import beautify



#############################################################################
################################  WORKFLOWS #################################
#############################################################################
class HubAssembly(Group):
    def initialize(self):
        self.metadata.declare('hub_model')
        self.metadata.declare('pitch_model')
        self.metadata.declare('spinner_model')
        self.metadata.declare('hub_aero_model')
        
    def setup(self):
        # metadata
        hub_model = self.metadata['hub_model']
        pitch_model = self.metadata['pitch_model']
        spinner_model = self.metadata['spinner_model']
        hub_aero_model = self.metadata['hub_aero_model']
        
        # sub-systems     
        self.add_subsystem('hub', hub_model(), \
                           promotes_inputs=['blade_root_diameter', 'machine_rating', 'blade_number'], \
                           promotes_outputs=[('diameter', 'hub_diameter'), ('thickness', 'hub_thickness'), ('mass', 'hub_mass')])
        
        self.add_subsystem('pitch', pitch_model(), \
                           promotes_inputs=['blade_mass', 'rotor_bending_moment', 'blade_number'], \
                           promotes_outputs=[('mass', 'pitch_mass')])
        
        self.add_subsystem('spinner', spinner_model(), \
                           promotes_inputs=['rotor_diameter'],  \
                           promotes_outputs=[('mass', 'spinner_mass')])  
        
        self.add_subsystem('aero', hub_aero_model(), \
                            promotes_inputs=['blade_number', 'blade_mass', 'shaft_angle', 'rotor_torque', 'rotor_thrust'], \
                            promotes_outputs=['hub_assembly_mass', 'rotor_mass', 'rotor_force', 'rotor_moment'])                
        
        # connections
        self.connect('hub_mass', 'aero.hub_mass')
        self.connect('pitch_mass', 'aero.pitch_mass')
        self.connect('spinner_mass', 'aero.spinner_mass')





#############################################################################
##############################  UNIT TESTING ################################
############################################################################# 
class UnitTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('blade_number', desc='number of turbine blades')
        i.add_output('blade_root_diameter', units='m', desc='blade root diameter')
        i.add_output('machine_rating', units = 'MW', desc = 'machine rating of turbine')
        i.add_output('blade_mass', units='kg', desc='mass of one blade')
        i.add_output('rotor_bending_moment', units='N*m', desc='flapwise bending moment at blade root')
        i.add_output('shaft_angle', units='deg', desc='angle of the main shaft inclindation wrt the horizontal')
        i.add_output('rotor_torque', units='N*m', desc='rotor torque')
        i.add_output('rotor_thrust',  units='N', desc='rotor thrust')
        
        # sub-components
        self.add_subsystem('dof', i, promotes=['*'])        
        self.add_subsystem('sys', HubAssembly(hub_model = hub.DriveSE, \
                                            pitch_model = pitch.DriveSE, \
                                            spinner_model = spinner.DriveSE, \
                                            hub_aero_model = hub_aerodynamics.Tanuj), \
                            promotes_inputs=['*'])

    def test(self): 
        start = time()
    
        # workflow setup
        prob = Problem(UnitTest())
        prob.setup()
        #view_model(prob, outfile='hub.html')
        
        # define inputs
        prob['dof.rotor_diameter'] = 126.0
        prob['dof.blade_number'] = 3
        prob['dof.blade_root_diameter'] = 3.542
        prob['dof.machine_rating'] = 5000.0
        prob['dof.blade_mass'] = 17740.0
        prob['dof.rotor_bending_moment'] = (3.06 * pi / 8) * 1.225 * (11.05 ** 2) * (0.0517 * (126.0 ** 3)) / 3.0
        prob['dof.shaft_angle'] = 5.0
        prob['dof.rotor_torque'] = (5000*1000/0.95)/(12.1*pi/30)
        prob['dof.rotor_thrust'] = 599610.0
        
        prob.run_model()
        
        # print outputs 
        params = ['hub_diameter', 'hub_thickness', 'hub_mass', 'pitch_mass', 'spinner_mass', 'hub_assembly_mass', \
                  'rotor_mass', 'rotor_force', 'rotor_moment']
        for p in params:
            beautify(p, prob['sys.'+p])
        
        print 'Done in ' + str(time() - start) + ' seconds'
        
        
if __name__ == "__main__":
    UnitTest().test()        
        
        
        
        
        
