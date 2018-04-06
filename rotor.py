from time import time

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model
from rotor_design import RotorDesign
from rotor_aerodynamics import AerodynamicsSimple, AerodynamicsAeroDyn
from power_curve import PowerCurve
from rotor_mechanics import RotorMechanicsAnalytical
from fixed_parameters import num_airfoils, beautify, model_aerodynamics, model_mechanics, model_power_curve



#############################################################################
################################  WORKFLOWS #################################
#############################################################################
class Rotor(Group):
    def setup(self):
        # sub systems
        self.add_subsystem('design', RotorDesign(), \
                           promotes_inputs=['*'], \
                           promotes_outputs=['*'])
        
        self.add_subsystem('aero1', globals()[model_aerodynamics](), \
                           promotes_inputs=['*'])
        
        self.add_subsystem('pc', globals()[model_power_curve](), \
                           promotes_inputs=['cut_in_speed', 'cut_out_speed', 'machine_rating'], \
                           promotes_outputs=['*'])
        
        self.add_subsystem('aero2', globals()[model_aerodynamics](), \
                           promotes_inputs=['*'], \
                           promotes_outputs=['*'])
        
        self.add_subsystem('struc', globals()[model_mechanics](), \
                           promotes_inputs=['*'], \
                           promotes_outputs=['*']) 
        
        # connections
        self.connect('aero1.swept_area', 'pc.swept_area')
        self.connect('aero1.rotor_cp', 'pc.rotor_cp')
        self.connect('aero1.rotor_ct', 'pc.rotor_ct')
        self.connect('rated_wind_speed', 'wind_speed')          








#############################################################################
##############################  UNIT TESTING ################################
############################################################################# 
class RotorTest(Group):
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
        #i.add_output('wind_speed', units = 'm/s', desc = 'hub height wind speed')
        i.add_output('hub_height', units = 'm', desc = 'hub height')
        i.add_output('precone', units='deg', desc='blade precone angle')
        i.add_output('yaw', units = 'deg', desc = 'rotor yaw misalignment angle')
        i.add_output('overhang', units='m', desc='overhang distance')
        i.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        i.add_output('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        i.add_output('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        i.add_output('machine_rating', units='kW', desc='machine rating')
        
        # sub-components        
        self.add_subsystem('dof', i, promotes_outputs=['*'])   
        self.add_subsystem('rotor', Rotor(), promotes_inputs=['*'])
        
        
    def test(self):    
        start = time()
    
        # workflow setup
        prob = Problem(RotorTest())
        prob.setup()
        view_model(prob, outfile='N2/rotor_new.html')
        
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
        #prob['dof.wind_speed'] = 10.8588
        prob['dof.hub_height'] = 90.0
        prob['dof.precone'] = -2.5
        prob['dof.yaw'] = 0.0
        prob['dof.overhang'] = -5.0191
        prob['dof.shaft_angle'] = -5.0
        prob['dof.cut_in_speed'] = 3.0
        prob['dof.cut_out_speed'] = 25.0
        prob['dof.machine_rating'] = 5000.0
         
        prob.run_model()
        
        # print outputs 
        print "Rotor Properties"
        print 'span_r = ' + beautify(prob['rotor.span_r'])
        print 'span_dr = ' + beautify(prob['rotor.span_dr'])
        print 'span_airfoil = ' + beautify(prob['rotor.span_airfoil'])
        print 'span_chord = ' + beautify(prob['rotor.span_chord'])
        print 'span_twist = ' + beautify(prob['rotor.span_twist'])
        print 'pitch = ' + beautify(prob['rotor.pitch']) 
        print 'span_thickness = ' + beautify(prob['rotor.span_thickness'])
        print 'span_mass = ' + beautify(prob['rotor.span_mass'])
        print 'span_flap_inertia = ' + beautify(prob['rotor.span_flap_inertia'], 0)
        print 'span_edge_inertia = ' + beautify(prob['rotor.span_edge_inertia'], 0)  
        print 'span_flap_stiff = ' + beautify(prob['rotor.span_flap_stiff'], 0)
        print 'span_edge_stiff = ' + beautify(prob['rotor.span_edge_stiff'], 0)
        print 'blade_mass = ' + beautify(prob['rotor.blade_mass'])  
        print 'swept_area = ' + beautify(prob['rotor.swept_area'])
        print 'rotor_cp = ' + str(prob['rotor.rotor_cp'])
        print 'rotor_cq = ' + str(prob['rotor.rotor_cq'])
        print 'rotor_ct = ' + str(prob['rotor.rotor_ct'])
        print 'rotor_power = ' + beautify(prob['rotor.rotor_power'])
        print 'rotor_torque = ' + beautify(prob['rotor.rotor_torque'])
        print 'rotor_thrust = ' + beautify(prob['rotor.rotor_thrust'])
        print 'rotor_force = ' + beautify(prob['rotor.rotor_force'])
        print 'rotor_moment = ' + beautify(prob['rotor.rotor_moment'])
        print 'span_fx = ' + beautify(prob['rotor.span_fx'])
        print 'span_fy = ' + beautify(prob['rotor.span_fy'])  
        print 'rated_wind_speed = ' + beautify(prob['rotor.rated_wind_speed'])
        print 'wind_bin = ' + beautify(prob['rotor.wind_bin'])
        print 'power_bin = ' + beautify(prob['rotor.power_bin'])
        print 'thrust_bin = ' + beautify(prob['rotor.thrust_bin'])
        print 'span_moment_flap = ' + beautify(prob['rotor.span_moment_flap'])
        print 'span_moment_edge = ' + beautify(prob['rotor.span_moment_edge'])
        print 'span_moment_gravity = ' + beautify(prob['rotor.span_moment_gravity'])
        print 'span_stress_flap = ' + beautify(prob['rotor.span_stress_flap'])
        print 'span_stress_edge = ' + beautify(prob['rotor.span_stress_edge'])
        print 'span_stress_gravity = ' + beautify(prob['rotor.span_stress_gravity'])
        print 'tip_deflection = ' + beautify(prob['rotor.tip_deflection'])  
      
        print 'Done in ' + str(time() - start) + ' seconds'

        
if __name__ == "__main__":
    RotorTest().test()
