from time import time

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model
from rotor_design import RotorDesign
from rotor_aerodynamics import RotorAerodynamics
from rotor_structures import RotorStructure
from fixed_parameters import num_airfoils, beautify


class Rotor(Group):
    def setup(self):
        
        self.add_subsystem('design', RotorDesign(), \
                           promotes_inputs=['*'], \
                           promotes_outputs=['*'])
                           #promotes_outputs=['span_r', 'span_chord', 'span_twist', 'pitch'])
        
        self.add_subsystem('aero', RotorAerodynamics(), \
                           promotes_inputs=['*'], \
                           promotes_outputs=['*'])
#                            promotes_inputs=['wind_speed', 'blade_number', 'rotor_diameter', 'rotor_speed', 'hub_radius', \
#                                             'cut_in_speed', 'cut_out_speed', 'machine_rating'], \
                           
        
        self.add_subsystem('struc', RotorStructure(), \
                           promotes_inputs=['*'], \
                           promotes_outputs=['*'])        
        
        
        
        # connections
#         self.connect('span_r', ['aero.span_r', 'struc.span_r'])
#         self.connect('design.span_dr', ['aero.span_dr', 'struc.span_dr'])
#         self.connect('design.span_airfoil', ['aero.span_airfoil'])
#         self.connect('span_chord', ['aero.span_chord', 'struc.span_chord'])
#         self.connect('span_twist', ['aero.span_twist'])



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
        i.add_output('wind_speed', units = 'm/s', desc = 'hub height wind speed')
        i.add_output('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        i.add_output('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        i.add_output('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        i.add_output('machine_rating', units='kW', desc='machine rating')
        i.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        
        # sub-components        
        self.add_subsystem('dof', i, promotes_outputs=['*'])   
        self.add_subsystem('rotor', Rotor(), promotes_inputs=['*'])
        

        
if __name__ == "__main__":
    start = time()
    
    # workflow setup
    prob = Problem(RotorTest())
    prob.setup()
    view_model(prob, outfile='rotor_siemens.html')
    
    # define inputs
    prob['dof.design_tsr'] = 9.0
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 108.0
    prob['dof.hub_radius'] = 4.0
    prob['dof.root_chord'] = 3.4
    prob['dof.span_airfoil_r'] = [4.0, 16.2]
    prob['dof.span_airfoil_id'] = [0, 1]
    prob['dof.adjust_pitch'] = 1    
    prob['dof.thickness_factor'] = 1.0
    prob['dof.wind_speed'] = 9.0 
    prob['dof.rotor_speed'] = 14.3
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 2300.0
    prob['dof.shaft_angle'] = 0.0
     
    prob.run_model()
    
    # print outputs 
    print "Rotor Design"
    print 'span_r = ' + beautify(prob['rotor.span_r'])
    print 'span_chord = ' + beautify(prob['rotor.span_chord'])
    print 'span_twist = ' + beautify(prob['rotor.span_twist'])
    print 'pitch = ' + beautify(prob['rotor.pitch'])
    print 'rotor_tsr = ' + beautify(prob['rotor.rotor_tsr'])
    print 'swept_area = ' + beautify(prob['rotor.swept_area'])
    print 'rotor_cp = ' + beautify(prob['rotor.rotor_cp'])
    print 'rotor_cq = ' + beautify(prob['rotor.rotor_cq'])
    print 'rotor_ct = ' + beautify(prob['rotor.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['rotor.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['rotor.rotor_torque'])
    print 'span_fx = ' + beautify(prob['rotor.span_fx'])
    print 'span_fy = ' + beautify(prob['rotor.span_fy'])
    print 'rotor_thrust = ' + beautify(prob['rotor.rotor_thrust'])
    print 'rated_wind_speed = ' + beautify(prob['rotor.rated_wind_speed'])
    print 'wind_bin = ' + beautify(prob['rotor.wind_bin'])
    print 'power_bin = ' + beautify(prob['rotor.power_bin'])
    print 'thrust_bin = ' + beautify(prob['rotor.thrust_bin'])
    print 'blade_mass = ' + beautify(prob['rotor.blade_mass'])
    print 'span_moment_flap = ' + beautify(prob['rotor.span_moment_flap'])
    print 'span_moment_edge = ' + beautify(prob['rotor.span_moment_edge'])
    print 'rotor_mass = ' + beautify(prob['rotor.rotor_mass'])
    print 'span_stress_flap = ' + beautify(prob['rotor.span_stress_flap'])
    print 'span_stress_edge = ' + beautify(prob['rotor.span_stress_edge'])
    print 'span_stress_gravity = ' + beautify(prob['rotor.span_stress_gravity'])
    print 'tip_deflection = ' + beautify(prob['rotor.tip_deflection'])
    print 'root_bending_moment = ' + beautify(prob['rotor.root_bending_moment'])
    print 'root_force = ' + beautify(prob['rotor.root_force'])      
  
    print 'Done in ' + str(time() - start) + ' seconds'
