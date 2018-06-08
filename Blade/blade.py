import numpy as np
from time import time

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model

import aerodynamic_design, structural_design, rotor_aerodynamics, power_curve, rotor_mechanics
from fixed_parameters import num_airfoils, num_pegged, num_nodes, beautify

        
        

#############################################################################
################################  WORKFLOWS #################################
#############################################################################
class Blade(Group):
    def initialize(self):
        self.metadata.declare('aerodynamic_design_model')
        self.metadata.declare('structural_design_model')
        self.metadata.declare('aerodynamics_model')
        self.metadata.declare('power_curve_model')
        self.metadata.declare('mechanics_model')
        
    def setup(self):
        # metadata
        aerodynamic_design_model = self.metadata['aerodynamic_design_model']
        structural_design_model = self.metadata['structural_design_model']
        aerodynamics_model = self.metadata['aerodynamics_model']
        power_curve_model = self.metadata['power_curve_model']
        mechanics_model = self.metadata['mechanics_model']
        
        # sub systems
        self.add_subsystem('aero_design', aerodynamic_design_model(), \
                           promotes_inputs=['design_tsr', 'blade_number', 'rotor_diameter', 'hub_radius', \
                                            'root_chord', 'chord_coefficients', 'twist_coefficients', \
                                            'span_airfoil_r', 'span_airfoil_id'], \
                           promotes_outputs=['span_r', 'span_chord', 'span_twist'])
        
        self.add_subsystem('struc_design', structural_design_model(), \
                           promotes_inputs=['rotor_diameter', 'thickness_factor', 'blade_number'], \
                           promotes_outputs=['blade_mass'])
                           
        
        self.add_subsystem('aero_partial', aerodynamics_model(), \
                           promotes_inputs=['design_tsr', 'blade_number', 'rotor_diameter', 'hub_height', 'hub_radius', \
                                            'precone', 'pitch', 'yaw', 'overhang', 'shaft_angle'], \
                           promotes_outputs=[])
                           
        
        self.add_subsystem('pc', power_curve_model(), \
                           promotes_inputs=['design_tsr', 'cut_in_speed', 'cut_out_speed', 'machine_rating', 'drive_train_efficiency'], \
                           promotes_outputs=['rated_wind_speed', 'wind_bin', 'elec_power_bin', 'ct_bin'])
                           
        
        self.add_subsystem('aero_full', aerodynamics_model(), \
                           promotes_inputs=['design_tsr', 'blade_number', 'rotor_diameter', 'hub_height', 'hub_radius', \
                                            'precone', 'pitch', 'yaw', 'overhang', 'shaft_angle'], \
                           promotes_outputs=['rotor_cp', 'rotor_ct', 'rotor_speed', 'rotor_torque', 'rotor_thrust'])
        
        self.add_subsystem('mech', mechanics_model(), \
                           promotes_inputs=['shaft_angle'], \
                           promotes_outputs=['root_moment_flap', 'span_stress_max', 'tip_deflection']) 
        
        # connections
        self.connect('span_r', ['struc_design.span_r', 'aero_partial.span_r', \
                                            'aero_full.span_r', 'mech.span_r'])
        self.connect('aero_design.span_dr', ['struc_design.span_dr', 'aero_partial.span_dr', \
                                             'aero_full.span_dr', 'mech.span_dr'])
        self.connect('aero_design.span_airfoil', ['aero_partial.span_airfoil', 'aero_full.span_airfoil'])
        self.connect('span_chord', ['struc_design.span_chord', 'aero_partial.span_chord', \
                                                'aero_full.span_chord', 'mech.span_chord'])
        self.connect('span_twist', ['aero_partial.span_twist', 'aero_full.span_twist'])
        
        self.connect('struc_design.span_thickness', ['mech.span_thickness'])
        self.connect('struc_design.span_mass', ['mech.span_mass'])
        self.connect('struc_design.span_flap_stiff', ['mech.span_flap_stiff'])
        self.connect('struc_design.span_edge_stiff', ['mech.span_edge_stiff'])
        
        self.connect('aero_partial.swept_area', ['pc.swept_area'])
        self.connect('aero_partial.rotor_cp', ['pc.rotor_cp'])
        self.connect('aero_partial.rotor_ct', ['pc.rotor_ct'])
        
        self.connect('rated_wind_speed', ['aero_full.wind_speed'])
        
        self.connect('aero_full.span_fx', ['mech.span_fx'])
        self.connect('aero_full.span_fy', ['mech.span_fy'])    








#############################################################################
##############################  UNIT TESTING ################################
############################################################################# 
class UnitTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('design_tsr', desc='design tip speed ratio')
        i.add_output('blade_number', desc='number of blades')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('root_chord', units = 'm', desc = 'length of root chord')
        i.add_output('chord_coefficients', units = 'm', desc = 'coefficients of polynomial chord profile', shape=num_pegged)
        i.add_output('twist_coefficients', units = 'deg', desc = 'coefficients of polynomial twist profile', shape=num_pegged)
        i.add_output('span_airfoil_r', units='m', desc='list of blade node radial location at which the airfoils are specified', shape=num_airfoils)
        i.add_output('span_airfoil_id', desc='list of blade node Airfoil ID', shape=num_airfoils)
        i.add_output('pitch', units = 'deg', desc = 'blade pitch angle')
        i.add_output('thickness_factor', desc='scaling factor for laminate thickness', shape=num_nodes)
        #i.add_output('wind_speed', units = 'm/s', desc = 'hub height wind speed')
        i.add_output('hub_height', units = 'm', desc = 'hub height')
        i.add_output('precone', units='deg', desc='blade precone angle')
        i.add_output('yaw', units = 'deg', desc = 'rotor yaw misalignment angle')
        i.add_output('overhang', units='m', desc='overhang distance')
        i.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        i.add_output('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        i.add_output('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        i.add_output('machine_rating', units='kW', desc='machine rating')
        i.add_output('drive_train_efficiency', desc='efficiency of aerodynamic to electrical conversion')
        
        # sub-components        
        self.add_subsystem('dof', i, promotes_outputs=['*'])   
        self.add_subsystem('rotor', Blade(aerodynamic_design_model = aerodynamic_design.Scaling, \
                                        structural_design_model = structural_design.VariableChord, \
                                        aerodynamics_model = rotor_aerodynamics.BEM, \
                                        power_curve_model = power_curve.PowerCurve, \
                                        mechanics_model = rotor_mechanics.Analytical), \
                                    promotes_inputs=['*'])
        
        
    def test(self):    
        start = time()
    
        # workflow setup
        prob = Problem(UnitTest())
        prob.setup()
        #view_model(prob, outfile='N2/rotor_new.html')
        
        # define inputs
        prob['dof.design_tsr'] = 7.0
        prob['dof.blade_number'] = 3
        prob['dof.rotor_diameter'] = 126.0
        prob['dof.hub_radius'] = 1.5
        #prob['dof.root_chord'] = 3.4
        prob['dof.chord_coefficients'] = [3.542, 3.01, 2.313]
        prob['dof.twist_coefficients'] = [13.308, 9.0, 3.125]
        prob['dof.span_airfoil_r'] =  [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
        prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]
        prob['dof.pitch'] = 0.0
        prob['dof.thickness_factor'] = [1.0]*num_nodes
        #prob['dof.wind_speed'] = 10.8588
        prob['dof.hub_height'] = 90.0
        prob['dof.precone'] = -2.5
        prob['dof.yaw'] = 0.0
        prob['dof.overhang'] = -5.0191
        prob['dof.shaft_angle'] = -5.0
        prob['dof.cut_in_speed'] = 3.0
        prob['dof.cut_out_speed'] = 25.0
        prob['dof.machine_rating'] = 5000.0
        prob['dof.drive_train_efficiency'] = 1.0
         
        prob.run_model()
        
        
        
        # print outputs 
        var_list = ['rotor_torque', 'rotor_thrust', 'rated_wind_speed' ,'wind_bin', 'elec_power_bin', \
            'ct_bin', 'root_moment_flap', 'span_stress_max', 'tip_deflection', 'blade_mass', \
            'aero_full.rotor_cp', 'aero_partial.rotor_cp']
        
        print "Rotor Properties"
        for p in var_list:
            beautify(p, prob['rotor.'+p])
         
      
        print 'Done in ' + str(time() - start) + ' seconds'

        
if __name__ == "__main__":
    UnitTest().test()
