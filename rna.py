from openmdao.api import Group, IndepVarComp
        
from Blade.fixed_parameters import num_pegged, num_nodes, num_airfoils         
from Blade import blade, aerodynamic_design, structural_design, rotor_aerodynamics, power_curve, rotor_mechanics
from HubNacelle import hub_assembly, nacelle_assembly, \
                        gearbox, lss, bearing, hss, generator, bedplate, yaw, transformer, above_yaw, rna, nacelle, \
                        hub, pitch, spinner, hub_aerodynamics
from Cost import cost         
        


#############################################################################
################################  WORKFLOW  #################################
#############################################################################
class RNA(Group):
    def initialize(self):
        
        self.metadata.declare('aerodynamic_design_model')
        self.metadata.declare('structural_design_model')
        self.metadata.declare('aerodynamics_model')
        self.metadata.declare('power_curve_model')
        self.metadata.declare('mechanics_model')
        
        self.metadata.declare('hub_model')
        self.metadata.declare('pitch_model')
        self.metadata.declare('spinner_model')
        self.metadata.declare('hub_aero_model')
        
        self.metadata.declare('gearbox_model')
        self.metadata.declare('lss_model')
        self.metadata.declare('main_bearing_model')
        self.metadata.declare('second_bearing_model')
        self.metadata.declare('hss_model')
        self.metadata.declare('generator_model')
        self.metadata.declare('bedplate_model')
        self.metadata.declare('yaw_model')
        self.metadata.declare('transformer_model')
        self.metadata.declare('above_yaw_model')
        self.metadata.declare('rna_model')
        self.metadata.declare('nacelle_model')        
        
        self.metadata.declare('cost_model')
        
    def setup(self):
        # metadata
        aerodynamic_design_model = self.metadata['aerodynamic_design_model']
        structural_design_model = self.metadata['structural_design_model']
        aerodynamics_model = self.metadata['aerodynamics_model']
        power_curve_model = self.metadata['power_curve_model']
        mechanics_model = self.metadata['mechanics_model']
        
        hub_model = self.metadata['hub_model']
        pitch_model = self.metadata['pitch_model']
        spinner_model = self.metadata['spinner_model']
        hub_aero_model = self.metadata['hub_aero_model']
        
        gearbox_model = self.metadata['gearbox_model']
        lss_model = self.metadata['lss_model']
        main_bearing_model = self.metadata['main_bearing_model']
        second_bearing_model = self.metadata['second_bearing_model']
        hss_model = self.metadata['hss_model']
        generator_model = self.metadata['generator_model']
        bedplate_model = self.metadata['bedplate_model']
        yaw_model = self.metadata['yaw_model']
        transformer_model = self.metadata['transformer_model']
        above_yaw_model = self.metadata['above_yaw_model']
        rna_model = self.metadata['rna_model']
        nacelle_model = self.metadata['nacelle_model']
        
        cost_model = self.metadata['cost_model']
        
        self.add_subsystem('blade', blade.Blade(aerodynamic_design_model = aerodynamic_design_model, \
                                                structural_design_model = structural_design_model, \
                                                aerodynamics_model = aerodynamics_model, \
                                                power_curve_model = power_curve_model, \
                                                mechanics_model = mechanics_model), \
                           promotes_inputs=['design_tsr', 'blade_number', 'rotor_diameter', 'hub_radius', \
                                            'root_chord', 'chord_coefficients', 'twist_coefficients', 'span_airfoil_r', 'span_airfoil_id', \
                                            'pitch','thickness_factor','hub_height','precone','yaw','overhang','shaft_angle',\
                                            'cut_in_speed', 'cut_out_speed', 'machine_rating', 'drive_train_efficiency'], \
                           promotes_outputs=['rotor_cp', 'rotor_ct', 'rotor_torque', 'rotor_thrust', \
                                             'rated_wind_speed', 'wind_bin', 'elec_power_bin', 'ct_bin', \
                                             'span_stress_max', 'tip_deflection', 'blade_mass', 'rotor_speed'])
        
        self.add_subsystem('hub', hub_assembly.HubAssembly(hub_model = hub_model, \
                                                           pitch_model = pitch_model, \
                                                           spinner_model = spinner_model, \
                                                           hub_aero_model = hub_aero_model), \
                           promotes_inputs=['machine_rating', 'blade_number', 'rotor_diameter', 'shaft_angle'], \
                           promotes_outputs=[('hub_assembly_mass', 'hub_mass'), 'rotor_mass', 'rotor_force', 'rotor_moment'])        
        
        
        self.add_subsystem('nacelle', nacelle_assembly.NacelleAssembly(gearbox_model = gearbox_model, \
                                                                       lss_model = lss_model, \
                                                                       main_bearing_model = main_bearing_model, \
                                                                       second_bearing_model = second_bearing_model, \
                                                                       hss_model = hss_model, \
                                                                       generator_model = generator_model, \
                                                                       bedplate_model = bedplate_model, \
                                                                       yaw_model = yaw_model, \
                                                                       transformer_model = transformer_model, \
                                                                       above_yaw_model = above_yaw_model, \
                                                                       rna_model = rna_model, \
                                                                       nacelle_model = nacelle_model), \
                           promotes_inputs=['rotor_diameter', 'machine_rating', 'overhang', 'shaft_angle', 'gear_ratio', 'Np', \
                                            'tower_top_diameter', 'gearbox_cm_x'], \
                           promotes_outputs=['nacelle_mass'])
        
        
        self.add_subsystem('cost', cost_model(), \
                           promotes_inputs=['machine_rating', 'rotor_diameter', 'blade_number'], \
                           promotes_outputs=['cost_rna'])
        
        
        
        # connections        
        self.connect('blade.span_chord', ['hub.blade_root_diameter'], src_indices=[0])
        self.connect('blade_mass', ['hub.blade_mass', 'cost.blade_mass'])
        self.connect('blade.root_moment_flap', ['hub.rotor_bending_moment'])
        self.connect('rotor_torque', ['hub.rotor_torque', 'nacelle.rotor_torque']) 
        self.connect('rotor_thrust', ['hub.rotor_thrust', 'nacelle.rotor_thrust']) 
        
        self.connect('rotor_speed', ['nacelle.rotor_speed']) 
        self.connect('rotor_force', ['nacelle.rotor_force']) 
        self.connect('rotor_moment', ['nacelle.rotor_bending_moment']) 
        self.connect('rotor_mass', ['nacelle.rotor_mass']) 
        
        self.connect('hub.hub_mass', 'cost.hub_mass')
        self.connect('hub.pitch_mass', 'cost.pitch_mass')
        self.connect('hub.spinner_mass', 'cost.spinner_mass')
        self.connect('nacelle.lss_mass', 'cost.lss_mass')
        self.connect('nacelle.main_bearing_mass', 'cost.main_bearing_mass')
        self.connect('nacelle.second_bearing_mass', 'cost.second_bearing_mass')
        self.connect('nacelle.gearbox_mass', 'cost.gearbox_mass')
        self.connect('nacelle.hss_mass', 'cost.hss_mass')
        self.connect('nacelle.generator_mass', 'cost.generator_mass')
        self.connect('nacelle.bedplate_mass', 'cost.bedplate_mass')
        self.connect('nacelle.platforms_mass', 'cost.platform_mass')
        self.connect('nacelle.crane_mass', 'cost.crane_mass')
        self.connect('nacelle.yaw_mass', 'cost.yaw_mass')
        self.connect('nacelle.vs_electronics_mass', 'cost.vs_electronics_mass')
        self.connect('nacelle.hvac_mass', 'cost.hvac_mass')
        self.connect('nacelle.cover_mass', 'cost.cover_mass')
        self.connect('nacelle.transformer_mass', 'cost.transformer_mass')
        
        
        
        
        
        

#############################################################################
##############################  UNIT TESTING ################################
############################################################################# 
class RNATest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        i.add_output('design_tsr', desc='design tip speed ratio')
        i.add_output('blade_number', desc='number of blades')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('root_chord', units = 'm', desc = 'length of root chord') 
        i.add_output('chord_coefficients', units = 'm', desc = 'coefficients of polynomial chord profile', shape=num_pegged)
        i.add_output('twist_coefficients', units = 'deg', desc = 'coefficients of polynomial twist profile', shape=num_pegged)       
        i.add_output('span_airfoil_r', units='m', desc='list of blade node radial location at which the airfoils are specified', shape=num_airfoils)
        i.add_output('span_airfoil_id', desc='list of blade node Airfoil ID', shape=num_airfoils)
        i.add_output('pitch', units='deg', desc = 'pitch angle')
        i.add_output('thickness_factor', desc='scaling factor for laminate thickness', shape=num_nodes)
        i.add_output('hub_height', units = 'm', desc = 'hub radius')
        i.add_output('precone', units='deg', desc='blade precone angle')
        i.add_output('yaw', units = 'deg', desc = 'rotor yaw misalignment angle')
        i.add_output('overhang', units='m', desc='overhang distance')
        i.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        i.add_output('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
        i.add_output('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
        i.add_output('machine_rating', units='kW', desc='machine rating')
        i.add_output('drive_train_efficiency', desc='efficiency of aerodynamic to electrical conversion')
        i.add_output('gear_ratio', desc='overall gearbox ratio')
        i.add_output('Np', desc='number of planets in each stage', shape=3)
        i.add_output('gearbox_cm_x', units = 'm', desc = 'distance from tower-top center to gearbox cm--negative for upwind')
        i.add_output('tower_top_diameter', units='m', desc='diameter of tower top')      
        
        # parameters
        self.add_subsystem('dof', i, promotes=['*'])        
        self.add_subsystem('rna', RNA(aerodynamic_design_model = aerodynamic_design.PeggedNodes, \
                                    structural_design_model = structural_design.VariableChord, \
                                    aerodynamics_model = rotor_aerodynamics.BEM, \
                                    power_curve_model = power_curve.PowerCurve, \
                                    mechanics_model = rotor_mechanics.Analytical, \
                                    hub_model = hub.DriveSE, \
                                    pitch_model = pitch.DriveSE, \
                                    spinner_model = spinner.DriveSE, \
                                    hub_aero_model = hub_aerodynamics.Tanuj, \
                                    gearbox_model = gearbox.DriveSE, \
                                    lss_model = lss.DriveSE4pt, \
                                    main_bearing_model = bearing.MainBearing, \
                                    second_bearing_model = bearing.SecondBearing, \
                                    hss_model = hss.DriveSE, \
                                    generator_model = generator.DriveSE, \
                                    bedplate_model = bedplate.DriveSE, \
                                    yaw_model = yaw.DriveSE, \
                                    transformer_model = transformer.DriveSE, \
                                    above_yaw_model = above_yaw.DriveSE, \
                                    rna_model = rna.DriveSE, \
                                    nacelle_model = nacelle.DriveSE, \
                                    cost_model=cost.CSM), 
                                      
                                promotes_inputs=['*'])        
    
           