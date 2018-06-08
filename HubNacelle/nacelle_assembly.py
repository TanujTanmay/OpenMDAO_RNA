from time import time
from math import pi
from openmdao.api import Group, IndepVarComp, Problem, view_model 

import gearbox, lss, bearing, hss, generator, bedplate, yaw, transformer, above_yaw, rna, nacelle
from fixed_parameters import beautify


#############################################################################
################################  WORKFLOWS #################################
#############################################################################
class NacelleAssembly(Group):
    
    def initialize(self):
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
        

        
    def setup(self):
        # metadata
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
        
        # sub-systems  
        self.add_subsystem('gearbox', gearbox_model(), \
                            promotes_inputs=['gear_ratio', 'Np', 'rotor_speed', 'rotor_diameter', \
                                             'rotor_torque', 'gearbox_cm_x'], \
                            promotes_outputs=[('mass', 'gearbox_mass')])
        

        self.add_subsystem('lss', lss_model(), \
                           promotes_inputs=['rotor_bending_moment', 'rotor_force', 'rotor_mass', 'rotor_diameter', \
                                            'machine_rating', 'overhang', 'shaft_angle'], \
                           promotes_outputs=[('mass', 'lss_mass')])

        
        self.add_subsystem('main_bearing', main_bearing_model(), \
                           promotes_inputs=['rotor_diameter', 'rotor_torque'], \
                           promotes_outputs=[('mass', 'main_bearing_mass')])
        
        self.add_subsystem('second_bearing', second_bearing_model(), \
                           promotes_inputs=['rotor_diameter', 'rotor_torque'], \
                           promotes_outputs=[('mass', 'second_bearing_mass')])
        
        self.add_subsystem('hss', hss_model(), \
                           promotes_inputs=['rotor_diameter', 'rotor_torque', 'gear_ratio'], \
                           promotes_outputs=[('mass', 'hss_mass')])        
        
        self.add_subsystem('generator', generator_model(), \
                           promotes_inputs=['rotor_diameter', 'machine_rating', 'gear_ratio', 'rotor_speed'], \
                           promotes_outputs=[('mass', 'generator_mass')])
        
        self.add_subsystem('bedplate', bedplate_model(), \
                           promotes_inputs=['rotor_diameter', 'tower_top_diameter', 'machine_rating', \
                                            'rotor_mass', 'rotor_bending_moment', 'rotor_force', 'overhang'], \
                           promotes_outputs=[('mass', 'bedplate_mass')])
        
        self.add_subsystem('yaw', yaw_model(), \
                           promotes_inputs=['rotor_diameter', 'rotor_thrust', 'tower_top_diameter'], \
                           promotes_outputs=[('mass', 'yaw_mass')])
        
        self.add_subsystem('transformer', transformer_model(), \
                           promotes_inputs=['machine_rating', 'rotor_diameter', 'tower_top_diameter', 'rotor_mass', 'overhang'], \
                           promotes_outputs=[('mass', 'transformer_mass')])
        
        self.add_subsystem('above_yaw', above_yaw_model(), \
                           promotes_inputs=['machine_rating'], \
                           promotes_outputs=['hvac_mass', 'crane_mass', 'platforms_mass', \
                                             'vs_electronics_mass', 'cover_mass', 'mainframe_mass'])
        
        self.add_subsystem('rna', rna_model(), \
                           promotes_inputs=['overhang', 'rotor_mass', 'machine_rating'], \
                           promotes_outputs=[])
        
        self.add_subsystem('nacelle', nacelle_model(), \
                           promotes_inputs=[], \
                           promotes_outputs=['nacelle_mass'])



        # connections
        self.connect('lss.diameter1', ['main_bearing.lss_diameter', 'hss.lss_diameter'])
        self.connect('lss.diameter2', 'second_bearing.lss_diameter')        
        self.connect('lss.bearing_location1', 'main_bearing.location')
        self.connect('lss.bearing_location2', 'second_bearing.location') 
        self.connect('lss.length', 'bedplate.lss_length')
        self.connect('gearbox.length', ['lss.gearbox_length', 'hss.gearbox_length', 'bedplate.gbx_length'])
        self.connect('gearbox.height', ['hss.gearbox_height'])
        self.connect('hss.length','generator.hss_length')
        self.connect('bedplate.height', 'yaw.bedplate_height')
        self.connect('bedplate.length', 'above_yaw.bedplate_length')
        self.connect('bedplate.width', 'above_yaw.bedplate_width')
        self.connect('lss.FW_mb1', 'bedplate.FW_mb1')
        
        self.connect('lss.bearing_mass1',['main_bearing.bearing_mass'])
        self.connect('lss.bearing_mass2',['second_bearing.bearing_mass'])
        self.connect('lss_mass', ['bedplate.lss_mass','above_yaw.lss_mass', 'nacelle.lss_mass', 'rna.lss_mass'])
        self.connect('main_bearing_mass', ['bedplate.mb1_mass','above_yaw.main_bearing_mass', \
                                           'nacelle.main_bearing_mass', 'rna.main_bearing_mass'])
        self.connect('second_bearing_mass', ['bedplate.mb2_mass','above_yaw.second_bearing_mass', \
                                             'nacelle.second_bearing_mass', 'rna.second_bearing_mass'])
        self.connect('gearbox_mass', ['lss.gearbox_mass', 'bedplate.gbx_mass', 'above_yaw.gearbox_mass', \
                                      'nacelle.gearbox_mass', 'rna.gearbox_mass'])
        self.connect('hss_mass', ['bedplate.hss_mass','above_yaw.hss_mass', 'nacelle.hss_mass', 'rna.hss_mass'])
        self.connect('generator_mass', ['bedplate.generator_mass','above_yaw.generator_mass', \
                                        'nacelle.generator_mass', 'rna.generator_mass'])
        self.connect('bedplate_mass', ['above_yaw.bedplate_mass', 'nacelle.bedplate_mass'])
        self.connect('transformer_mass', ['above_yaw.transformer_mass','bedplate.transformer_mass','nacelle.transformer_mass'])
        self.connect('mainframe_mass', ['nacelle.mainframe_mass'])
        self.connect('above_yaw.above_yaw_mass', ['yaw.above_yaw_mass', 'nacelle.above_yaw_mass'])
        self.connect('yaw_mass', ['nacelle.yawMass', 'rna.yawMass'])
        self.connect('rna.RNA_mass', ['transformer.RNA_mass'])
        
        
        self.connect('lss.cm', ['nacelle.lss_cm', 'rna.lss_cm'])
        self.connect('lss.cm', 'bedplate.lss_location', src_indices=0)
        self.connect('main_bearing.cm', ['nacelle.main_bearing_cm', 'rna.main_bearing_cm'])
        self.connect('main_bearing.cm', 'bedplate.mb1_location', src_indices=0)
        self.connect('second_bearing.cm', ['nacelle.second_bearing_cm', 'rna.second_bearing_cm'])
        self.connect('second_bearing.cm', 'bedplate.mb2_location', src_indices=0)
        self.connect('gearbox.cm', ['lss.gearbox_cm', 'hss.gearbox_cm', 'nacelle.gearbox_cm', 'rna.gearbox_cm'])
        self.connect('gearbox.cm', 'bedplate.gbx_location', src_indices=0)
        self.connect('hss.cm', ['generator.hss_cm', 'nacelle.hss_cm', 'rna.hss_cm'])
        self.connect('hss.cm','bedplate.hss_location', src_indices=0)
        self.connect('generator.cm', ['transformer.generator_cm', 'nacelle.generator_cm', 'rna.generator_cm'])
        self.connect('generator.cm', 'bedplate.generator_location', src_indices=0)
        self.connect('bedplate.cm', 'nacelle.bedplate_cm')
        self.connect('rna.RNA_cm','transformer.RNA_cm', src_indices=0)
        self.connect('transformer.cm','nacelle.transformer_cm')
        self.connect('transformer.cm', 'bedplate.transformer_location', src_indices=0)
        
        self.connect('lss.I', ['nacelle.lss_I'])
        self.connect('main_bearing.I', 'nacelle.main_bearing_I')
        self.connect('second_bearing.I', 'nacelle.second_bearing_I')
        self.connect('gearbox.I', ['nacelle.gearbox_I'])
        self.connect('hss.I', ['nacelle.hss_I'])
        self.connect('generator.I', ['nacelle.generator_I'])
        self.connect('bedplate.I', ['nacelle.bedplate_I'])
        self.connect('transformer.I', 'nacelle.transformer_I')
        
        
        
        
        
        
        
        
        
        
        
#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class UnitTest(Group):
    def setup(self):
        
        # design variables
        i = IndepVarComp()
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('rotor_mass', units='kg', desc='rotor mass')
        i.add_output('rotor_torque', units='N*m', desc='rotor torque at rated power')
        i.add_output('rotor_thrust', units='N', desc='maximum rotor thrust')
        i.add_output('rotor_speed', units='rpm', desc='rotor speed at rated')
        i.add_output('machine_rating', units='kW', desc='machine rating of generator')
        i.add_output('gear_ratio', desc='overall gearbox ratio')
        i.add_output('tower_top_diameter', units='m', desc='diameter of tower top')
        i.add_output('rotor_bending_moment', units='N*m', desc='The bending moment', shape=3)
        i.add_output('rotor_force', units='N', desc='The force along the x axis applied at hub center', shape=3)
        i.add_output('shaft_angle', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
        i.add_output('shaft_ratio', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
        i.add_output('overhang', units='m', desc='Overhang distance')
        i.add_output('gearbox_cm_x', units = 'm', desc = 'distance from tower-top center to gearbox cm--negative for upwind')
        i.add_output('Np', desc='number of planets in each stage', shape=3)
    
        # sub-components
        self.add_subsystem('dof', i, promotes=['*'])        
        self.add_subsystem('sys', NacelleAssembly(gearbox_model = gearbox.DriveSE, \
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
                                            nacelle_model = nacelle.DriveSE), \
                            promotes_inputs=['*'])
    
    
            