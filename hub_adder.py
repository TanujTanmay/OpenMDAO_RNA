from hub import Hub_System_Adder_drive, Hub_drive, PitchSystem_drive, Spinner_drive
from nacelle import *

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model

from time import time, clock
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil

class HubAdder(Group):
    def setup(self):
             
        self.add_subsystem('hub', Hub_drive(), \
                           promotes_inputs=['blade_root_diameter', 'machine_rating', 'blade_number'], \
                           promotes_outputs=[('diameter', 'hub_diameter'), ('thickness', 'hub_thickness'), ('mass', 'hub_mass')])
        
        self.add_subsystem('pitch', PitchSystem_drive(), \
                           promotes_inputs=['blade_mass', 'rotor_bending_moment', 'blade_number'], \
                           promotes_outputs=[('mass', 'pitch_system_mass')])
        
        self.add_subsystem('spinner', Spinner_drive(), promotes_inputs=['rotor_diameter'],  promotes_outputs=[('mass', 'spinner_mass')])
        
        self.add_subsystem('adder', Hub_System_Adder_drive(), \
                           promotes_inputs=['rotor_diameter', 'L_rb', 'shaft_angle', 'MB1_location'], \
                           promotes_outputs=['hub_system_cm', 'hub_system_I', 'hub_system_mass'])
        
        
        self.connect('hub_mass', 'adder.hub_mass')
        self.connect('hub_thickness', 'adder.hub_thickness')
        self.connect('hub_diameter', 'adder.hub_diameter')
        self.connect('pitch_system_mass', 'adder.pitch_system_mass')
        self.connect('spinner_mass', 'adder.spinner_mass')





class NacelleAdder(Group):
    def setup(self):
             
        self.add_subsystem('gearbox', Gearbox_drive(), \
                           promotes_inputs=['gear_ratio', 'Np', 'rotor_speed', 'rotor_diameter', 'rotor_torque', 'gearbox_cm_x'], \
                           promotes_outputs=[('mass', 'gearbox_mass')])
        
        self.add_subsystem('lowSpeedShaft', LowSpeedShaft_drive3pt(), \
                           promotes_inputs=['rotor_bending_moment_x', 'rotor_bending_moment_y', 'rotor_bending_moment_z', \
                                            'rotor_force_x', 'rotor_force_y', 'rotor_force_z', \
                                            'rotor_mass', 'rotor_diameter', 'machine_rating', \
                                            'carrier_mass', 'overhang', 'L_rb', 'shrink_disc_mass', \
                                            'flange_length', 'shaft_angle', 'shaft_ratio'], \
                           promotes_outputs=[('mass', 'low_speed_shaft_mass')])
        
        self.add_subsystem('mainBearing', MainBearing_drive(), \
                           promotes_inputs=[('lss_design_torque', 'rotor_torque'), 'rotor_diameter'], \
                           promotes_outputs=[('mass', 'main_bearing_mass'), ('cm', 'MB1_location')])
        
        self.add_subsystem('secondBearing', SecondBearing_drive(), \
                           promotes_inputs=[('lss_design_torque', 'rotor_torque'), 'rotor_diameter'], \
                           promotes_outputs=[('mass', 'second_bearing_mass')])
        
        self.add_subsystem('highSpeedSide', HighSpeedSide_drive(), \
                           promotes_inputs=['rotor_diameter', 'rotor_torque', 'gear_ratio', 'hss_length'], \
                           promotes_outputs=[('mass', 'high_speed_side_mass')])
        
        self.add_subsystem('generator', Generator_drive(), \
                           promotes_inputs=['rotor_diameter', 'machine_rating', 'gear_ratio', 'rotor_speed'], \
                           promotes_outputs=[('mass', 'generator_mass')])
        
        self.add_subsystem('bedplate', Bedplate_drive(), \
                           promotes_inputs=['tower_top_diameter', 'rotor_diameter', 'machine_rating', \
                                            'rotor_mass', 'rotor_bending_moment_y', 'rotor_force_z', \
                                            'flange_length', 'L_rb', 'overhang'], \
                           promotes_outputs=[('mass', 'bedplate_mass')])
        
        self.add_subsystem('above_yaw_massAdder', AboveYawMassAdder_drive(), \
                           promotes_inputs=['machine_rating', 'crane'], \
                           promotes_outputs=['electrical_mass', 'vs_electronics_mass', 'hvac_mass', \
                                             'controls_mass', 'platforms_mass', 'crane_mass', 'mainframe_mass', \
                                             'cover_mass', 'above_yaw_mass'])
        
        self.add_subsystem('yawSystem', YawSystem_drive(), \
                           promotes_inputs=['rotor_diameter', 'rotor_thrust', 'tower_top_diameter'], \
                           promotes_outputs=[('mass', 'yaw_system_mass')])
        
        self.add_subsystem('rna', RNASystemAdder_drive(), \
                           promotes_inputs=['overhang', 'rotor_mass', 'machine_rating'], \
                           promotes_outputs=['RNA_mass'])
        
        self.add_subsystem('nacelleSystem', NacelleSystemAdder_drive(), \
                           promotes_outputs=['nacelle_mass'])
        
        
        
        
        
        self.connect('gearbox_mass', ['lowSpeedShaft.gearbox_mass', 'bedplate.gbx_mass', 'above_yaw_massAdder.gearbox_mass', 'rna.gearbox_mass', 'nacelleSystem.gearbox_mass'])
        self.connect('gearbox.cm', ['lowSpeedShaft.gearbox_cm', 'highSpeedSide.gearbox_cm', 'rna.gearbox_cm', 'nacelleSystem.gearbox_cm'])
        self.connect('gearbox.length', ['lowSpeedShaft.gearbox_length', 'highSpeedSide.gearbox_length', 'bedplate.gbx_length'])
        self.connect('gearbox.height', ['highSpeedSide.gearbox_height'])
        self.connect('gearbox.I', ['nacelleSystem.gearbox_I'])
        
        self.connect('low_speed_shaft_mass', ['bedplate.lss_mass', 'above_yaw_massAdder.lss_mass', 'rna.lss_mass', 'nacelleSystem.lss_mass'])
        self.connect('lowSpeedShaft.cm', ['rna.lss_cm', 'nacelleSystem.lss_cm'])
        self.connect('lowSpeedShaft.length', ['bedplate.lss_length'])
        self.connect('lowSpeedShaft.FW_mb', ['bedplate.FW_mb1'])
        self.connect('lowSpeedShaft.I', ['nacelleSystem.lss_I'])
        self.connect('lowSpeedShaft.bearing_mass1', 'mainBearing.bearing_mass')
        self.connect('lowSpeedShaft.bearing_mass2', 'secondBearing.bearing_mass')
        self.connect('lowSpeedShaft.bearing_location1', 'mainBearing.location')
        self.connect('lowSpeedShaft.bearing_location2', 'secondBearing.location')
        self.connect('lowSpeedShaft.diameter1', ['mainBearing.lss_diameter', 'highSpeedSide.lss_diameter'])
        self.connect('lowSpeedShaft.diameter2', 'secondBearing.lss_diameter')
        #self.connect('lowSpeedShaft.design_torque', ['mainBearing.lss_design_torque', 'secondBearing.lss_design_torque'])
        
        self.connect('main_bearing_mass', ['bedplate.mb1_mass', 'above_yaw_massAdder.main_bearing_mass', 'rna.main_bearing_mass', 'nacelleSystem.main_bearing_mass'])
        self.connect('MB1_location', ['rna.main_bearing_cm', 'nacelleSystem.main_bearing_cm'])
        self.connect('mainBearing.I', ['nacelleSystem.main_bearing_I'])
        
        self.connect('second_bearing_mass', ['bedplate.mb2_mass', 'above_yaw_massAdder.second_bearing_mass', 'rna.second_bearing_mass', 'nacelleSystem.second_bearing_mass'])
        self.connect('secondBearing.cm', ['rna.second_bearing_cm', 'nacelleSystem.second_bearing_cm'])
        self.connect('secondBearing.I', ['nacelleSystem.second_bearing_I'])       

        
        self.connect('high_speed_side_mass', ['bedplate.hss_mass', 'above_yaw_massAdder.hss_mass', 'rna.hss_mass', 'nacelleSystem.hss_mass'])
        self.connect('highSpeedSide.length', ['generator.highSpeedSide_length'])
        self.connect('highSpeedSide.cm', ['generator.highSpeedSide_cm', 'rna.hss_cm', 'nacelleSystem.hss_cm'])
        self.connect('highSpeedSide.I', ['nacelleSystem.hss_I'])
        
        self.connect('generator_mass', ['bedplate.generator_mass', 'above_yaw_massAdder.generator_mass', 'rna.generator_mass', 'nacelleSystem.generator_mass'])
        self.connect('generator.cm', ['rna.generator_cm', 'nacelleSystem.generator_cm'])
        self.connect('generator.I', ['nacelleSystem.generator_I'])
        
        self.connect('bedplate_mass', ['above_yaw_massAdder.bedplate_mass', 'nacelleSystem.bedplate_mass'])
        self.connect('bedplate.length', ['above_yaw_massAdder.bedplate_length'])
        self.connect('bedplate.width', ['above_yaw_massAdder.bedplate_width'])
        self.connect('bedplate.height', ['yawSystem.bedplate_height'])
        self.connect('bedplate.cm', ['nacelleSystem.bedplate_cm'])
        self.connect('bedplate.I', ['nacelleSystem.bedplate_I'])
        
        self.connect('gearbox.cm', 'bedplate.gbx_location', src_indices=[0])
        self.connect('highSpeedSide.cm', 'bedplate.hss_location', src_indices=[0])
        self.connect('generator.cm', 'bedplate.generator_location', src_indices=[0])
        self.connect('lowSpeedShaft.cm', 'bedplate.lss_location', src_indices=[0])
        self.connect('MB1_location', 'bedplate.mb1_location', src_indices=[0])
        self.connect('secondBearing.cm', 'bedplate.mb2_location', src_indices=[0])
        
        self.connect('above_yaw_mass', ['yawSystem.above_yaw_mass', 'nacelleSystem.above_yaw_mass'])
        self.connect('mainframe_mass', ['nacelleSystem.mainframe_mass'])
        
        self.connect('yaw_system_mass', ['rna.yawMass', 'nacelleSystem.yawMass'])





        
        



        
class HubSE(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('blade_number', desc='number of turbine blades')
        i.add_output('blade_root_diameter', units='m', desc='blade root diameter')
        i.add_output('machine_rating', units = 'MW', desc = 'machine rating of turbine')
        i.add_output('blade_mass', units='kg', desc='mass of one blade')
        i.add_output('rotor_bending_moment', units='N*m', desc='flapwise bending moment at blade root')
        i.add_output('L_rb', units='m', desc='distance between hub center and upwind main bearing')
        i.add_output('shaft_angle', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
        i.add_output('MB1_location', units = 'm', desc = 'x,y,z location from shaft model', shape=3)
        
        
        
        
        # parameters
        self.add_subsystem('dof', i)        
        self.add_subsystem('hub', HubAdder())
        
        
        self.connect('dof.rotor_diameter', 'hub.rotor_diameter')
        self.connect('dof.blade_number', 'hub.blade_number')
        self.connect('dof.blade_root_diameter', 'hub.blade_root_diameter')
        self.connect('dof.machine_rating', 'hub.machine_rating')
        self.connect('dof.blade_mass', 'hub.blade_mass')
        self.connect('dof.rotor_bending_moment', 'hub.rotor_bending_moment')
        self.connect('dof.L_rb', 'hub.L_rb')
        self.connect('dof.shaft_angle', 'hub.shaft_angle')
        self.connect('dof.MB1_location', 'hub.MB1_location')
        
        
class NacelleSE(Group):
    def setup(self):
        
        torque = 1.5 * (5000.0 * 1000 / 0.95) / (12.1 * (pi / 30))
        sd_mass = 333.3*5000.0/1000.0
        # variables
        i = IndepVarComp()
        i.add_output('rotor_diameter', units='m', desc='rotor diameter', val=126.0)
        i.add_output('rotor_speed', units='rpm', desc='rotor speed at rated', val=12.1)
        i.add_output('machine_rating', units='kW', desc='machine rating of generator', val=5000.0)
        i.add_output('rotor_torque', units='N*m', desc='rotor torque at rated power', val=torque)
        i.add_output('rotor_thrust', units='N', desc='maximum rotor thrust', val=599610.0)
        i.add_output('rotor_mass', units='kg', desc='rotor mass')
        i.add_output('rotor_bending_moment_x', units='N*m', desc='The bending moment about the x axis', val=330770.0)
        i.add_output('rotor_bending_moment_y', units='N*m', desc='The bending moment about the y axis', val=-16665000.0)
        i.add_output('rotor_bending_moment_z', units='N*m', desc='The bending moment about the z axis', val=2896300.0)
        i.add_output('rotor_force_x', units='N', desc='The force along the x axis applied at hub center', val=599610.0)
        i.add_output('rotor_force_y', units='N', desc='The force along the y axis applied at hub center', val=186780.0)
        i.add_output('rotor_force_z', units='N', desc='The force along the z axis applied at hub center', val=-842710.0)
        i.add_output('gear_ratio', desc='overall gearbox ratio', val=96.76)
        i.add_output('crane', desc='flag for presence of crane', val=1)
        i.add_output('shaft_angle', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal', val=5.0)
        i.add_output('shaft_ratio', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS', val=0.10)
        i.add_output('Np', desc='number of planets in each stage', val=np.array([3.0,3.0,1.0,]))
        i.add_output('shrink_disc_mass', units='kg', desc='Mass of the shrink disc', val=sd_mass)
        i.add_output('carrier_mass', units='kg', desc='Carrier mass', val=8000.0)
        i.add_output('flange_length', units='m', desc='flange length', val=0.5)
        i.add_output('overhang', units='m', desc='Overhang distance', val=5.0)
        i.add_output('L_rb', units='m', desc='distance between hub center and upwind main bearing', val=1.912)
        i.add_output('gearbox_cm_x', units = 'm', desc = 'distance from tower-top center to gearbox cm--negative for upwind', val=0.0)
        i.add_output('tower_top_diameter', units='m', desc='diameter of tower top', val=3.78)
        i.add_output('hss_length', units = 'm', desc = 'optional high speed shaft length determined by user')
        
        
        self.add_subsystem('dof', i)   
        self.add_subsystem('nacelle', NacelleAdder())  
        
        self.connect('dof.rotor_diameter', 'nacelle.rotor_diameter')
        self.connect('dof.rotor_speed', 'nacelle.rotor_speed')
        self.connect('dof.machine_rating', 'nacelle.machine_rating')
        self.connect('dof.rotor_torque', 'nacelle.rotor_torque')
        self.connect('dof.rotor_thrust', 'nacelle.rotor_thrust')
        self.connect('dof.rotor_mass', 'nacelle.rotor_mass')
        self.connect('dof.rotor_bending_moment_x', 'nacelle.rotor_bending_moment_x')
        self.connect('dof.rotor_bending_moment_y', 'nacelle.rotor_bending_moment_y')
        self.connect('dof.rotor_bending_moment_z', 'nacelle.rotor_bending_moment_z')
        self.connect('dof.rotor_force_x', 'nacelle.rotor_force_x')
        self.connect('dof.rotor_force_y', 'nacelle.rotor_force_y')
        self.connect('dof.rotor_force_z', 'nacelle.rotor_force_z')
        self.connect('dof.gear_ratio', 'nacelle.gear_ratio')
        self.connect('dof.crane', 'nacelle.crane')
        self.connect('dof.shaft_angle', 'nacelle.shaft_angle')
        self.connect('dof.shaft_ratio', 'nacelle.shaft_ratio')
        self.connect('dof.Np', 'nacelle.Np')
        self.connect('dof.shrink_disc_mass', 'nacelle.shrink_disc_mass')
        self.connect('dof.carrier_mass', 'nacelle.carrier_mass')
        self.connect('dof.flange_length', 'nacelle.flange_length')
        self.connect('dof.overhang', 'nacelle.overhang')
        self.connect('dof.L_rb', 'nacelle.L_rb')
        self.connect('dof.gearbox_cm_x', 'nacelle.gearbox_cm_x')
        self.connect('dof.tower_top_diameter', 'nacelle.tower_top_diameter')
        self.connect('dof.hss_length', 'nacelle.hss_length') 




def HubTest():
    
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(HubSE())
    prob.setup()
    #view_model(prob, outfile='hub.html')
     
    AirDensity= 1.225 # kg/(m^3)
    Solidity  = 0.0517
    RatedWindSpeed = 11.05 # m/s
     
    prob['dof.rotor_diameter'] = 126.0
    prob['dof.blade_number'] = 3
    prob['dof.blade_root_diameter'] = 3.4
    prob['dof.machine_rating'] = 5000.0
    prob['dof.blade_mass'] = 17740.0
    prob['dof.rotor_bending_moment'] = -16665000.0 #(3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (prob['dof.rotor_diameter'] ** 3)) / prob['dof.blade_number']
 
    prob['dof.L_rb'] = 1.91 #inputs['L_rb']
    prob['dof.shaft_angle']     = 5.0 #inputs['shaft_angle']
    prob['dof.MB1_location']     = [-1.70200443,  0. ,         0.84751353] #[-1.03966219,  0.  ,       -0.20292427] #inputs['MB1_location']
     
     
     
     
 
     
    prob.run_model()
     
    print "NREL 5 MW turbine test"
    print "Hub Components"
    print prob['hub.hub.mass']  # 26404.35539331 # 31644.47
    print prob['hub.pitch.mass'] # 14627.41018363 # 17003.98
    print prob['hub.spinner.mass'] # 1329.5 # 1810.50
    print prob['hub.hub_system_mass'] # 42361.26557694
    print prob['hub.hub_diameter']
#     print prob['hub_system_cm']     
#     print prob['hub_system_I']   
#     print prob['hub_system_mass']   
    print 'Done in ' + str(time() - start) + ' seconds'  
    
    
    
def NacelleTest():
    start = time()
    
    # get and set values of the variables using Problem
    prob = Problem(NacelleSE())
    prob.setup()
    #view_model(prob, outfile='nacelle2.html')
    
    prob['dof.rotor_diameter'] = 126.
    prob['dof.rotor_speed'] = 12.1
    prob['dof.machine_rating'] = 5000.
    prob['dof.rotor_torque'] = (1.5*5000*1000/0.95)/(12.1*pi/30) #6230511.04
    prob['dof.rotor_thrust'] = 599610.
    prob['dof.rotor_mass'] = 0.
    prob['dof.rotor_bending_moment_x'] = 330770.
    prob['dof.rotor_bending_moment_y'] = -16665000.
    prob['dof.rotor_bending_moment_z'] = 2896300.
    prob['dof.rotor_force_x'] = 599610.
    prob['dof.rotor_force_y'] = 186780.
    prob['dof.rotor_force_z'] = -842710.
    prob['dof.gear_ratio'] = 96.76
    prob['dof.crane'] = 1
    prob['dof.shaft_angle'] = 5.
    prob['dof.shaft_ratio'] = 0.1
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.shrink_disc_mass'] = 5000.0/3.0
    prob['dof.carrier_mass'] = 8000.0
    prob['dof.flange_length'] = 0.5
    prob['dof.overhang'] = 5.
    prob['dof.L_rb'] = 1.912
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.tower_top_diameter'] = 3.78
    prob['dof.hss_length'] = 1.5
     
     
     
     
 
     
    prob.run_model()
     
    print "NREL 5 MW turbine test"
    print "Nacelle Components"
    print prob['nacelle.nacelle_mass'] # 46331.00166551
    print prob['nacelle.MB1_location'] # [-1.09809735  0.          0.64357787]
    
     
    print 'Done in ' + str(time() - start) + ' seconds'      
            
if __name__ == "__main__":
    #HubTest()
    NacelleTest()
    
       