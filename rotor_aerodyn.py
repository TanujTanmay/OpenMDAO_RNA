from math import pi
from time import time
import os

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model

from fixed_parameters import beautify, num_nodes, wind_shear, \
        aerodyn_folder, aerodyn_exe, root_name, driver_ext, summary_ext, output_ext
from input_generator import set_input_driver, set_primary_input, set_blade_file, read_output      

     
        
        

class RotorAerodyn(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('wind_speed', units = 'm/s', desc = 'wind speed')
        self.add_input('blade_number', desc='number of blades')
        #self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        self.add_input('hub_radius', units = 'm', desc = 'hub radius')
        self.add_input('hub_height', units = 'm', desc = 'hub height')
        self.add_input('overhang', units='m', desc='overhang distance')
        self.add_input('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        self.add_input('precone', units='deg', desc='blade precone angle')
        self.add_input('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        #self.add_input('span_dr', units='m', desc='list of blade annulus thickness', shape=num_nodes)
        self.add_input('span_airfoil', desc='list of blade node Airfoil ID', shape=num_nodes)
        self.add_input('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        self.add_input('span_twist', units='deg',desc='list of blade node twist angle', shape=num_nodes)
        self.add_input('pitch', units = 'deg', desc = 'blade pitch angle')
        self.add_input('yaw', units = 'deg', desc = 'rotor yaw misalignment angle')
        
     
        # outputs
        self.add_output('span_fx', units='N/m', desc='list of spanwise normal force to the plane of rotation', shape=num_nodes)
        self.add_output('span_fy', units='N/m', desc='list of spanwise normal force to the plane of rotation', shape=num_nodes)
        self.add_output('rotor_tsr', desc='rotor tip speed ratio')
        self.add_output('swept_area', units='m**2', desc='rotor swept area')
        self.add_output('rotor_cp', desc='rotor power coefficient')
        self.add_output('rotor_cq', desc='rotor torque coefficient')
        self.add_output('rotor_ct',  desc='rotor thrust coefficient')
        self.add_output('rotor_power', units='W', desc='rotor power')
        self.add_output('rotor_torque', units='N*m', desc='rotor torque')
        self.add_output('rotor_thrust',  units='N', desc='rotor thrust')
        self.add_output('rotor_force', units='N', desc='rotor load vector in hub coordinate system', shape=3)
        self.add_output('rotor_moment',  units='N*m', desc='rotor moment vector in hub coordinate system', shape=3)

 
 
    def compute(self, inputs, outputs):
         
        NumBlades = int(inputs['blade_number'])
        HubRad = float(inputs['hub_radius'])
        HubHt = float(inputs['hub_height'])
        Overhang = float(inputs['overhang'])
        ShftTilt = float(inputs['shaft_angle'])
        Precone = float(inputs['precone'])
        WndSpeed = float(inputs['wind_speed'])
        RotSpd = float(inputs['rotor_speed'])
        Pitch = float(inputs['pitch'])
        Yaw = float(inputs['yaw'])
        
        BlSpn = inputs['span_r']
        BlChord = inputs['span_chord']
        BlTwist = inputs['span_twist']
        BlAFID = [int(x)+1 for x in inputs['span_airfoil']] # because in AeroDyn it starts with 1
        
        ShearExp = wind_shear
        dT = 0.138
        Tmax = 12.7935
        
        # generate AeroDyn input files
        #root_name    = root_name # + dt.datetime.now().strftime('%d_%H_%M')
        driver_file  = root_name + driver_ext
        summary_file = aerodyn_folder + root_name + summary_ext
        output_file  = aerodyn_folder + root_name + output_ext   
            
        set_input_driver(root_name, NumBlades, HubRad, HubHt, Overhang, ShftTilt, Precone, \
                     WndSpeed, ShearExp, RotSpd, Pitch, Yaw, dT, Tmax) 
         
        set_primary_input(root_name, BlAFID)
         
        set_blade_file(root_name, BlSpn, BlTwist, BlChord, BlAFID)

        # execute AeroDyn
        command = './/' + aerodyn_folder + aerodyn_exe + ' ' + aerodyn_folder + driver_file
        command = command.replace('//', '\\') # change folder separator to CMD style (slash -> backslash)
        try:
            os.system(command)
        except IOError:
            print "FATAL ERROR while running Aerodyn"
            quit()
        
        # read AeroDyn output file
        [rotor, spanwise, timeseries] = read_output(summary_file, output_file, BlSpn)    
        
        #outputs['span_force'] = [[x,y] for (x,y) in zip(spanwise['Fx'], spanwise['Fy'])]
        outputs['span_fx'] = spanwise['Fx']
        outputs['span_fy'] = spanwise['Fy']
        outputs['rotor_tsr'] = rotor['RtTSR']
        outputs['swept_area'] = rotor['RtArea']
        outputs['rotor_cp'] = rotor['RtAeroCp']
        outputs['rotor_cq'] = rotor['RtAeroCq']
        outputs['rotor_ct'] = rotor['RtAeroCt']
        outputs['rotor_power'] = rotor['RtAeroPwr']
        outputs['rotor_torque'] = rotor['RtAeroPwr'] / (RotSpd*2*pi/60.0)
        outputs['rotor_thrust'] = rotor['RtAeroCt'] * 0.5 * rho_air * rotor['RtArea'] * WndSpeed**2
        outputs['rotor_force'] =  [rotor['RtAeroFxh'], rotor['RtAeroFyh'], rotor['RtAeroFzh']]
        outputs['rotor_moment'] = [rotor['RtAeroMxh'], rotor['RtAeroMyh'], rotor['RtAeroMzh']]

        


class RotorAerodynamicsTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('wind_speed', units = 'm/s', desc = 'hub height wind speed')
        i.add_output('blade_number', desc='number of blades')
        #i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('rotor_speed', units = 'rpm', desc = 'rotor rotational speed')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('hub_height', units = 'm', desc = 'hub height')
        i.add_output('overhang', units='m', desc='overhang distance')
        i.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        i.add_output('precone', units='deg', desc='blade precone angle')
        i.add_output('span_r', units='m', desc='spanwise radial location of blade junctions', shape=num_nodes)
        #i.add_output('span_dr', units='m', desc='spanwise blade node thickness', shape=num_nodes)
        i.add_output('span_airfoil', desc='list of blade node airfoil ID', shape=num_nodes)
        i.add_output('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        i.add_output('span_twist', units='deg', desc='list of blade node twist angle', shape=num_nodes)
        i.add_output('pitch', units='deg', desc='blade pitch angle')
        i.add_output('yaw', units = 'deg', desc = 'rotor yaw misalignment angle')
#         i.add_output('cut_in_speed', units = 'm/s', desc = 'cut-in wind speed')
#         i.add_output('cut_out_speed', units = 'm/s', desc = 'cut-out wind speed')
#         i.add_output('machine_rating', units='kW', desc='machine rating')
        
        # sub-components         
        self.add_subsystem('dof', i, promotes=['*'])   
        self.add_subsystem('aero', RotorAerodyn(), promotes_inputs=['*'])

        
        
        


if __name__ == "__main__":
    
    start = time()
    
    # workflow setup
    prob = Problem(RotorAerodynamicsTest())
    prob.setup()
    view_model(prob, outfile='N2/rotor_aero_siemens.html')
    
    # define inputs
    prob['dof.wind_speed'] = 9.0 
    prob['dof.blade_number'] = 3
    #prob['dof.rotor_diameter'] = 108.0
    prob['dof.rotor_speed'] = 14.3
    prob['dof.hub_radius'] = 1.5
    prob['dof.hub_height'] = 90.0
    prob['dof.overhang'] = -5.0191
    prob['dof.shaft_angle'] = -5
    prob['dof.precone'] = -2.5
    prob['dof.span_r'] = [ 0, 11.5, 16.5, 21.5, 26.5, 31.5, 36.5, 41.5, 46.5, 51.5]
    #prob['dof.span_dr'] = [5., 5., 5., 5., 5., 5., 5., 5., 5., 5.]
    prob['dof.span_airfoil'] = [0., 7, 7, 7, 7, 7, 7, 7, 7, 7]
    prob['dof.span_chord'] = [3.4, 3.4, 4.2, 3.2, 1.8, 1.5, 1.3, 1.1, 1.0, 0.9]
    prob['dof.span_twist'] = [4.0, 4.0, 12.4, 9.3, 4.1, 2.8, 1.8, 1.1, 0.5, 0.0]
    prob['dof.pitch'] = 0.0
    prob['dof.yaw'] = 0.0
#     prob['dof.cut_in_speed'] = 3.0
#     prob['dof.cut_out_speed'] = 25.0
#     prob['dof.machine_rating'] = 2300.0
    
     
    prob.run_model()
    
    # print outputs 
    print "Rotor Aerodynamics"
    print 'rotor_tsr = ' + beautify(prob['aero.rotor_tsr'])
    print 'swept_area = ' + beautify(prob['aero.swept_area'])
    print 'rotor_cp = ' + str(prob['aero.rotor_cp'])
    print 'rotor_cq = ' + str(prob['aero.rotor_cq'])
    print 'rotor_ct = ' + str(prob['aero.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['aero.rotor_power'])
    print 'rotor_force = ' + beautify(prob['aero.rotor_force'])
    print 'rotor_moment = ' + beautify(prob['aero.rotor_moment'])
    print 'span_fx = ' + str(prob['aero.span_fx'])
  
    print 'Done in ' + str(time() - start) + ' seconds'


 