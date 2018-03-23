from time import time
from math import pi, sin, cos, radians

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model    
from bem import bem_rotor        
from aerodyn_driver import execute_aerodyn
from fixed_parameters import num_nodes, num_bins,  beautify,\
    bem_spanwise_params, rho_air, wind_shear, g 




#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class RotorAerodynamics(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('design_tsr', desc='design tip speed ratio')
        self.add_input('wind_speed', units = 'm/s', desc = 'wind speed at hub height')
        self.add_input('blade_number', desc='number of blades')
        self.add_input('blade_mass', units = 'kg', desc='mass of the one blade')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('hub_height', units = 'm', desc = 'hub height')
        self.add_input('hub_radius', units = 'm', desc = 'hub radius')
        self.add_input('span_r', units='m', desc='list of blade node radial location', shape=num_nodes)
        self.add_input('span_dr', units='m', desc='list of blade annulus thickness', shape=num_nodes)
        self.add_input('span_airfoil', desc='list of blade node Airfoil ID', shape=num_nodes)
        self.add_input('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        self.add_input('span_twist', units='deg',desc='list of blade node twist angle', shape=num_nodes)
        self.add_input('precone', units='deg', desc='blade precone angle')
        self.add_input('pitch', units = 'deg', desc = 'blade pitch angle')
        self.add_input('yaw', units = 'deg', desc = 'rotor yaw misalignment angle')
        self.add_input('overhang', units='m', desc='overhang distance')
        self.add_input('shaft_angle', units='deg', desc='angle of the main shaft inclindation wrt the horizontal')
     
        # outputs        
        self.add_output('rotor_speed', units='rpm', desc='rotor speed')
        self.add_output('swept_area', units='m**2', desc='rotor swept area')
        self.add_output('rotor_cp', desc='rotor power coefficient')
        self.add_output('rotor_cq', desc='rotor torque coefficient')
        self.add_output('rotor_ct',  desc='rotor thrust coefficient')
        self.add_output('rotor_power', units='W', desc='rotor power')
        self.add_output('rotor_torque', units='N*m', desc='rotor torque')
        self.add_output('rotor_thrust',  units='N', desc='rotor thrust')
        self.add_output('rotor_force', units='N', desc='rotor load vector in hub coordinate system', val=[0., 0., 0.])
        self.add_output('rotor_moment',  units='N*m', desc='rotor moment vector in hub coordinate system', val=[0., 0., 0.])
        self.add_output('span_fx', units='N/m', desc='list of spanwise normal force to the plane of rotation', shape=num_nodes)
        self.add_output('span_fy', units='N/m', desc='list of spanwise normal force to the plane of rotation', shape=num_nodes)
  








class AerodynamicsSimple(RotorAerodynamics):
    def compute(self, inputs, outputs):   
        # inputs     
        design_tsr = inputs['design_tsr']
        wind_speed = inputs['wind_speed'] 
        blade_number = inputs['blade_number']
        blade_mass = inputs['blade_mass']
        rotor_diameter = inputs['rotor_diameter']
        
        hub_radius = inputs['hub_radius']
        span_r = inputs['span_r']
        span_dr = inputs['span_dr']
        span_chord = inputs['span_chord']
        span_twist = inputs['span_twist']
        span_airfoil = inputs['span_airfoil']
        pitch = inputs['pitch']
        shaft_angle = radians(abs(inputs['shaft_angle']))
        
        rotor_radius = rotor_diameter/2.0
        rotor_speed = (design_tsr*wind_speed/rotor_radius) * (30/pi) # rpm
        
        [spanwise, rotor] = bem_rotor(wind_speed, blade_number, rotor_radius, hub_radius, design_tsr, pitch, \
                                      span_r, span_dr, span_chord, span_twist, span_airfoil, is_prandtl=1, is_glauert=1)
        

        # rotor forces and moments
        rotor_mass = blade_mass * blade_number
        fx = rotor['thrust'] + rotor_mass * g * sin(shaft_angle)
        fy = 0 # cancels for all blades
        fz = -1 * rotor_mass * g * cos(shaft_angle)        
        mx = rotor['torque']
        my = 0
        mz = 0
        
        # outputs
        outputs['rotor_speed'] = rotor_speed
        outputs['swept_area'] = rotor['swept_area']
        outputs['rotor_cp'] = rotor['cp']
        outputs['rotor_cq'] = rotor['cq']
        outputs['rotor_ct'] = rotor['ct']
        outputs['rotor_power'] = rotor['power']
        outputs['rotor_torque'] = rotor['torque']
        outputs['rotor_thrust'] = rotor['thrust']
        outputs['rotor_force'] = [fx, fy, fz]
        outputs['rotor_moment'] = [mx, my, mz]
        outputs['span_fx'] = spanwise['fx']
        outputs['span_fy'] = spanwise['fy']
        
        





class AerodynamicsAeroDyn(RotorAerodynamics):
    def adjust_inputs(self, span_airfoil, span_r, span_dr):
        '''
            the inputs in AeroDyn are slightly different from the way this model is built
            this function does some adjustments for this
            Note: CamelCase variables are used to make it consistent with AeroDyn format
        '''
        
        # AirfoilIDs start with 1 in AeroDyn
        BlAFID = [int(x)+1 for x in span_airfoil] 
        
        # AeroDyn: the first node should be at the blade root
        # MDAO: the first node is at the centre of the first blade section
        span_r[0] = span_r[0] - span_dr[0]/2.0
        HubRad = span_r[0]
        
        # AeroDyn: the last node should be at the blade tip
        # MDAO: the last node is at the centre of the last blade section
        span_r[-1] = span_r[-1] + span_dr[-1]/2.0
        RotRad = span_r[-1]
        
        # AeroDyn: blade root starts at r=0
        # MDAO: blade root starts at blade root
        BlSpn = [x - HubRad for x in span_r]

        return [BlAFID, BlSpn, RotRad]
        
    
    
    def compute(self, inputs, outputs):
        # inputs 
        design_tsr = float(inputs['design_tsr'])
        NumBlades = int(inputs['blade_number'])
        HubRad = float(inputs['hub_radius'])
        HubHt = float(inputs['hub_height'])
        Overhang = float(inputs['overhang'])
        ShftTilt = float(inputs['shaft_angle'])
        Precone = float(inputs['precone'])
        WndSpeed = float(inputs['wind_speed'])
        Pitch = float(inputs['pitch'])
        Yaw = float(inputs['yaw'])
        span_r = inputs['span_r']
        span_dr = inputs['span_dr']
        span_airfoil = inputs['span_airfoil']
        BlChord = inputs['span_chord']
        BlTwist = inputs['span_twist']
        
        
        # adjust some inputs for AeroDyn
        [BlAFID, BlSpn, RotRad] = self.adjust_inputs(span_airfoil, span_r, span_dr)
        RotSpd = (design_tsr*WndSpeed/RotRad) * (30/pi) # rpm
        
        # execute AeroDyn
        [rotor, spanwise, timeseries] = execute_aerodyn(NumBlades, HubRad, HubHt, Overhang, ShftTilt, Precone, \
                                                        WndSpeed, wind_shear, RotSpd, Pitch, Yaw, \
                                                        BlSpn, BlTwist, BlChord, BlAFID)   
        
        # outputs
        outputs['rotor_speed'] = RotSpd
        outputs['swept_area'] = rotor['RtArea']
        outputs['rotor_cp'] = rotor['RtAeroCp']
        outputs['rotor_cq'] = rotor['RtAeroCq']
        outputs['rotor_ct'] = rotor['RtAeroCt']
        outputs['rotor_power'] = rotor['RtAeroPwr']
        outputs['rotor_torque'] = rotor['RtAeroPwr'] / (RotSpd*2*pi/60.0)
        outputs['rotor_thrust'] = rotor['RtAeroCt'] * 0.5 * rho_air * rotor['RtArea'] * WndSpeed**2
        outputs['rotor_force'] =  [rotor['RtAeroFxh'], rotor['RtAeroFyh'], rotor['RtAeroFzh']]
        outputs['rotor_moment'] = [rotor['RtAeroMxh'], rotor['RtAeroMyh'], rotor['RtAeroMzh']]
        outputs['span_fx'] = spanwise['Fx']
        outputs['span_fy'] = spanwise['Fy']
        
        
        

        
        
        
        
           
        





#############################################################################
##############################  UNIT TESTING ################################
#############################################################################         
class RotorAerodynamicsTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('design_tsr', desc='design tip speed ratio')
        i.add_output('wind_speed', units = 'm/s', desc = 'hub height wind speed')
        i.add_output('blade_number', desc='number of blades')
        i.add_output('blade_mass', units = 'kg', desc='mass of the one blade')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('hub_height', units = 'm', desc = 'hub height')
        i.add_output('hub_radius', units = 'm', desc = 'hub radius')
        i.add_output('span_r', units='m', desc='spanwise radial location of blade junctions', shape=num_nodes)
        i.add_output('span_dr', units='m', desc='spanwise blade node thickness', shape=num_nodes)
        i.add_output('span_airfoil', desc='list of blade node airfoil ID', shape=num_nodes)
        i.add_output('span_chord', units='m', desc='list of blade node chord length', shape=num_nodes)
        i.add_output('span_twist', units='deg', desc='list of blade node twist angle', shape=num_nodes)
        i.add_output('precone', units='deg', desc='blade precone angle')
        i.add_output('pitch', units='deg', desc='blade pitch angle')
        i.add_output('yaw', units = 'deg', desc = 'rotor yaw misalignment angle')
        i.add_output('overhang', units='m', desc='overhang distance')
        i.add_output('shaft_angle', units='deg', desc='angle of the LSS inclindation with respect to the horizontal')
        
        # sub-components         
        self.add_subsystem('dof', i, promotes=['*'])   
        self.add_subsystem('aero', AerodynamicsAeroDyn(), promotes_inputs=['*'])
        
        
        


if __name__ == "__main__":
    start = time()
    
    # workflow setup
    prob = Problem(RotorAerodynamicsTest())
    prob.setup()
    view_model(prob, outfile='N2/rotor_aero.html')
    
    # define inputs
    span_r = [3.0375, 6.1125, 9.1875, 12.2625, 15.3375, 18.4125, 21.4875, 24.5625, 27.6375, 30.7125, 33.7875, 36.8625, 39.9375, 43.0125, 46.0875, 49.1625, 52.2375, 55.3125, 58.3875, 61.4625]
    span_dr = [3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075, 3.075]
    span_airfoil = [0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0, 4.0, 5.0, 5.0, 5.0, 6.0, 6.0, 6.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0]
    span_chord = [3.5615, 3.9127, 4.2766, 4.5753, 4.6484, 4.5489, 4.3819, 4.2206, 4.0382, 3.8449, 3.6549, 3.4713, 3.2868, 3.1022, 2.9178, 2.7332, 2.5487, 2.3691, 2.1346, 1.4683]
    span_twist = [13.235, 13.235, 13.235, 13.1066, 11.6516, 10.5523, 9.6506, 8.7896, 7.876, 6.937, 6.0226, 5.1396, 4.2562, 3.428, 2.735, 2.1466, 1.5521, 0.9525, 0.3813, 0.0477]

    prob['dof.design_tsr'] = 7.0
    prob['dof.wind_speed'] = 12.2
    prob['dof.blade_number'] = 3
    prob['dof.blade_mass'] = 17030.8561
    prob['dof.rotor_diameter'] = 126.0
    prob['dof.hub_height'] = 90.0
    prob['dof.hub_radius'] = 1.5
    prob['dof.span_r'] = span_r
    prob['dof.span_dr'] = span_dr
    prob['dof.span_airfoil'] = span_airfoil
    prob['dof.span_chord'] = span_chord
    prob['dof.span_twist'] = span_twist
    prob['dof.precone'] = -2.5
    prob['dof.pitch'] = 0.0
    prob['dof.yaw'] = 0.0
    prob['dof.overhang'] = -5.0191
    prob['dof.shaft_angle'] = -5.0
    
     
    prob.run_model()
    
    # print outputs 
    print "Rotor Aerodynamics"
    print 'rotor_speed = ' + beautify(prob['aero.rotor_speed'])
    print 'swept_area = ' + beautify(prob['aero.swept_area'])
    print 'rotor_cp = ' + str(prob['aero.rotor_cp'])
    print 'rotor_cq = ' + str(prob['aero.rotor_cq'])
    print 'rotor_ct = ' + str(prob['aero.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['aero.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['aero.rotor_torque'])
    print 'rotor_thrust = ' + beautify(prob['aero.rotor_thrust'])
    print 'rotor_force = ' + beautify(prob['aero.rotor_force'])
    print 'rotor_moment = ' + beautify(prob['aero.rotor_moment'])
    print 'span_fx = ' + beautify(prob['aero.span_fx'])
    print 'span_fy = ' + beautify(prob['aero.span_fy'])
  
    print 'Done in ' + str(time() - start) + ' seconds'    
     
    


 