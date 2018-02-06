from openmdao.api import ExplicitComponent, Group, IndepVarComp
from input_generator import *

import numpy as np
from time import time, clock
from math import pi, cos, sqrt, radians, sin, exp, log10, log, floor, ceil


num_airfoils = 2
num_nodes = 6

class RotorAero(ExplicitComponent):
    def setup(self):
        # variables
        self.add_input('NumBlades', desc='number of blades')
        self.add_input('HubRad', units = 'm', desc = 'hub radius')
        self.add_input('HubHt', units = 'm', desc = 'hub radius')
        self.add_input('Overhang', units = 'm', desc = 'overhang')
        self.add_input('ShftTilt', units = 'deg', desc = 'shaft tilt')
        self.add_input('Precone', units = 'deg', desc = 'blade precone')
        
        self.add_input('WndSpeed', units = 'm/s', desc = 'hub radius')
        self.add_input('ShearExp', desc = 'hub radius')
        self.add_input('RotSpd', units = 'rpm', desc = 'rotor rotational speed')
        self.add_input('Pitch', units = 'deg', desc = 'blade pitch angle')
        self.add_input('Yaw', units = 'deg', desc = 'yaw angle')
        self.add_input('dT', units = 's', desc = 'time step of simulation')
        self.add_input('Tmax', units = 's', desc = 'duration of simulation')
        
        self.add_input('Airfoils', desc='list of airfoil names with extension (assumed to be in "Airfoils" folder)', shape=num_airfoils)
        self.add_input('BlSpn', units='m', desc='list of blade node radial location', shape=num_nodes)
        self.add_input('BlChord', units='m', desc='list of blade node chord length', shape=num_nodes)
        self.add_input('BlTwist', units='deg',desc='list of blade node twist angle', shape=num_nodes)
        self.add_input('BlAFID', desc='list of blade node Airfoil ID', shape=num_nodes)
     
        # outputs
        self.add_output('RtAeroCp', desc='rotor Cp')
        self.add_output('RtAeroCt', desc='rotor Ct')
        self.add_output('RtTSR', desc='rotor tip speed ratio')
        self.add_output('B1N1Cl', desc='time series of lift coefficient at blade root')
        self.add_output('B1N1Cd', desc='time series of drag coefficient at blade root')
        self.add_output('SpanwiseCl', desc='Cl at each node')
        self.add_output('SpanwiseCd', desc='Cd at each node')
        self.add_output('SpanwiseAlpha', desc='angle of attack at each node')
        self.add_output('SpanwisePhi', desc='inflow angle at each node')
        self.add_output('SpanwiseAxInd', desc='axial induction at each node')
        self.add_output('SpanwiseTnInd', desc='tangential induction at each node')
 
 
    def compute(self, inputs, outputs):
         
        NumBlades = inputs['NumBlades']
        HubRad = inputs['HubRad']
        HubHt = inputs['HubHt']
        Overhang = inputs['Overhang']
        ShftTilt = inputs['ShftTilt']
        Precone = inputs['Precone']
        WndSpeed = inputs['WndSpeed']
        ShearExp = inputs['ShearExp']
        RotSpd = inputs['RotSpd']
        Pitch = inputs['Pitch']
        Yaw = inputs['Yaw']
        dT = inputs['dT']
        Tmax = inputs['Tmax']
        Airfoils = inputs['Airfoils']
        BlSpn = inputs['BlSpn']
        BlChord = inputs['BlChord']
        BlTwist = inputs['BlTwist']
        BlAFID = inputs['BlAFID']
        
        root_name = 'test'         
        set_input_driver(root_name, NumBlades, HubRad, HubHt, Overhang, ShftTilt, Precone, \
                     WndSpeed, ShearExp, RotSpd, Pitch, Yaw, dT, Tmax) 
        
        set_primary_input(root_name, Airfoils)
        
        set_blade_file(root_name, BlSpn, BlTwist, BlChord, BlAFID)



class RotorGroup(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # variables
        i.add_output('NumBlades', desc='number of blades')
        i.add_output('HubRad', units = 'm', desc = 'hub radius')
        i.add_output('HubHt', units = 'm', desc = 'hub radius')
        i.add_output('Overhang', units = 'm', desc = 'overhang')
        i.add_output('ShftTilt', units = 'deg', desc = 'shaft tilt')
        i.add_output('Precone', units = 'deg', desc = 'blade precone')
        
        i.add_output('WndSpeed', units = 'm/s', desc = 'hub radius')
        i.add_output('ShearExp', desc = 'hub radius')
        i.add_output('RotSpd', units = 'rpm', desc = 'rotor rotational speed')
        i.add_output('Pitch', units = 'deg', desc = 'blade pitch angle')
        i.add_output('Yaw', units = 'deg', desc = 'yaw angle')
        i.add_output('dT', units = 's', desc = 'time step of simulation')
        i.add_output('Tmax', units = 's', desc = 'duration of simulation')
        
        i.add_output('Airfoils', desc='tuple of airfoil names with extension (assumed to be in "Airfoils" folder)', shape=num_airfoils)
        i.add_output('BlSpn', units='m', desc='tuple of blade node radial location', shape=num_nodes)
        i.add_output('BlChord', units='m', desc='tuple of blade node chord length', shape=num_nodes)
        i.add_output('BlTwist', units='deg',desc='tuple of blade node twist angle', shape=num_nodes)
        i.add_output('BlAFID', desc='tuple of blade node Airfoil ID', shape=num_nodes)
                
        
               
        # parameters
        self.add_subsystem('dof', i)        
        self.add_subsystem('aero', RotorAero())
        
        self.connect('dof.NumBlades', 'aero.NumBlades')
        self.connect('dof.HubRad', 'aero.HubRad')
        self.connect('dof.HubHt', 'aero.HubHt')
        self.connect('dof.Overhang', 'aero.Overhang')
        self.connect('dof.ShftTilt', 'aero.ShftTilt')
        self.connect('dof.Precone', 'aero.Precone')
        self.connect('dof.WndSpeed', 'aero.WndSpeed')
        self.connect('dof.ShearExp', 'aero.ShearExp')
        self.connect('dof.RotSpd', 'aero.RotSpd')
        self.connect('dof.Pitch', 'aero.Pitch')
        self.connect('dof.Yaw', 'aero.Yaw')
        self.connect('dof.dT', 'aero.dT')
        self.connect('dof.Tmax', 'aero.Tmax')
        self.connect('dof.Airfoils', 'aero.Airfoils')
        self.connect('dof.BlSpn', 'aero.BlSpn')
        self.connect('dof.BlChord', 'aero.BlChord')
        self.connect('dof.BlTwist', 'aero.BlTwist')
        self.connect('dof.BlAFID', 'aero.BlAFID')
        
        


if __name__ == "__main__":
    
    from openmdao.api import Problem, view_model
    
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RotorGroup())
    prob.setup()
    view_model(prob, outfile='rotor_aero.html')
    
    prob['dof.NumBlades'] = 3
    prob['dof.HubRad'] = 1.5
    prob['dof.HubHt'] = 90.0
    prob['dof.Overhang'] = -5.0
    prob['dof.ShftTilt'] = -5.0
    prob['dof.Precone'] = 0.0
    prob['dof.WndSpeed'] = [10.0]
    prob['dof.ShearExp'] = [0.0]
    prob['dof.RotSpd'] = [19.1]
    prob['dof.Pitch'] = [0.0]
    prob['dof.Yaw'] = [0.0]
    prob['dof.dT'] = [0.5]
    prob['dof.Tmax'] = [10.0]
    prob['dof.Airfoils'] = ('cyclinder.dat', 'ah.csv')
    prob['dof.BlSpn'] = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    prob['dof.BlChord'] = [3.4, 3.4, 1.9, 1.3, 1.0, 0.8]
    prob['dof.BlTwist'] = [0.0, 5.9, -2.3, -5.5, -7.0, -7.9]
    prob['dof.BlAFID'] = [1, 2, 2, 2, 2, 2]
    
    
    prob.run_model()
    
    print "My Rotor"
    print prob['aero.RtAeroCp']  # 31644.47
    print prob['aero.RtTSR'] # 17003.98
    print prob['aero.SpanwiseAxInd'] # 1810.50
    print prob['aero.SpanwiseAlpha'] 

 
    print(time() - start, "seconds", clock())
    
            