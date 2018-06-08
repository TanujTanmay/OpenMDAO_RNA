from math import pi
from openmdao.api import ExplicitComponent 


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class Hub(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('blade_root_diameter', units='m', desc='blade root diameter')
        self.add_input('machine_rating', units = 'kW', desc = 'machine rating of turbine')
        self.add_input('blade_number', desc='number of turbine blades')
    
        # outputs
        self.add_output('diameter', units='m', desc='hub diameter')
        self.add_output('thickness', units='m',desc='hub thickness')
        self.add_output('mass', units='kg', desc='overall component mass')
        
        

#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class DriveSE(Hub):
    def compute(self, inputs, outputs):
        # inputs    
        self.blade_root_diameter = inputs['blade_root_diameter']
        self.machine_rating = inputs['machine_rating']
        self.blade_number = inputs['blade_number'] 
        
        if self.blade_root_diameter > 0.0: #added 8/6/14 to allow analysis of hubs for unknown blade roots.
            blade_root_diameter = self.blade_root_diameter
        else:
            blade_root_diameter = 2.659*self.machine_rating**.3254

        #Model hub as a cyclinder with holes for blade root and nacelle flange.
        rCyl=1.1*blade_root_diameter/2.0
        hCyl=2.8*blade_root_diameter/2.0
        castThickness = rCyl/10.0
        approxCylVol=2*pi*rCyl*castThickness*hCyl
        bladeRootVol=pi*(blade_root_diameter/2.0)**2*castThickness

        #assume nacelle flange opening is similar to blade root opening
        approxCylNetVol = approxCylVol - (1.0 + self.blade_number)*bladeRootVol
        castDensity = 7200.0 # kg/m^3
        self.mass=approxCylNetVol*castDensity

        # calculate mass properties
        self.diameter=2*rCyl
        self.thickness=castThickness
        
        # outputs
        outputs['diameter'] = self.diameter
        outputs['thickness'] = self.thickness
        outputs['mass'] = self.mass
           


#############################################################################
#############################  UNIT TESTING #################################
#############################################################################  
if __name__ == "__main__":
    inputs = {'blade_root_diameter' : 3.542, \
              'machine_rating' : 5000.0, \
              'blade_number' : 3}
    outputs={}
    DriveSE().compute(inputs, outputs)
    print outputs
        