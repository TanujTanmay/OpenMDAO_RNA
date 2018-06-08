import numpy as np
from time import time
from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model 

from fixed_parameters import uptower_transformer, safety_factor, beautify
from drivese_utils import setup_Bedplate, characterize_Bedplate_Rear, \
                        setup_Bedplate_Front, characterize_Bedplate_Front, size_Bedplate


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class Bedplate(ExplicitComponent):
    def setup(self):
        # inputs
        self.add_input('gbx_length', units = 'm', desc = 'gearbox length')
        self.add_input('gbx_location', units = 'm', desc = 'gearbox CM location')
        self.add_input('gbx_mass', units = 'kg', desc = 'gearbox mass')
        self.add_input('hss_location', units = 'm', desc='HSS CM location')
        self.add_input('hss_mass', units = 'kg', desc='HSS mass')
        self.add_input('generator_location', units = 'm', desc='generator CM location')
        self.add_input('generator_mass', units = 'kg', desc='generator mass')
        self.add_input('lss_location', units = 'm', desc='LSS CM location')
        self.add_input('lss_mass', units = 'kg', desc='LSS mass')
        self.add_input('lss_length', units = 'm', desc = 'LSS length')
        self.add_input('mb1_location', units = 'm', desc='Upwind main bearing CM location')
        self.add_input('FW_mb1', units = 'm', desc = 'Upwind main bearing facewidth')
        self.add_input('mb1_mass', units = 'kg', desc='Upwind main bearing mass')
        self.add_input('mb2_location', units = 'm', desc='Downwind main bearing CM location', val=0.0)
        self.add_input('mb2_mass', units = 'kg', desc='Downwind main bearing mass', val=0.0)
        self.add_input('tower_top_diameter', units = 'm', desc='diameter of the top tower section at the yaw gear')
        self.add_input('rotor_diameter', units = 'm', desc='rotor diameter')
        self.add_input('machine_rating', units='kW', desc='machine_rating machine rating of the turbine')
        self.add_input('rotor_mass', units='kg', desc='rotor mass')
        self.add_input('rotor_bending_moment', units='N*m', desc='The bending moment about the y axis', shape=3)
        self.add_input('rotor_force', units='N', desc='The force along the z axis applied at hub center', shape=3)
        #self.add_input('flange_length', units='m', desc='flange length')
        #self.add_input('L_rb', units = 'm', desc = 'length between rotor center and upwind main bearing')
        self.add_input('overhang', units='m', desc='Overhang distance')
        self.add_input('transformer_mass', units = 'kg', desc='Transformer mass')
        self.add_input('transformer_location', units = 'm', desc = 'transformer CM location')
    
        # outputs
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)    
        self.add_output('length', units='m', desc='length of bedplate')
        self.add_output('height', units='m', desc='max height of bedplate')
        self.add_output('width', units='m', desc='width of bedplate')
         
        




#############################################################################
#############################  MODEL#1: DriveSE #############################
#############################################################################        
class DriveSE(Bedplate):
    def compute(self, inputs, outputs):
        # inputs
        self.gbx_length = inputs['gbx_length']
        self.gbx_location = inputs['gbx_location']
        self.gbx_mass = inputs['gbx_mass']
        self.hss_location = inputs['hss_location']
        self.hss_mass = inputs['hss_mass']
        self.generator_location = inputs['generator_location']
        self.generator_mass = inputs['generator_mass']
        self.lss_location = inputs['lss_location']
        self.lss_mass = inputs['lss_mass']
        self.lss_length = inputs['lss_length']
        self.mb1_location = inputs['mb1_location']
        self.FW_mb1 = inputs['FW_mb1']
        self.mb1_mass = inputs['mb1_mass']
        self.mb2_location = inputs['mb2_location']
        self.mb2_mass = inputs['mb2_mass']
        self.tower_top_diameter = inputs['tower_top_diameter']
        self.rotor_diameter = inputs['rotor_diameter']
        self.machine_rating = inputs['machine_rating']
        self.rotor_mass = inputs['rotor_mass']
        self.rotor_bending_moment_y = inputs['rotor_bending_moment'][1]*safety_factor
        self.rotor_force_z = inputs['rotor_force'][2]*safety_factor
        self.flange_length = 0.0
        self.L_rb = 0.0
        self.overhang = inputs['overhang']
        self.transformer_mass = inputs['transformer_mass']
        self.transformer_location = inputs['transformer_location']
        self.uptower_transformer = uptower_transformer
        #self.transformer_mass = 0.0
        #self.transformer_location = np.zeros(3)
        
        
        setup_Bedplate(self)
        counter = 0
        while self.rootStress*self.stress_mult - self.stressMax >  self.stressTol or self.totalTipDefl - self.deflMax >  self.deflTol:

            counter += 1
            
            characterize_Bedplate_Rear(self)
            
            self.tf += 0.002 
            self.tw += 0.002
            self.b0 += 0.006
            self.h0 += 0.006
            rearCounter = counter

        self.rearHeight = self.h0

        #Front cast section:
        setup_Bedplate_Front(self)

        counter = 0

        while self.rootStress*self.stress_mult - self.stressMax >  self.stressTol or self.totalTipDefl - self.deflMax >  self.deflTol:
            counter += 1
            characterize_Bedplate_Front(self)
            self.tf += 0.002 
            self.tw += 0.002
            self.b0 += 0.006
            self.h0 += 0.006
            
            frontCounter=counter

        size_Bedplate(self)
        
        
        
        # outputs
        outputs['mass'] = self.mass
        outputs['cm'] = self.cm
        outputs['I'] = self.I
        outputs['length'] = self.length
        outputs['height'] = self.height
        outputs['width'] = self.width
        
       
        
        
        


        
        



#############################################################################
##############################  UNIT TESTING ################################
#############################################################################         
class ComponentTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('gbx_length', units = 'm', desc = 'gearbox length')
        i.add_output('gbx_location', units = 'm', desc = 'gearbox CM location')
        i.add_output('gbx_mass', units = 'kg', desc = 'gearbox mass')
        i.add_output('hss_location', units = 'm', desc='HSS CM location')
        i.add_output('hss_mass', units = 'kg', desc='HSS mass')
        i.add_output('generator_location', units = 'm', desc='generator CM location')
        i.add_output('generator_mass', units = 'kg', desc='generator mass')
        i.add_output('lss_location', units = 'm', desc='LSS CM location')
        i.add_output('lss_mass', units = 'kg', desc='LSS mass')
        i.add_output('lss_length', units = 'm', desc = 'LSS length')
        i.add_output('mb1_location', units = 'm', desc='Upwind main bearing CM location')
        i.add_output('FW_mb1', units = 'm', desc = 'Upwind main bearing facewidth')
        i.add_output('mb1_mass', units = 'kg', desc='Upwind main bearing mass')
        i.add_output('mb2_location', units = 'm', desc='Downwind main bearing CM location')
        i.add_output('mb2_mass', units = 'kg', desc='Downwind main bearing mass')
        i.add_output('tower_top_diameter', units = 'm', desc='diameter of the top tower section at the yaw gear')
        i.add_output('rotor_diameter', units = 'm', desc='rotor diameter')
        i.add_output('machine_rating', units='kW', desc='machine_rating machine rating of the turbine')
        i.add_output('rotor_mass', units='kg', desc='rotor mass')
        i.add_output('rotor_bending_moment', units='N*m', desc='The bending moment about the y axis', shape=3)
        i.add_output('rotor_force', units='N', desc='The force along the z axis applied at hub center', shape=3)
        i.add_output('flange_length', units='m', desc='flange length')
        i.add_output('L_rb', units = 'm', desc = 'length between rotor center and upwind main bearing')
        i.add_output('overhang', units='m', desc='Overhang distance')
        
        # sub-components         
        self.add_subsystem('dof', i, promotes=['*'])   
        self.add_subsystem('sys', DriveSE(), promotes_inputs=['*'])
        
        
    def test5(self):
        start = time()
    
        # workflow setup
        prob = Problem(ComponentTest())
        prob.setup()
        #view_model(prob, outfile='lss.html')
        
        # define inputs
        prob['dof.gbx_length'] = 0. #1.512
        prob['dof.gbx_location'] = 0. #0.1
        prob['dof.gbx_mass'] = 0. #58594.4943
        prob['dof.hss_location'] = 0. #[1.606] #, 0.   , 1.134]
        prob['dof.hss_mass'] = 0. #2414.6772
        prob['dof.generator_location'] = [4.057] #, 0.   , 1.134]
        prob['dof.generator_mass'] = 16699.8513
        prob['dof.lss_location'] = [-1.1633] #,  0.    , -0.959 ]
        prob['dof.lss_mass'] = 16873.6459
        prob['dof.lss_length'] = 3.053
        prob['dof.mb1_location'] = [-1.3356] #,  0.    , -1.5413]
        prob['dof.FW_mb1'] = 0.3147
        prob['dof.mb1_mass'] = 6489.0463
        prob['dof.mb2_location'] = [-0.6831] #,  0.    ,  0.6643]
        prob['dof.mb2_mass'] = 1847.7989
        prob['dof.tower_top_diameter'] = 3.78
        prob['dof.rotor_diameter'] = 126.0
        prob['dof.machine_rating'] = 5000.0
        prob['dof.rotor_mass'] = 53220.0
        prob['dof.rotor_bending_moment'] = [0, -16665000.0, 0]
        prob['dof.rotor_force'] = [0, 0, -842710.0] 
        prob['dof.flange_length'] = 0.5
        prob['dof.L_rb'] = 1.912
        prob['dof.overhang'] = 5.0
         
        prob.run_model()
        
        # print outputs 
        print "DriveSE BedPlate"
        beautify('mass', prob['sys.mass'])
        beautify('cm', prob['sys.cm'])
        beautify('I', prob['sys.I'])
        beautify('length', prob['sys.length'])
        beautify('height', prob['sys.height'])
        beautify('width', prob['sys.width'])
      
        print 'Done in ' + str(time() - start) + ' seconds'   


if __name__ == "__main__":
    ComponentTest().test5()
    #ComponentTest().test1_5()
    #print 0.3*(126.0/100.0)**2.0 - 0.1 * (126.0 / 100.0) + 0.4
    #print get_L_rb(126.0, False)
    #print get_My(51092.0,1.95)
    #print get_Mz(51092.0,1.95)
        