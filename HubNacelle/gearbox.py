import numpy as np
import scipy.optimize as opt
from time import time
from math import pi
from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model 

from fixed_parameters import safety_factor, gearbox_stages, gear_configuration, ratio_type, shaft_type, beautify


#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class Gearbox(ExplicitComponent):
    def setup(self):
        #inputs
        self.add_input('gear_ratio', desc='overall gearbox speedup ratio')
        self.add_input('Np', desc='number of planets in each stage', shape=gearbox_stages)
        self.add_input('rotor_speed', units='rpm', desc='rotor rpm at rated power')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_torque', units='N*m', desc='rotor torque at rated power')
        self.add_input('gearbox_cm_x', units='m', desc ='gearbox position along x-axis')
    
        # outputs
        self.add_output('stage_masses', units='kg', desc='individual gearbox stage masses', shape=gearbox_stages)
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)    
        self.add_output('length', units='m', desc='gearbox length')
        self.add_output('height', units='m', desc='gearbox height')
        self.add_output('diameter', units='m', desc='gearbox diameter')
        self.add_output('efficiency', desc='gearbox transmission efficiency')
        
        




#############################################################################
##############################  MODEL#1: DriveSE ############################
#############################################################################        
class DriveSE(Gearbox):
    def compute(self, inputs, outputs):

        self.gear_ratio = inputs['gear_ratio']
        self.Np = inputs['Np']
        self.rotor_speed = inputs['rotor_speed']
        self.rotor_diameter = inputs['rotor_diameter']
        self.rotor_torque = inputs['rotor_torque']*safety_factor
        self.cm_input = inputs['gearbox_cm_x']
        
        self.gear_configuration=gear_configuration
        self.ratio_type=ratio_type
        self.shaft_type=shaft_type


        self.stageRatio=np.zeros(gearbox_stages)
        self.stageTorque = np.zeros(gearbox_stages) #filled in when ebxWeightEst is called
        self.stageMass = np.zeros(gearbox_stages) #filled in when ebxWeightEst is called
        self.stageType=self.stageTypeCalc(self.gear_configuration)
        #print self.gear_ratio
        #print self.Np
        #print self.ratio_type
        #print self.gear_configuration
        self.stageRatio=self.stageRatioCalc(self.gear_ratio,self.Np,self.ratio_type,self.gear_configuration)
        #print self.stageRatio

        m=self.gbxWeightEst(self.gear_configuration,self.gear_ratio,self.Np,self.ratio_type,self.shaft_type,self.rotor_torque)
        self.mass = float(m)
        self.stage_masses=self.stageMass
        # calculate mass properties

        self.length = (0.012 * self.rotor_diameter)
        self.height = (0.015 * self.rotor_diameter)
        self.diameter = (0.75 * self.height)

        cm0   = self.cm_input
        cm1   = 0.0
        cm2   = 0.4*self.height #TODO validate or adjust factor. origin is modified to be above bedplate top
        self.cm = np.array([cm0, cm1, cm2])

        I0 = self.mass * (self.diameter ** 2 ) / 8 + (self.mass / 2) * (self.height ** 2) / 8
        I1 = self.mass * (0.5 * (self.diameter ** 2) + (2 / 3) * (self.length ** 2) + 0.25 * (self.height ** 2)) / 8
        I2 = I1
        self.I = np.array([I0, I1, I2])
        
        outputs['stage_masses'] = self.stage_masses #np.reshape(self.stage_masses, 3)
        outputs['mass'] = self.mass
        outputs['cm'] = self.cm
        outputs['I'] = np.reshape(self.I, 3)
        outputs['length'] = self.length
        outputs['height'] = self.height
        outputs['diameter'] = self.diameter
        


    def stageTypeCalc(self, config):
        temp=[]
        for character in config:
                if character == 'e':
                    temp.append(2)
                if character == 'p':
                    temp.append(1)
        return temp

    def stageMassCalc(self, indStageRatio,indNp,indStageType):

        '''
        Computes the mass of an individual gearbox stage.

        Parameters
        ----------
        indStageRatio : str
          Speedup ratio of the individual stage in question.
        indNp : int
          Number of planets for the individual stage.
        indStageType : int
          Type of gear.  Use '1' for parallel and '2' for epicyclic.
        '''

        #Application factor to include ring/housing/carrier weight
        Kr=0.4
        Kgamma=1.1

        if indNp == 3:
            Kgamma=1.1
        elif indNp == 4:
            Kgamma=1.1
        elif indNp == 5:
            Kgamma=1.35

        if indStageType == 1:
            indStageMass=1.0+indStageRatio+indStageRatio**2+(1.0/indStageRatio)

        elif indStageType == 2:
            sunRatio=0.5*indStageRatio - 1.0
            indStageMass=Kgamma*((1/indNp)+(1/(indNp*sunRatio))+sunRatio+sunRatio**2+Kr*((indStageRatio-1)**2)/indNp+Kr*((indStageRatio-1)**2)/(indNp*sunRatio))

        return indStageMass
        
    def gbxWeightEst(self, config,overallRatio,Np,ratio_type,shaft_type,torque):


        '''
        Computes the gearbox weight based on a surface durability criteria.
        '''

        ## Define Application Factors ##
        #Application factor for weight estimate
        Ka=0.6
        Kshaft=0.0
        Kfact=0.0

        #K factor for pitting analysis
        if self.rotor_torque < 200000.0:
            Kfact = 850.0
        elif self.rotor_torque < 700000.0:
            Kfact = 950.0
        else:
            Kfact = 1100.0

        #Unit conversion from Nm to inlb and vice-versa
        Kunit=8.029

        # Shaft length factor
        if self.shaft_type == 'normal':
            # 4-point suspension
            Kshaft = 1.0
        elif self.shaft_type == 'short':
            # 3-point suspension
            Kshaft = 1.25

        #Individual stage torques
        torqueTemp=self.rotor_torque
        for s in range(len(self.stageRatio)):
            #print torqueTemp
            #print self.stageRatio[s]
            self.stageTorque[s]=torqueTemp/self.stageRatio[s]
            torqueTemp=self.stageTorque[s]
            self.stageMass[s]=Kunit*Ka/Kfact*self.stageTorque[s]*self.stageMassCalc(self.stageRatio[s],self.Np[s],self.stageType[s])
        
        gbxWeight=(sum(self.stageMass))*Kshaft
        
        return gbxWeight

    def stageRatioCalc(self, overallRatio,Np,ratio_type,config):
        '''
        Calculates individual stage ratios using either empirical relationships from the Sunderland model or a SciPy constrained optimization routine.
        '''

        K_r=0
                    
        #Assumes we can model everything w/Sunderland model to estimate speed ratio
        if ratio_type == 'empirical':
            if config == 'p': 
                x=[overallRatio]
            if config == 'e':
                x=[overallRatio]
            elif config == 'pp':
                x=[overallRatio**0.5,overallRatio**0.5]
            elif config == 'ep':
                x=[overallRatio/2.5,2.5]
            elif config =='ee':
                x=[overallRatio**0.5,overallRatio**0.5]
            elif config == 'eep':
                x=[(overallRatio/3)**0.5,(overallRatio/3)**0.5,3]
            elif config == 'epp':
                x=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
            elif config == 'eee':
                x=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
            elif config == 'ppp':
                x=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
        
        elif ratio_type == 'optimal':
            x=np.zeros([3,1])

            if config == 'eep':
                x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                B_1=Np[0]
                B_2=Np[1]
                K_r1=0
                K_r2=0 #2nd stage structure weight coefficient

                def volume(x):
                    return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+ \
                    (x[0]/2.0-1)**2+K_r1*((x[0]-1.0)**2)/B_1 + K_r1*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + \
                    (1.0/(x[0]*x[1]))*((1.0/B_2)+(1/(B_2*((x[1]/2.0)-1.0)))+(x[1]/2.0-1.0)+(x[1]/2.0-1.0)**2.0+K_r2*((x[1]-1.0)**2.0)/B_2 + \
                     K_r2*((x[1]-1.0)**2.0)/(B_2*(x[1]/2.0-1.0))) + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                
                def constr1(x,overallRatio):
                    return x[0]*x[1]*x[2]-overallRatio
        
                def constr2(x,overallRatio):
                    return overallRatio-x[0]*x[1]*x[2]

                x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7, iprint = 0)
        
            elif config == 'eep_3':
                #fixes last stage ratio at 3
                x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                B_1=Np[0]
                B_2=Np[1]
                K_r1=0
                K_r2=0.8 #2nd stage structure weight coefficient

                def volume(x):
                    return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+(x[0]/2.0-1)**2+K_r1*((x[0]-1.0)**2)/B_1 + K_r1*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + (1.0/(x[0]*x[1]))*((1.0/B_2)+(1/(B_2*((x[1]/2.0)-1.0)))+(x[1]/2.0-1.0)+(x[1]/2.0-1.0)**2.0+K_r2*((x[1]-1.0)**2.0)/B_2 + K_r2*((x[1]-1.0)**2.0)/(B_2*(x[1]/2.0-1.0))) + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                
                def constr1(x,overallRatio):
                    return x[0]*x[1]*x[2]-overallRatio
        
                def constr2(x,overallRatio):
                    return overallRatio-x[0]*x[1]*x[2]
                
                def constr3(x,overallRatio):
                    return x[2]-3.0
                
                def constr4(x,overallRatio):
                    return 3.0-x[2]

                x=opt.fmin_cobyla(volume, x0,[constr1,constr2,constr3,constr4],consargs=[overallRatio],rhoend=1e-7,iprint=0)
            
            elif config == 'eep_2':
                #fixes final stage ratio at 2
                x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                B_1=Np[0]
                B_2=Np[1]
                K_r1=0
                K_r2=1.6 #2nd stage structure weight coefficient

                def volume(x):
                    return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+(x[0]/2.0-1)**2+K_r1*((x[0]-1.0)**2)/B_1 + K_r1*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + (1.0/(x[0]*x[1]))*((1.0/B_2)+(1/(B_2*((x[1]/2.0)-1.0)))+(x[1]/2.0-1.0)+(x[1]/2.0-1.0)**2.0+K_r2*((x[1]-1.0)**2.0)/B_2 + K_r2*((x[1]-1.0)**2.0)/(B_2*(x[1]/2.0-1.0))) + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                
                def constr1(x,overallRatio):
                    return x[0]*x[1]*x[2]-overallRatio
        
                def constr2(x,overallRatio):
                    return overallRatio-x[0]*x[1]*x[2]

                x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7, iprint = 0)
            elif config == 'epp':
                #fixes last stage ratio at 3
                x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                B_1=Np[0]
                B_2=Np[1]
                K_r=0
               
                def volume(x):
                    return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+(x[0]/2.0-1)**2+ \
                    K_r*((x[0]-1.0)**2)/B_1 + K_r*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + \
                    (1.0/(x[0]*x[1]))*(1.0+(1.0/x[1])+x[1] + x[1]**2) \
                    + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                
                def constr1(x,overallRatio):
                    return x[0]*x[1]*x[2]-overallRatio
        
                def constr2(x,overallRatio):
                    return overallRatio-x[0]*x[1]*x[2]
                
                x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7,iprint=0)
                
            else:  # what is this subroutine for?  Yi on 04/16/2014
                x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                B_1=Np[0]
                K_r=0.0
                def volume(x):
                    return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1)+(x[0]/2.0-1.0)**2+K_r*((x[0]-1.0)**2)/B_1 + K_r*((x[0]-1)**2)/(B_1*(x[0]/2.0-1.0))) + (1.0/(x[0]*x[1]))*(1.0+(1.0/x[1])+x[1] + x[1]**2)+ (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)
                                  
                def constr1(x,overallRatio):
                    return x[0]*x[1]*x[2]-overallRatio
        
                def constr2(x,overallRatio):
                    return overallRatio-x[0]*x[1]*x[2]

                x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7, iprint = 0)
        else:
            x='fail'
                  
        return x        
        
        








#############################################################################
#########################  MODEL#2: 1-Stage Polinder ########################
#############################################################################        
class Polinder(Gearbox):
    def compute(self, inputs, outputs):

        gear_ratio = inputs['gear_ratio']
        Np = inputs['Np']
        rotor_torque = inputs['rotor_torque']
        
        loss = 0.015 # loss percentage at rated power
        rw = (gear_ratio/2.0) - 1
        Tm = rotor_torque/gear_ratio
        Fw = (1.0/Np) + (1.0/(Np*rw)) + rw + (rw**2) + 0.4*((1+rw)*(gear_ratio-1)**2)/Np # weight factor
        Fs = 1.25 # gearbox service factor - surface damage and failure by metal fatigue
        mass = 3.2*Tm*Fs*Fw/1000.0        
        
        outputs['stage_masses'] = mass
        outputs['mass'] = mass
        outputs['efficiency'] = 1.0 - loss
        
        # the following outputs are not calculated
        outputs['cm'] = np.zeros(3)
        outputs['I'] = np.zeros(3)
        outputs['length'] = 0.0
        outputs['height'] = 0.0
        outputs['diameter'] = 0.0
        
        
        
        
        



#############################################################################
##############################  UNIT TESTING ################################
#############################################################################         
class GearboxTest(Group):
    def setup(self):
        
        i = IndepVarComp()
        
        # design variables
        i.add_output('gear_ratio', desc='overall gearbox speedup ratio')
        i.add_output('Np', desc='number of planets in each stage', shape=gearbox_stages)
        i.add_output('rotor_speed', units='rpm', desc='rotor rpm at rated power')
        i.add_output('rotor_diameter', units='m', desc='rotor diameter')
        i.add_output('rotor_torque', units='N*m', desc='rotor torque at rated power')
        i.add_output('gearbox_cm_x', units='m', desc ='gearbox position along x-axis')
        
        # sub-components         
        self.add_subsystem('dof', i, promotes=['*'])   
        self.add_subsystem('sys', DriveSE(), promotes_inputs=['*'])
        
        
    def test5(self):
        start = time()
    
        # workflow setup
        prob = Problem(GearboxTest())
        prob.setup()
        #view_model(prob, outfile='gearbox.html')
        
        # define inputs
        prob['dof.gear_ratio'] = 96.76
        prob['dof.Np'] = [3,3,1]
        prob['dof.rotor_speed'] = 12.1
        prob['dof.rotor_diameter'] = 126.0
        prob['dof.rotor_torque'] = (1.5*5000*1000/0.95)/(12.1*pi/30)
        prob['dof.gearbox_cm_x'] = 0.1
        
         
        prob.run_model()
        
        # print outputs 
        print "DriveSE Gearbox"
        beautify('stage_masses', prob['sys.stage_masses'])
        beautify('mass', prob['sys.mass'])
        beautify('cm', prob['sys.cm'])
        beautify('I', prob['sys.I'])
        beautify('length', prob['sys.length'])
        beautify('height', prob['sys.height'])
        beautify('diameter', prob['sys.diameter'])
      
        print 'Done in ' + str(time() - start) + ' seconds'  
        
        
    def Polinder(self):
        start = time()
    
        # workflow setup
        prob = Problem(GearboxTest())
        prob.setup()
        #view_model(prob, outfile='gearbox.html')
        
        # define inputs
        prob['dof.gear_ratio'] = 6
        prob['dof.Np'] = 3
        prob['dof.rotor_torque'] = (1.5*1500*1000/0.95)/(16.18*pi/30)
        
         
        prob.run_model()
        
        # print outputs 
        print "1-stage gearbox"
        beautify('stage_masses', prob['sys.stage_masses'])
        beautify('mass', prob['sys.mass'])
        beautify('efficiency', prob['sys.efficiency'])
      
        print 'Done in ' + str(time() - start) + ' seconds'     
        
        
        


if __name__ == "__main__":
    GearboxTest().test5()
    #GearboxTest().Polinder()
        