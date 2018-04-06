from time import time
from math import pi, sin, cos, radians

from openmdao.api import ExplicitComponent, Group, IndepVarComp, Problem, view_model   

#############################################################################
##############################  I/O SKELETON ################################
#############################################################################
class Gearbox(ExplicitComponent):
    def setup(self):
        #inputs
        self.add_input('gear_ratio', desc='overall gearbox speedup ratio')
        self.add_input('Np', desc='number of planets in each stage', shape=3)
        self.add_input('rotor_speed', units='rpm', desc='rotor rpm at rated power')
        self.add_input('rotor_diameter', units='m', desc='rotor diameter')
        self.add_input('rotor_torque', units='N*m', desc='rotor torque at rated power')
        self.add_input('gearbox_cm_x', units='m', desc ='gearbox position along x-axis')
    
        #parameters
        #self.add_input('gear_configuration', desc='string that represents the configuration of the gearbox (stage number and types)')
        #self.add_input('ratio_type', desc='optimal or empirical stage ratios')
        #self.add_input('shaft_type', desc = 'normal or short shaft length')
    
        # outputs
        self.add_output('stage_masses', units='kg', desc='individual gearbox stage masses', shape=3)
        self.add_output('mass', units='kg', desc='overall component mass')
        self.add_output('cm', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system', shape=3)
        self.add_output('I', units='kg*m**2', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass', shape=3)    
        self.add_output('length', units='m', desc='gearbox length')
        self.add_output('height', units='m', desc='gearbox height')
        self.add_output('diameter', units='m', desc='gearbox diameter')
        
        
        
        
        
        


class GearboxDriveSE(Gearbox):
    def compute(self, inputs, outputs):
        # inputs
        gear_ratio = inputs['gear_ratio']
        Np = inputs['Np']
        rotor_speed = inputs['rotor_speed']
        rotor_diameter = inputs['rotor_diameter']
        rotor_torque = inputs['rotor_torque']
        gearbox_cm_x = inputs['gearbox_cm_x']
        #gear_configuration = inputs['gear_configuration']
        #ratio_type = inputs['ratio_type']
        #shaft_type = inputs['shaft_type']
        
        gear_configuration='eep'
        ratio_type='optimal' # 'empirical'
        shaft_type='normal'

        stageRatio=np.zeros([3,1])
        stageTorque = np.zeros([len(stageRatio),1]) #filled in when ebxWeightEst is called
        stageMass = np.zeros([len(stageRatio),1]) #filled in when ebxWeightEst is called
        stageType=stageTypeCalc(gear_configuration)
        stageRatio=stageRatioCalc(gear_ratio,Np,ratio_type,gear_configuration)
        
        [m, stageMass] =gbxWeightEst(gear_configuration,gear_ratio,Np,ratio_type,shaft_type, \
                       rotor_torque, stageRatio, stageTorque, stageMass, stageType)
        #print(m)
        mass = float(m)
        outputs['mass'] = mass 
        outputs['stage_masses']= stageMass
        # calculate mass properties

        length = (0.012 * rotor_diameter)
        height = (0.015 * rotor_diameter)
        diameter = (0.75 * height)

        cm0   = gearbox_cm_x
        cm1   = 0.0
        cm2   = 0.4*height #TODO validate or adjust factor. origin is modified to be above bedplate top
        outputs['length'] = length
        outputs['height'] = height
        outputs['diameter'] = diameter
        outputs['cm'] = np.array([cm0, cm1, cm2]) #np.ndarray(shape=(3,), buffer=np.array([cm0, cm1, cm2])) #np.ndarray([cm0, cm1, cm2])
#         outputs['cm'][0] = cm0
#         outputs['cm'][1] = cm1
#         outputs['cm'][2] = cm2
        #outputs['cm'] = np.reshape(cm, (3,1))

        I0 = mass * (diameter ** 2 ) / 8 + (mass / 2) * (height ** 2) / 8
        I1 = mass * (0.5 * (diameter ** 2) + (2 / 3) * (length ** 2) + 0.25 * (height ** 2)) / 8
        I2 = I1
        outputs['I'] = np.array([I0, I1, I2])
        
        
        
            
    def stageTypeCalc(self, config):
        temp=[]
        for character in config:
                if character == 'e':
                    temp.append(2)
                if character == 'p':
                    temp.append(1)
        return temp
    
    def stageMassCalc(indStageRatio,indNp,indStageType):
    
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
        
    def gbxWeightEst(config,overallRatio,Np,ratio_type,shaft_type,rotor_torque, stageRatio, stageTorque, stageMass, stageType):
    
    
        '''
        Computes the gearbox weight based on a surface durability criteria.
        '''
    
        ## Define Application Factors ##
        #Application factor for weight estimate
        Ka=0.6
        Kshaft=0.0
        Kfact=0.0
    
        #K factor for pitting analysis
        if rotor_torque < 200000.0:
            Kfact = 850.0
        elif rotor_torque < 700000.0:
            Kfact = 950.0
        else:
            Kfact = 1100.0
    
        #Unit conversion from Nm to inlb and vice-versa
        Kunit=8.029
    
        # Shaft length factor
        if shaft_type == 'normal':
            Kshaft = 1.0
        elif shaft_type == 'short':
            Kshaft = 1.25
    
        #Individual stage torques
        torqueTemp=rotor_torque
        for s in range(len(stageRatio)):
            #print torqueTemp
            #print stageRatio[s]
            stageTorque[s]=torqueTemp/stageRatio[s]
            torqueTemp=stageTorque[s]
            stageMass[s]=Kunit*Ka/Kfact*stageTorque[s]*stageMassCalc(stageRatio[s],Np[s],stageType[s])
        
        gbxWeight=(sum(stageMass))*Kshaft
        
        return np.array([gbxWeight, stageMass])
    
    def stageRatioCalc(overallRatio,Np,ratio_type,config):
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
    
                x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7, disp = 0)
        
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