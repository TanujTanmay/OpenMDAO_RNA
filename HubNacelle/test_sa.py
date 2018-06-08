from time import time
from math import pi
from openmdao.api import Group, IndepVarComp, Problem, view_model 

from nacelle_assembly import UnitTest
import gearbox, lss, bearing, hss, generator, bedplate, yaw, transformer, above_yaw, rna, nacelle
from fixed_parameters import beautify
   
    
    
#############################################################################
################################  Test Unit #################################
#############################################################################        
def test():   
    start = time()

    # workflow setup
    prob = Problem(UnitTest())
    prob.setup()
    #view_model(prob, outfile='nacelle.html')
    
    # define inputs
    prob['dof.rotor_diameter'] = 126.0
    prob['dof.rotor_mass'] = 99072.6974
    prob['dof.rotor_torque'] = (5000*1000/0.95)/(12.1*pi/30)
    prob['dof.rotor_thrust'] = 599610.0 
    prob['dof.rotor_speed'] = 12.1
    prob['dof.machine_rating'] = 5000.0
    prob['dof.gear_ratio'] = 96.76
    prob['dof.tower_top_diameter'] = 3.78
    prob['dof.rotor_bending_moment'] = [330770.0, -16665000.0 , 2896300.0]
    prob['dof.rotor_force'] = [599610.0, 186780.0 , -842710.0 ]
    prob['dof.shaft_angle'] = 5.0
    prob['dof.shaft_ratio'] = 0.10
    prob['dof.overhang'] = 5.0
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.Np'] = [3,3,1]
    
    prob.run_model()

    
    # print outputs 
    var_list = ['gearbox.mass', 'lss.mass', 'lss.length' ,'lss.diameter1', 'lss.diameter2', \
            'main_bearing.mass', 'second_bearing.mass', \
            'hss.mass', 'generator.mass', 'bedplate.mass', 'yaw.mass', 'transformer.mass', 'nacelle.nacelle_mass']
    for var in var_list:
        beautify(var, prob['sys.' + var])
        
    print 'Done in ' + str(time() - start) + ' seconds'





#############################################################################
############################# Sensitivity Analysis ##########################
#############################################################################          
def sa():
    import matplotlib.pyplot as plt
    start = time()
    
    # variables of interest
    var_list = ['gearbox.mass', 'lss.mass', 'lss.length' ,'lss.diameter1', 'lss.diameter2', \
            'main_bearing.mass', 'second_bearing.mass', \
            'hss.mass', 'generator.mass', 'bedplate.mass', 'yaw.mass', 'transformer.mass', 'nacelle.nacelle_mass']

    # workflow setup
    prob = Problem(UnitTest())
    prob.setup()
    #view_model(prob, outfile='nacelle.html')
    
    # define inputs
    prob['dof.rotor_diameter'] = 126.0
    prob['dof.rotor_mass'] = 110000.0 # 51092.6639
    prob['dof.rotor_torque'] = (1.5*5000*1000/0.95)/(12.1*pi/30)
    prob['dof.rotor_thrust'] = 599610.0 
    prob['dof.rotor_speed'] = 12.1
    prob['dof.machine_rating'] = 5000.0
    prob['dof.gear_ratio'] = 96.76
    prob['dof.tower_top_diameter'] = 3.78
    prob['dof.rotor_bending_moment'] = [330770.0, -16665000.0 , 2896300.0]
    prob['dof.rotor_force'] = [599610.0, 186780.0 , -842710.0 ]
    prob['dof.shaft_angle'] = 5.0
    prob['dof.shaft_ratio'] = 0.10
    prob['dof.overhang'] = 5.0
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.Np'] = [3,3,1]
    
    prob.run_model()
    
    ref_lss_mass =  prob['sys.lss.mass'][0]
    ref_bedplate_mass =  prob['sys.bedplate.mass'][0]
    
    f1= plt.figure(1)
    p1 = f1.add_subplot(121)
    p1.set_title('Force - LSS')
    p2 = f1.add_subplot(122)
    p2.set_title('Force - Bedplate')
    
    f2= plt.figure(2)
    p3 = f2.add_subplot(121)
    p3.set_title('Moment - LSS')
    p4 = f2.add_subplot(122)
    p4.set_title('Moment - Bedplate')
    
    i_list = [0.5, 0.75, 1.0, 1.25, 1.5]
    
    # force sensitivity
    for j in range(3):
        m_lss = []
        m_bed = []
        for i in i_list:
            prob['dof.rotor_force'] = [599610.0, 186780.0 , -842710.0 ]
            prob['dof.rotor_force'][j] *= i
            prob.run_model()
            m_lss.append(prob['sys.lss.mass'][0]/ref_lss_mass)
            m_bed.append(prob['sys.bedplate.mass'][0]/ref_bedplate_mass)
                
        p1.plot(i_list, m_lss, label='F'+str(j))    
        p2.plot(i_list, m_bed, label='F'+str(j)) 
        
    # bending moment sensitivity
    for j in range(3):
        m_lss = []
        m_bed = []
        for i in i_list:
            prob['dof.rotor_bending_moment'] = [330770.0, -16665000.0 , 2896300.0]
            prob['dof.rotor_bending_moment'][j] *= i
            prob.run_model()
            m_lss.append(prob['sys.lss.mass'][0]/ref_lss_mass)
            m_bed.append(prob['sys.bedplate.mass'][0]/ref_bedplate_mass)
                
        p3.plot(i_list, m_lss, label='M'+str(j))    
        p4.plot(i_list, m_bed, label='M'+str(j))     
    
    print 'Done in ' + str(time() - start) + ' seconds' 
    
    for p in [p1, p2, p3, p4]: 
        p.legend() 
    plt.show() 
        




        
if __name__ == "__main__":
    test() 
    #sa()     
