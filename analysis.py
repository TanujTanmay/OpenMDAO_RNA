from time import time
import numpy as np
import matplotlib.pyplot as plt

from openmdao.api import Problem, view_model 
from rna import RNASimpleTest, RNAAeroDynTest
from fixed_parameters import beautify


def TestAeroDyn():
    start = time()
    
    print 'I am here'
    # get and set values of the variables using Problem
    prob = Problem(RNASimpleTest())
    prob.setup()
     
    prob['dof.design_tsr'] = 7.0
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 126.0
    prob['dof.hub_radius'] = 1.5
    prob['dof.root_chord'] = 3.4
    prob['dof.span_airfoil_r'] =  [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
    prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]
    prob['dof.adjust_pitch'] = 0    
    prob['dof.thickness_factor'] = 1.0
    prob['dof.wind_speed'] = 11.3
    prob['dof.hub_height'] = 90.0
    prob['dof.precone'] = -2.5
    prob['dof.yaw'] = 0.0
    prob['dof.overhang'] = 5.0191
    prob['dof.shaft_angle'] = -5.0
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 5000.0/0.944
    
    prob['dof.gear_ratio'] = 97.0
    prob['dof.crane'] = 1
    prob['dof.shaft_ratio'] = 0.1
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.shrink_disc_mass'] = 5000.0/3.0
    prob['dof.carrier_mass'] = 8000.
    prob['dof.flange_length'] = 0.5
    #prob['dof.overhang'] = 5.
    prob['dof.L_rb'] = 1.912
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.tower_top_diameter'] = 3.87
    prob['dof.hss_length'] = 1.5
    
    
    prob.run_model()
     
    print "NREL 5 MW"
    print 'rated_wind_speed = ' + beautify(prob['rna.rated_wind_speed'])
    print 'swept_area = ' + beautify(prob['rna.swept_area'])
    print 'rotor_cp = ' + str(prob['rna.rotor_cp'])
    print 'rotor_cq = ' + str(prob['rna.rotor_cq'])
    print 'rotor_ct = ' + str(prob['rna.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['rna.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['rna.rotor_torque'])
    print 'rotor_thrust = ' + beautify(prob['rna.rotor_thrust'])
    print 'pitch = ' + beautify(prob['rna.pitch'])
    
    
    print 'span_r = ' + beautify(prob['rna.span_r'])
    print 'span_airfoil = ' + beautify(prob['rna.span_airfoil'])
    print 'span_dr = ' + beautify(prob['rna.span_dr'])
    print 'span_chord = ' + beautify(prob['rna.span_chord'])
    print 'span_twist = ' + beautify(prob['rna.span_twist'])
    print 'span_mass = ' + beautify(prob['rna.span_mass'])
    print 'hub_diameter = ' + beautify(prob['rna.hub_diameter'])
    print 'span_fx = ' + beautify(prob['rna.span_fx'])
    print 'span_fy = ' + beautify(prob['rna.span_fy'])
    print 'span_moment_flap = ' + beautify(prob['rna.span_moment_flap'])
    print 'span_moment_edge = ' + beautify(prob['rna.span_moment_edge'])
    print 'span_stress_flap = ' + beautify(prob['rna.span_stress_flap'])
    print 'span_stress_edge = ' + beautify(prob['rna.span_stress_edge'])
    print 'span_stress_gravity = ' + beautify(prob['rna.span_stress_gravity'])
    print 'tip_deflection = ' + beautify(prob['rna.tip_deflection'])
    print 'hub_system_mass = ' + beautify(prob['rna.hub_system_mass'])
    print 'nacelle_mass = ' + beautify(prob['rna.nacelle_mass'])
    print 'mb_mass = ' + beautify(prob['rna.main_bearing_mass'])
    print 'gb_mass = ' + beautify(prob['rna.gearbox_mass'])
    print 'rotor_mass = ' + beautify(prob['rna.rotor_mass'])
    print 'cost_rna_total = ' + beautify(prob['rna.cost_rna_total'])
    print 'rotor_moment = ' + beautify(prob['rna.rotor_moment'])
    print 'rotor_force = ' + beautify(prob['rna.rotor_force'])
    print 'power_bin = ' + beautify(prob['rna.power_bin'])

    plt.close('all')    
    f, (ax1, ax2, ax3) = plt.subplots(1, 3)
     
    ax1.plot(np.array(prob['rna.span_r']), np.array(prob['rna.span_fx']), linestyle='-', label='BEM')
    ax1.set_xlabel('Span [m]')
    ax1.set_ylabel('Fx [N]')
    ax1.hold(True)
    
    ax2.plot(np.array(prob['rna.span_r']), np.array(prob['rna.span_fy']), linestyle='-', label='BEM')
    ax2.set_xlabel('Span [m]')
    ax2.set_ylabel('Fy [N]')
    ax2.hold(True)
    
    ax3.plot(np.array(prob['rna.wind_bin']), np.array(prob['rna.power_bin']), linestyle='-', label='BEM')
    ax3.set_xlabel('Wind Speed [m/s]')
    ax3.set_ylabel('Aerodynamic power [kW]')
    ax3.hold(True)
     



    # get and set values of the variables using Problem
    prob = Problem(RNAAeroDynTest())
    prob.setup()
     
    prob['dof.design_tsr'] = 7.0
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 126.0
    prob['dof.hub_radius'] = 1.5
    prob['dof.root_chord'] = 3.4
    prob['dof.span_airfoil_r'] =  [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
    prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]
    prob['dof.adjust_pitch'] = 0    
    prob['dof.thickness_factor'] = 1.0
    prob['dof.wind_speed'] = 11.3
    prob['dof.hub_height'] = 90.0
    prob['dof.precone'] = -2.5
    prob['dof.yaw'] = 0.0
    prob['dof.overhang'] = 5.0191
    prob['dof.shaft_angle'] = -5.0
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 5000.0/0.944
    
    prob['dof.gear_ratio'] = 97.0
    prob['dof.crane'] = 1
    prob['dof.shaft_ratio'] = 0.1
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.shrink_disc_mass'] = 5000.0/3.0
    prob['dof.carrier_mass'] = 8000.
    prob['dof.flange_length'] = 0.5
    #prob['dof.overhang'] = 5.
    prob['dof.L_rb'] = 1.912
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.tower_top_diameter'] = 3.87
    prob['dof.hss_length'] = 1.5
    
    
    prob.run_model()
     
    print "NREL 5 MW"
    print 'rated_wind_speed = ' + beautify(prob['rna.rated_wind_speed'])
    print 'swept_area = ' + beautify(prob['rna.swept_area'])
    print 'rotor_cp = ' + str(prob['rna.rotor_cp'])
    print 'rotor_cq = ' + str(prob['rna.rotor_cq'])
    print 'rotor_ct = ' + str(prob['rna.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['rna.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['rna.rotor_torque'])
    print 'rotor_thrust = ' + beautify(prob['rna.rotor_thrust'])
    print 'pitch = ' + beautify(prob['rna.pitch'])
    
    
    print 'span_r = ' + beautify(prob['rna.span_r'])
    print 'span_airfoil = ' + beautify(prob['rna.span_airfoil'])
    print 'span_dr = ' + beautify(prob['rna.span_dr'])
    print 'span_chord = ' + beautify(prob['rna.span_chord'])
    print 'span_twist = ' + beautify(prob['rna.span_twist'])
    print 'span_mass = ' + beautify(prob['rna.span_mass'])
    print 'hub_diameter = ' + beautify(prob['rna.hub_diameter'])
    print 'span_fx = ' + beautify(prob['rna.span_fx'])
    print 'span_fy = ' + beautify(prob['rna.span_fy'])
    print 'span_moment_flap = ' + beautify(prob['rna.span_moment_flap'])
    print 'span_moment_edge = ' + beautify(prob['rna.span_moment_edge'])
    print 'span_stress_flap = ' + beautify(prob['rna.span_stress_flap'])
    print 'span_stress_edge = ' + beautify(prob['rna.span_stress_edge'])
    print 'span_stress_gravity = ' + beautify(prob['rna.span_stress_gravity'])
    print 'tip_deflection = ' + beautify(prob['rna.tip_deflection'])
    print 'hub_system_mass = ' + beautify(prob['rna.hub_system_mass'])
    print 'nacelle_mass = ' + beautify(prob['rna.nacelle_mass'])
    print 'mb_mass = ' + beautify(prob['rna.main_bearing_mass'])
    print 'gb_mass = ' + beautify(prob['rna.gearbox_mass'])
    print 'rotor_mass = ' + beautify(prob['rna.rotor_mass'])
    print 'cost_rna_total = ' + beautify(prob['rna.cost_rna_total'])
    print 'rotor_moment = ' + beautify(prob['rna.rotor_moment'])
    print 'rotor_force = ' + beautify(prob['rna.rotor_force'])
    print 'power_bin = ' + beautify(prob['rna.power_bin'])

     
    ax1.plot(np.array(prob['rna.span_r']), np.array(prob['rna.span_fx']), linestyle='-', label='AeroDyn')
    ax1.set_xlabel('Span [m]')
    ax1.set_ylabel('Fx [N]')
    ax1.hold(True)
    
    
    
     
    ax2.plot(np.array(prob['rna.span_r']), np.array(prob['rna.span_fy']), linestyle='-', label='AeroDyn')
    ax2.set_xlabel('Span [m]')
    ax2.set_ylabel('Fy [N]')
    ax2.hold(True)

    ax3.plot(np.array(prob['rna.wind_bin']), np.array(prob['rna.power_bin']), linestyle='-', label='AeroDyn')
    ax3.set_xlabel('Wind Speed [m/s]')
    ax3.set_ylabel('Aerodynamic power [kW]')
    ax3.hold(True)







     
    ax1.legend(loc='best')
    ax2.legend(loc='best')
    ax3.legend(loc='best')
    plt.show()
    plt.savefig('Plots/scaling.png')
    
  
    print 'Done in ' + str(time() - start) + ' seconds'
    
    
    

   
def TestSiemens():
    
    start = time()
    
    # get and set values of the variables using Problem
    prob = Problem(RNA_usecase())
    prob.setup()
    view_model(prob, outfile='N2/rna_siemens.html')
    
    span_af = [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
    
    prob['dof.design_tsr'] = 7.4
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 108.0
    prob['dof.hub_radius'] = 1.5
    prob['dof.root_chord'] = 3.4
    prob['dof.span_airfoil_r'] =  [x*108.0/126.0 for x in span_af]
    prob['dof.span_airfoil_id'] = [0,     1,     2,     3,     4,     5,     6,     7]
    prob['dof.adjust_pitch'] = 1    
    prob['dof.thickness_factor'] = 0.63
    prob['dof.wind_speed'] = 9.0
    prob['dof.hub_height'] = 90.0
    prob['dof.precone'] = -2.5
    prob['dof.yaw'] = 0.0
    #prob['dof.overhang'] = -5.0191
    prob['dof.shaft_angle'] = -5.0
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 20.0
    prob['dof.machine_rating'] = 2300.0
    
    prob['dof.gear_ratio'] = 91.0
    prob['dof.crane'] = 0
    prob['dof.shaft_ratio'] = 0.1
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.shrink_disc_mass'] = 2300.0/3.0
    prob['dof.carrier_mass'] = 8000.
    prob['dof.flange_length'] = 0.5
    prob['dof.overhang'] = 5.
    prob['dof.L_rb'] = 1.912
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.tower_top_diameter'] = 3.87
    prob['dof.hss_length'] = 1.5
     
     
    prob.run_model()
     
    print "SWT-2.3-108"
    print 'rated_wind_speed = ' + beautify(prob['rna.rated_wind_speed'])
    print 'swept_area = ' + beautify(prob['rna.swept_area'])
    print 'rotor_cp = ' + str(prob['rna.rotor_cp'])
    print 'rotor_cq = ' + str(prob['rna.rotor_cq'])
    print 'rotor_ct = ' + str(prob['rna.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['rna.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['rna.rotor_torque'])
    print 'rotor_thrust = ' + beautify(prob['rna.rotor_thrust'])
    print 'pitch = ' + beautify(prob['rna.pitch'])
    print 'span_r = ' + beautify(prob['rna.span_r'])
    print 'span_airfoil = ' + beautify(prob['rna.span_airfoil'])
    print 'span_dr = ' + beautify(prob['rna.span_dr'])
    print 'span_chord = ' + beautify(prob['rna.span_chord'])
    print 'span_twist = ' + beautify(prob['rna.span_twist'])
    print 'span_mass = ' + beautify(prob['rna.span_mass'])
    print 'hub_diameter = ' + beautify(prob['rna.hub_diameter'])
    print 'span_fx = ' + beautify(prob['rna.span_fx'])
    print 'span_fy = ' + beautify(prob['rna.span_fy'])
    print 'span_moment_flap = ' + beautify(prob['rna.span_moment_flap'])
    print 'span_moment_edge = ' + beautify(prob['rna.span_moment_edge'])
    print 'span_stress_flap = ' + beautify(prob['rna.span_stress_flap'])
    print 'span_stress_edge = ' + beautify(prob['rna.span_stress_edge'])
    print 'span_stress_gravity = ' + beautify(prob['rna.span_stress_gravity'])
    print 'tip_deflection = ' + beautify(prob['rna.tip_deflection'])
    print 'hub_system_mass = ' + beautify(prob['rna.hub_system_mass'])
    print 'nacelle_mass = ' + beautify(prob['rna.nacelle_mass'])
    print 'rotor_mass = ' + beautify(prob['rna.rotor_mass'])
    print 'cost_rna_total = ' + beautify(prob['rna.cost_rna_total'])
    print 'rotor_moment = ' + beautify(prob['rna.rotor_moment'])
    print 'rotor_force = ' + beautify(prob['rna.rotor_force'])


    
    
  
    print 'Done in ' + str(time() - start) + ' seconds'    
    
    
    
    
    
    
    
    
def TestVestas():
    
    from openmdao.api import Problem, view_model
     
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RNA_usecase())
    prob.setup()
    #view_model(prob, outfile='N2/rna_new.html')
     
    prob['dof.pot_wind_speed'] = 8.3
    prob['dof.hub_height'] = 90.0
    
    prob['dof.design_tsr'] = 8.1
    prob['dof.blade_number'] = 3
    prob['dof.rotor_diameter'] = 120.0
    prob['dof.hub_radius'] = 2.3
    prob['dof.root_chord'] = 4.2
    prob['dof.span_airfoil_r'] = [4.0, 12.0]
    prob['dof.span_airfoil_id'] = [0, 1]
    prob['dof.adjust_pitch'] = 1    
    prob['dof.thickness_factor'] = 0.5
    #prob['dof.wind_speed'] = 9.0 
    prob['dof.rotor_speed'] = 13.0
    prob['dof.cut_in_speed'] = 4.0
    prob['dof.cut_out_speed'] = 25.0
    prob['dof.machine_rating'] = 3600.0
    prob['dof.shaft_angle'] = 0.0
    
    prob['dof.gear_ratio'] = 119.0
    prob['dof.crane'] = 0
    prob['dof.shaft_ratio'] = 0.1
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.shrink_disc_mass'] = 3600.0/3.0
    prob['dof.carrier_mass'] = 5000.
    prob['dof.flange_length'] = 0.3
    prob['dof.overhang'] = 4.
    prob['dof.L_rb'] = 1.5
    prob['dof.gearbox_cm_x'] = 0.0
    prob['dof.tower_top_diameter'] = 2.3
    prob['dof.hss_length'] = 0.
    
    
     
     
    prob.run_model()
     
    print "Siemens SWT-2.3-108"
    print 'wind_speed = ' + beautify(prob['rna.wind_speed'])
    print 'rated_wind_speed = ' + beautify(prob['rna.rated_wind_speed'])
    print 'rotor_tsr = ' + beautify(prob['rna.rotor_tsr'])
    print 'swept_area = ' + beautify(prob['rna.swept_area'])
    print 'rotor_cp = ' + beautify(prob['rna.rotor_cp'])
    print 'rotor_cq = ' + beautify(prob['rna.rotor_cq'])
    print 'rotor_ct = ' + beautify(prob['rna.rotor_ct'])
    print 'rotor_power = ' + beautify(prob['rna.rotor_power'])
    print 'rotor_torque = ' + beautify(prob['rna.rotor_torque'])
    print 'rotor_thrust = ' + beautify(prob['rna.rotor_thrust'])
    print 'root_bending_moment = ' + beautify(prob['rna.root_bending_moment'])
    print 'root_force = ' + beautify(prob['rna.root_force'])
    print 'pitch = ' + beautify(prob['rna.pitch'])
    print 'span_dr = ' + beautify(prob['rna.span_dr'])
    print 'span_chord = ' + beautify(prob['rna.span_chord'])
    print 'span_twist = ' + beautify(prob['rna.span_twist'])
    print 'hub_diameter = ' + beautify(prob['rna.hub_diameter'])
    print 'span_fx = ' + beautify(prob['rna.span_fx'])
    print 'span_fy = ' + beautify(prob['rna.span_fy'])
    print 'span_moment_flap = ' + beautify(prob['rna.span_moment_flap'])
    print 'span_moment_edge = ' + beautify(prob['rna.span_moment_edge'])
    print 'span_stress_flap = ' + beautify(prob['rna.span_stress_flap'])
    print 'span_stress_edge = ' + beautify(prob['rna.span_stress_edge'])
    print 'span_stress_gravity = ' + beautify(prob['rna.span_stress_gravity'])
    print 'tip_deflection = ' + beautify(prob['rna.tip_deflection'])
    print 'hub_system_mass = ' + beautify(prob['rna.hub_system_mass'])
    print 'nacelle_mass = ' + beautify(prob['rna.nacelle_mass'])
    print 'rotor_mass = ' + beautify(prob['rna.rotor_mass'])
    print 'cost_rna_total = ' + beautify(prob['rna.cost_rna_total'])

    
    
  
    print 'Done in ' + str(time() - start) + ' seconds'       
    
    
    
    
    
    
   
    
    
    
    
def Scaling():
    
    from openmdao.api import Problem, view_model
     
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RNAAeroDynTest())
    prob.setup()
    #view_model(prob, outfile='N2/rna_scale.html')
    
    
    
    span_af = [01.36, 06.83, 10.25, 14.35, 22.55, 26.65, 34.85, 43.05]
    
    prob['dof.design_tsr'] = 7.0
    prob['dof.blade_number'] = 3
    
    prob['dof.span_airfoil_id'] = [0,     1,     2,        3,     4,     5,     6,     7]
    prob['dof.adjust_pitch'] = 0    
    prob['dof.thickness_factor'] = 1.0
    prob['dof.wind_speed'] = 11.3
    prob['dof.hub_height'] = 90.0
    prob['dof.precone'] = -2.5
    prob['dof.yaw'] = 0.0
    prob['dof.overhang'] = 5.0191
    prob['dof.shaft_angle'] = -5.0
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0

    
    prob['dof.gear_ratio'] = 97.0
    prob['dof.crane'] = 0
    prob['dof.shaft_ratio'] = 0.1
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.carrier_mass'] = 8000.
    prob['dof.flange_length'] = 0.5
    #prob['dof.overhang'] = 5.
    prob['dof.L_rb'] = 1.912
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.tower_top_diameter'] = 3.87
    prob['dof.hss_length'] = 1.5
    
    td = []
    flap = []
    grav = []
    mb_mass = []
    rotor_mass = []
    gb_mass = []
    
    for d in range(100,150):
        prob['dof.rotor_diameter'] = d
        s = d/126.0
        prob['dof.hub_radius'] = 1.5 * s
        prob['dof.root_chord'] = 3.4 * s
        prob['dof.machine_rating'] = (5000.0/0.944) * s**2
        prob['dof.shrink_disc_mass'] = (5000.0 * s**2) /3.0
        #prob['dof.rotor_speed'] = 11.9/s
        
        prob['dof.span_airfoil_r'] =  [x*s for x in span_af]
         
        prob.run_model()
        
        #print prob['rna.tip_deflection']
        #print prob['rna.span_stress_flap'][0]
        #print prob['rna.span_stress_gravity'][0], prob['rna.span_stress_flap'][0], prob['rna.tip_deflection'][0]
        print d, prob['rna.main_bearing_mass'][0], prob['rna.rotor_mass'][0], prob['rna.gearbox_mass'][0], \
        prob['rna.span_stress_gravity'][0], prob['rna.span_stress_flap'][0], prob['rna.tip_deflection'][0]
        
        td.append(prob['rna.tip_deflection'][0])
        flap.append(prob['rna.span_stress_flap'][0])
        grav.append(prob['rna.span_stress_gravity'][0])
        mb_mass.append(prob['rna.main_bearing_mass'][0])
        rotor_mass.append(prob['rna.rotor_mass'][0])
        gb_mass.append(prob['rna.gearbox_mass'][0])


    d = np.array(range(100,150))
    td = np.array([x/5.2527 for x in td]) # np.array([x/4.9584 for x in td])
    flap = np.array([x/72297582.7318 for x in flap]) # np.array([x/67923595.1004 for x in flap])
    grav = np.array([x/35473276.5686 for x in grav]) # np.array([x/35473276.5686 for x in grav])
    mb = np.array([x/1393.1178 for x in mb_mass]) # np.array([x/1345.243 for x in mb_mass])
    rotor = np.array([x/51092.6639 for x in rotor_mass]) # np.array([x/51092.6639 for x in rotor_mass])
    gb = np.array([x/38074.7896 for x in gb_mass]) # np.array([x/35653.1846 for x in gb_mass])
    
    plt.close('all')    
    plt.xlabel('Rotor diameter [m]')
    plt.ylabel('Normalized wrt reference [-]')
     
    plt.plot(d, td, linestyle='-', label='Tip deflection')
    plt.hold(True)
     
    plt.plot(d, flap, marker='o', label='Flapwise stress at root')
    plt.hold(True)
     
    plt.plot(d, grav, linestyle='--', label='Gravity stress at root')
    plt.hold(True)
    
    plt.plot(d, mb, linestyle='-.', label='Main bearing mass')
    plt.hold(True)
     
    plt.plot(d, rotor, linestyle=':', label='Rotor mass')
    plt.hold(True)
     
    plt.plot(d, gb, marker='.', label='Gearbox mass')
    plt.hold(True)
     
    plt.legend(loc='best')
    plt.show()
    #plt.savefig('Plots/scaling.png')
  
    print 'Done in ' + str(time() - start) + ' seconds'        
    



def SA():
    
    from openmdao.api import Problem, view_model
     
    start = time()
    # get and set values of the variables using Problem
    prob = Problem(RNA_usecase())
    prob.setup()
    #view_model(prob, outfile='N2/rna_new.html')
    
    prob['dof.span_airfoil_r'] = [1.5, 10.0]
    prob['dof.span_airfoil_id'] = [0,   7 ]
    prob['dof.rotor_diameter'] = 100.0 
    prob['dof.pot_wind_speed'] = 9.1 #11.4
    prob['dof.hub_height'] = 90.0
    prob['dof.root_chord'] = 3.4
    
    
    prob['dof.hub_radius'] = 1.5
    
    
    
    prob['dof.adjust_pitch'] = 1    
    prob['dof.thickness_factor'] = 1.0
    prob['dof.machine_rating'] = (5000.0/0.944)
    prob['dof.cut_in_speed'] = 3.0
    prob['dof.cut_out_speed'] = 25.0
    
    prob['dof.shaft_angle'] = 0.0
    
    prob['dof.gear_ratio'] = 97.0
    prob['dof.crane'] = 1
    prob['dof.shaft_ratio'] = 0.1
    prob['dof.Np'] = np.array([3.0,3.0,1.0,])
    prob['dof.shrink_disc_mass'] = 5000.0/3.0
    prob['dof.carrier_mass'] = 8000.
    prob['dof.flange_length'] = 0.5
    prob['dof.overhang'] = 5.
    prob['dof.L_rb'] = 1.912
    prob['dof.gearbox_cm_x'] = 0.1
    prob['dof.tower_top_diameter'] = 3.87
    prob['dof.hss_length'] = 1.5
    
    
    cp2 = []
    cp3 = []
    
    for b in range(2,4):
        prob['dof.blade_number'] = b
        for tsr in range(5,15):
            prob['dof.design_tsr'] = tsr
            prob['dof.rotor_speed'] = (tsr*11.1*60)/(50.0*2*pi)
            
            prob.run_model()
            
            if b==2:
                cp2.append(prob['rna.rotor_cp'][0])
            else:
                cp3.append(prob['rna.rotor_cp'][0])
            
            #print prob['rna.tip_deflection']
            #print prob['rna.span_stress_flap'][0]
            print prob['dof.blade_number'], prob['dof.design_tsr'][0], prob['dof.rotor_speed'][0], prob['rna.rotor_cp'][0]


    tsr = np.array(range(5,15))
    cp2 = np.array(cp2)
    cp3 = np.array(cp3)
    
    plt.close('all')    
    plt.xlabel('Tip speed ratio [-]')
    plt.ylabel('Power coefficient [-]')
     
    plt.plot(tsr, cp2, label='2 Blades')
    plt.hold(True)
     
    plt.plot(tsr, cp3, label='3 Blades')
    plt.hold(True)

    plt.legend(loc='best')
    plt.show()
    plt.savefig('Plots/cp_lambda.png')
  
    print 'Done in ' + str(time() - start) + ' seconds' 
    
    
    
    
if __name__ == "__main__":
    TestAeroDyn()