# folders
import os 
abs_path = os.path.dirname(os.path.realpath(__file__)).replace('\\', '//') + '//'
airfoil_folder = abs_path + 'Airfoils//'
plots_folder = abs_path + 'Plots//'
aerodyn_folder = abs_path + 'AeroDyn//'

# fixed parameters
num_airfoils = 8
num_nodes = 20
num_bins = 31
num_pegged = 3
plot_graphs = False

# model choices
# model_aerodynamic_design    = 'AerodynamicDesignScaling' # ['AerodynamicDesignBetz', 'AerodynamicDesignScaling', 'AerodynamicDesignBurton', 'AerodynamicDesignCoefficients']
# model_structural_design     = 'VariableRadius' # ['VariableChord', 'VariableRadius']
# model_aerodynamics          = 'AerodynamicsSimple' # ['AerodynamicsSimple', 'AerodynamicsAeroDyn'] 
# model_mechanics             = 'RotorMechanicsAnalytical' # ['RotorMechanicsAnalytical']
# model_power_curve           = 'PowerCurve' # ['PowerCurve']

# types
airfoils_db = ['Cylinder1_10Hz.dat', 'Cylinder2_10Hz.dat', 'DU40_A17_10Hz.dat', 'DU35_A17_10Hz.dat', \
                'DU30_A17_10Hz.dat', 'DU25_A17_10Hz.dat', 'DU21_A17_10Hz.dat', 'NACA64_A17_10Hz.dat']



# wind condition
z0 = 0.0002 # terrain roughness at sea [m]
h_ref = 10 # reference height for potential wind speed [m]
wind_shear = 0.00 # wind shear exponent over sea [-]
k_air = 1.4639e-05 # kinematic viscosity of air [m^2/s]

# constants
g = 9.8

# densities (kg/m^3)
rho_air = 1.225
rho_blade = 2600
rho_iron = 7200


# other material propeorties
E_blade = 36.233e9 #50.0e9 # Young's modulus of glass fiber [Pa]
UTS = 399.4e6 #900.0e6 # ultimate tensile strength of steel [Pa]
SN = 9.0 # slope (inverse negative) of SN Curve


# AeroDyn
aerodyn_exe = 'AeroDyn_Driver_x64.exe'
root_name   = 'reference'
driver_ext  = '.dvr'
summary_ext = '.AD.sum'
output_ext  = '.1.out'

itr_max = 100
itr_tol = 5e-5
itr_relax = 0.75
dT = 0.138
Tmax = 12.7935

aerodyn_spanwise_params = ['Alpha', 'Phi', 'AxInd', 'TnInd', 'Cl', 'Cd', 'Cx', 'Cy', 'Fx', 'Fy', 'Fn', 'Ft', 'Cm']
aerodyn_rotor_params    = ['RtArea', 'RtSpeed',  'RtTSR', 'RtAeroPwr', 'RtAeroCp', 'RtAeroCq', 'RtAeroCt', \
                           'RtAeroFxh', 'RtAeroFyh', 'RtAeroFzh', 'RtAeroMxh', 'RtAeroMyh', 'RtAeroMzh']
aerodyn_timeseries_params = ['Fx', 'Fy']

# Simple BEM
bem_spanwise_params = ['chord', 'twist', 'alpha', 'phi', 'aA', 'aT', 'cl', 'cd', 'cp', 'ct']



#############################################################################
##############################  FIXED PARAMS ################################
############################################################################# 
params = ['rotor_cp', 'rotor_ct', 'rated_wind_speed', \
#           'wind_bin', 'elec_power_bin', \
          'span_stress_max', 'tip_deflection', 'blade.root_moment_flap', \
#           'rotor_force', 'rotor_moment', \
          'rotor_mass', 'nacelle_mass', 'cost.cost_rna', \
          'blade.struc_design.blades_mass', 'hub_mass', 'nacelle.gearbox_mass', 'nacelle.lss_mass', \
          'nacelle.main_bearing_mass', 'nacelle.second_bearing_mass', 'nacelle.hss_mass', 'nacelle.generator_mass', \
          'nacelle.mainframe_mass', 'nacelle.yaw_mass', 'nacelle.transformer_mass', \
          'nacelle.hvac_mass', 'nacelle.cover_mass', \
          'cost.cost_hub', 'cost.cost_pitch', 'cost.cost_spinner', 'cost.cost_lss', \
          'cost.cost_main_bearing', 'cost.cost_second_bearing', 'cost.cost_gearbox', \
          'cost.cost_hss', 'cost.cost_generator', 'cost.cost_mainframe', 'cost.cost_yaw', \
          'cost.cost_vs_electronics', 'cost.cost_hvac', 'cost.cost_cover', \
          'cost.cost_electrical', 'cost.cost_controls', 'cost.cost_transformer', 'cost.cost_blades', \
          'cost.cost_hub_system', 'cost.cost_nacelle']

def beautify_old(val, small=1):  
    from decimal import Decimal    
    val = [str(round(x,4)) for x in val]  if small else ['%.4E' % Decimal(x) for x in val]
    beautiful = ', '.join(val) 
    return '[' + beautiful + ']'

def beautify(title, val):  
    import numpy as np
    DEC=4
    
    if len(val) > 1:
        print title + ' = ' + str(repr(np.around(np.array(val),DEC).tolist()))
    else:
        print title + ' = ' + str(round(val[0],DEC))



