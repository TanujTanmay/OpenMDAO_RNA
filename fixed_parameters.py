# folders
airfoil_folder = 'Airfoils//'
plots_folder = 'Plots//'
aerodyn_folder = 'AeroDyn//'

num_airfoils = 3
num_nodes = 10
num_bins = 30
plot_graphs = True
af_skip_rows = 0

# types
airfoils_db = ['Cylinder1_10Hz.dat', 'Cylinder2_10Hz.dat', 'DU40_A17_10Hz.dat', 'DU35_A17_10Hz.dat', \
                'DU30_A17_10Hz.dat', 'DU25_A17_10Hz.dat', 'DU21_A17_10Hz.dat', 'NACA64_A17_10Hz.dat']
bearing_types = ['CARB','TRB1','TRB2','SRB','CRB','RB']


# wind condition
z0 = 0.0002 # terrain roughness at sea [m]
h_ref = 10 # reference height for potential wind speed [m]
wind_shear = 0.11 # wind shear exponent over sea [-]
k_air = 1.4639e-05 # kinematic viscosity of air [m^2/s]

# constants
g = 9.8

# densities (kg/m^3)
rho_air = 0.9526
rho_blade = 2600
rho_iron = 7200


# other material propeorties
E_blade = 80.0e6 # Young's modulus of glass fiber [Pa]
UTS = 900.0e6 # ultimate tensile strength of steel [Pa]
SN = 9.0 # slope (inverse negative) of SN Curve


# AeroDyn
aerodyn_exe = 'AeroDyn_Driver_x64.exe'
root_name   = 'aerodyn'
driver_ext  = '.dvr'
summary_ext = '.AD.sum'
output_ext  = '.1.out'

aerodyn_spanwise_params = ['Alpha', 'Phi', 'AxInd', 'TnInd', 'Cl', 'Cd', 'Cx', 'Cy', 'Fx', 'Fy', 'Fn', 'Ft', 'Cm']
aerodyn_rotor_params    = ['RtArea', 'RtSpeed',  'RtTSR', 'RtAeroPwr', 'RtAeroCp', 'RtAeroCq', 'RtAeroCt', \
                           'RtAeroFxh', 'RtAeroFyh', 'RtAeroFzh', 'RtAeroMxh', 'RtAeroMyh', 'RtAeroMzh']
aerodyn_timeseries_params = ['Fx', 'Fy']



def beautify(val, small=1):  
    from decimal import Decimal
    
    val = [str(round(x,1)) for x in val]  if small else ['%.1E' % Decimal(x) for x in val]
    beautiful = ', '.join(val) 
    return '[' + beautiful + ']'



