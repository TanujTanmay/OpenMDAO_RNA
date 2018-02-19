absolute_url = 'H:\\GitHub\\OpenMDAO_RNA\\'

num_airfoils = 2
num_nodes = 5
degree_blade_param = 3
num_bins = 2

rho_air = 1.225
z0 = 0.0002 # terrain roughness at sea [m]
h_ref = 10 # reference height for potential wind speed [m]
wind_shear = 0.11 # wind shear exponent over sea [-]


spanwise_params = ['Alpha', 'Phi', 'AxInd', 'TnInd', 'Cl', 'Cd', 'Fx']
airfoils_db = ['cylinder.dat', 'naca-63-210.csv', 'ah.csv']