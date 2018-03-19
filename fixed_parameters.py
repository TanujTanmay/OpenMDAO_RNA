absolute_url = 'H:\\GitHub\\OpenMDAO_RNA\\'

num_airfoils = 8
num_nodes = 20
num_bins = 30
plot_graphs = False

airfoils_db = ['cylinder.dat', 'ellipse.dat', 'du-99w-405.csv', 'du-99w-350.csv', \
               'du-97w-300.csv', 'du-91-w2-250.csv', 'du-93w-210.csv', 'naca-64-618.csv']
airfoil_folder = 'Airfoils//'
af_skip_rows = 0

bearing_types = ['CARB','TRB1','TRB2','SRB','CRB','RB']

# wind condition
z0 = 0.0002 # terrain roughness at sea [m]
h_ref = 10 # reference height for potential wind speed [m]
wind_shear = 0.11 # wind shear exponent over sea [-]

# constants
g = 9.8

# densities (kg/m^3)
rho_air = 1.225
rho_blade = 2600
rho_iron = 7200

# other material propeorties
E_blade = 80.0e6 # Young's modulus of glass fiber [Pa]
UTS = 900.0e6 # ultimate tensile strength of steel [Pa]
SN = 9.0 # slope (inverse negative) of SN Curve





def beautify(val, small=1):  
    from decimal import Decimal
    
    val = [str(round(x,1)) for x in val]  if small else ['%.1E' % Decimal(x) for x in val]
    beautiful = ', '.join(val) 
    return '[' + beautiful + ']'