import numpy as np
import pandas as pd

from fixed_parameters import airfoils_db, airfoil_folder, af_skip_rows


def ReadAirfoil(airfoil_id):
    airfoil_name = airfoil_folder + airfoils_db[airfoil_id]
    airfoil = pd.read_csv(airfoil_name, skiprows=af_skip_rows)
    
    return airfoil




def ReferenceTurbine():
    file_name = airfoil_folder + 'reference_turbine.csv'
    ref_turbine = pd.read_csv(file_name)
    
    return ref_turbine




def AirfoilProperties(airfoil_id):
    airfoil_name = airfoils_db[airfoil_id]
    airfoil = pd.read_csv(airfoil_folder + airfoil_name, skiprows=af_skip_rows)
    airfoil['Cl_Cd'] = airfoil['Cl']/airfoil['Cd']
    
    cl_opt = airfoil.loc[airfoil['Cl_Cd'].idxmax(), 'Cl']
    alpha_opt = airfoil.loc[airfoil['Cl_Cd'].idxmax(), 'Alpha']
    
    result = {'id' : airfoil_id, \
              'name' : airfoil_name, \
              'cl_opt' : cl_opt, \
              'alpha_opt' : alpha_opt}
    
    return result



def BladeScaling(ref_turbine, mu, chord, thickness_factor, rotor_radius):
    ref_radius = ref_turbine.r.iat[-1]
    ref_chord = np.interp(mu, ref_turbine['mu'], ref_turbine['chord'])
    ref_thickness = np.interp(mu, ref_turbine['mu'], ref_turbine['thickness'])
    ref_mass = np.interp(mu, ref_turbine['mu'], ref_turbine['mass'])
    ref_flap_inertia = np.interp(mu, ref_turbine['mu'], ref_turbine['flap_inertia'])
    ref_edge_inertia = np.interp(mu, ref_turbine['mu'], ref_turbine['edge_inertia'])
    ref_flap_stiff = np.interp(mu, ref_turbine['mu'], ref_turbine['flap_stiffness'])
    ref_edge_stiff = np.interp(mu, ref_turbine['mu'], ref_turbine['edge_stiffness'])
    
    thickness = ref_thickness * (chord/ref_chord)
    
    # when radius is fixed
    mass = ref_mass * ((chord/ref_chord)**1) * thickness_factor
    flap_inertia = ref_flap_inertia * ((chord/ref_chord)**3) * thickness_factor
    edge_inertia = ref_edge_inertia * ((chord/ref_chord)**3) * thickness_factor
    flap_stiff = ref_flap_stiff * ((chord/ref_chord)**3) * thickness_factor
    edge_stiff = ref_edge_stiff * ((chord/ref_chord)**3) * thickness_factor
    
    # when radius varies and constant thickness factor
#     mass = ref_mass * ((chord/ref_chord)**2) * thickness_factor
#     flap_inertia = ref_flap_inertia * ((chord/ref_chord)**4) * thickness_factor
#     edge_inertia = ref_edge_inertia * ((chord/ref_chord)**4) * thickness_factor
#     flap_stiff = ref_flap_stiff * ((chord/ref_chord)**4) * thickness_factor
#     edge_stiff = ref_edge_stiff * ((chord/ref_chord)**4) * thickness_factor
    
    return [thickness, mass, flap_inertia, edge_inertia, flap_stiff, edge_stiff]

        



        
def ReadAirfoilCoordinates(file_name):
    airfoil_name = airfoil_folder + file_name
    airfoil = pd.read_csv(airfoil_name, sep='\s+', skiprows=0, header=0, names=['x', 'y'])
    
    airfoil_upper = airfoil[airfoil.y >= 0].reset_index(drop=True)
    airfoil_lower = airfoil[airfoil.y < 0].reset_index(drop=True)
    return [airfoil_upper, airfoil_lower, airfoil] 



            
         




        
if __name__ == "__main__":
    print AirfoilProperties(0)
    
            