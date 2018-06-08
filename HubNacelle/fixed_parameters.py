# GeneratorSE
C_Cu   = 4.786                  # Unit cost of Copper $/kg
C_Fe   = 0.556                # Unit cost of Iron $/kg
C_Fes  = 0.50139                 # specific cost of Structural_mass
C_PM   = 95.0                    # Specific cost of Magnet 
    

# DriveSE
gear_configuration = 'eep' # ['eep', 'epp', 'eep_2', 'eep_3']
gearbox_stages = 3 # {0 : direct-drive, 1 : single-stage, 3 : three-stage}
ratio_type = 'optimal' # ['optimal', 'empirical']
shaft_type = 'normal' # ['short', 'normal']
drivetrain_design = 'geared' # ['geared', 'single_stage', 'multi_drive', 'pm_direct_drive']
mb1Type = 'CARB' # ['CARB','TRB1','TRB2','SRB','CRB','RB']
mb2Type = 'SRB' 
has_crane = True
uptower_transformer = True

safety_factor = 1.5 # due to steady force

# Material Properties
rho_Fes = 7850. # Structural Steel density [kg/m^3]
rho_Fe = 7700. # [kg/m^3]
rho_Copper = 8900. # [kg/m^3]
rho_PM = 7450.0 # Magnet density [kg/m^3]

# constants
g = 9.81


def beautify(title, val):  
    import numpy as np
    print title + ' = ' + str(repr(np.around(np.array(val),4)))

def beautify_output(output):
    import numpy as np
    for title,val in output.items():
        print title + ' = ' + str(repr(np.around(np.array(val),4)))



