
# ------------------- COMMON ROUTINES ----------------------------
def buffer_header(header):
    buffer = '----- ' + header + ' '
    buffer = buffer + '-' * (80 - len(buffer)) + '\n'
    
    return buffer
    

def buffer_records(records):
    
    buffer = ''
    for record in records:
        buffer = buffer + str(record[2]) + '\t' + str(record[0]) + '\t - '+ str(record[1]) + '\n'
    
    return buffer


def buffer_table(records):
    
    buffer = ''
    num_records = len(records[0][2])
        
    # header
    for record in records:
        buffer = buffer + str(record[0]) + '\t'
    buffer = buffer + '\n'
    
    # units
    for record in records:
        buffer = buffer + str(record[1]) + '\t'
    buffer = buffer + '\n'
    
    # records
    for i in xrange(num_records):
        for record in records:
            buffer = buffer + str(record[2][i]) + '\t'      
        buffer = buffer + '\n'
            
    return buffer


def buffer_list(records, is_inline):
    
    if is_inline:
        buffer = str(records[2][0])
        start  = 1
    else:
        buffer = ''
        start = 0
        
    buffer = buffer + '\t' + records[0] + '\t - ' + records[1] + '\n'
    
    for i in range(start, len(records[2])):
        buffer = buffer + records[2][i] + '\n'
        
    return buffer








# ------------------- DRIVER FILE ----------------------------
def gen_input_driver(driver_file, input_config, turbine_data, io_setting, cc_analysis, cases):
    
    # create the file to write
    f = open(driver_file, 'w')
    
    
    # Section 1: AeroDyn Driver v1.00.x Input File
    f.write(buffer_header('AeroDyn Driver v1.00.x Input File'))
    f.write('Driver file for AeroDyn simulation for Rotor Aerodynamics \n')
    
    
    # Section 2: Input Configuration
    f.write(buffer_header('Input Configuration'))
    f.write(buffer_records(input_config))
    
    
    # Section 3: Turbine Data
    f.write(buffer_header('Turbine Data'))
    f.write(buffer_records(turbine_data))
    
    # Section 4: I/O Settings
    f.write(buffer_header('I/O Settings'))
    f.write(buffer_records(io_setting))
    
    # Section 5: Combined-Case Analysis
    f.write(buffer_header('Combined-Case Analysis'))
    f.write(buffer_records(cc_analysis))
    f.write(buffer_table(cases))
    
    
    # close the file
    f.close()


   
def set_input_driver():
    
    driver_file = 'test.dvr'
    primary_input = '"test_primary_input.dat"'
    
    input_config = (('Echo',        'Echo input parameters to "<rootname>.ech"?',   False), \
                    ('AD_InputFile','Name of the primary AeroDyn input file',       primary_input)
                    )
    
    turbine_data = (('NumBlades',   'Number of blades (-)',    3), \
                    ('HubRad',      'Hub radius (m)',          1.5), \
                    ('HubHt',       'Hub height (m)',          90), \
                    ('Overhang',    'Overhang (m)',            -5.0191), \
                    ('ShftTilt',    'Shaft tilt (deg)',        -5), \
                    ('Precone',     'Blade precone (deg)',     -2.5)
                    )
    
    io_setting = (('OutFileRoot',   'Root name for any output files (use "" for .dvr rootname) (-)',    '""'), \
                  ('TabDel',        'When generating formatted output (OutForm=True), make output tab-delimited (fixed-width otherwise) (flag)',    'True'), \
                  ('OutFmt',        'Format used for text tabular output, excluding the time channel.  Resulting field should be 10 characters. (quoted string)',   '"ES15.6E3"'), \
                  ('Beep',          'Beep on exit (flag)',  'True')
                 )
    
    cc_analysis = (('NumCases',     'Number of cases to run',    1), )
    
    # make sure the number of elements in the values column is same as NumCases
    #            Title        Unit     Values (row)
    cases = (  ('WndSpeed', '(m/s)',   [7.0]), \
               ('ShearExp', '(-)',     [0.0]), \
               ('RotSpd',   '(rpm)',   [10.0]), \
               ('Pitch',    '(deg)',   [0.0]), \
               ('Yaw',      '(deg)',   [0.0]), \
               ('dT',       '(s)',     [0.138]), \
               ('Tmax',     '(s)',     [12.7935])
            )
    
    
    
    gen_input_driver(driver_file, input_config, turbine_data, io_setting, cc_analysis, cases)
    
    
    
    



# ------------------- PRIMARY INPUT FILE ----------------------------
def gen_primary_input(input_file, general_options, env_conditions, bem_options, unsteady_options, \
                      airfoil_info, airfoil_files, rotor_prop, tower_prop, tower_nodes, output_prop, output_params):
    
    # create the file to write
    f = open(input_file, 'w')
    
    
    # Section 1: Definition
    f.write(buffer_header('AERODYN v15.04.* INPUT FILE'))
    f.write('NREL 5.0 MW offshore baseline aerodynamic input properties. \n')
    
    
    # Section 2: General Options
    f.write(buffer_header('General Options'))
    f.write(buffer_records(general_options))
    
    
    # Section 3: Environmental Conditions
    f.write(buffer_header('Environmental Conditions'))
    f.write(buffer_records(env_conditions))
    
    # Section 4: Blade-Element/Momentum Theory Options
    f.write(buffer_header('Blade-Element/Momentum Theory Options [used only when WakeMod=1]'))
    f.write(buffer_records(bem_options))
    
    # Section 5: Beddoes-Leishman Unsteady Airfoil Aerodynamics Options
    f.write(buffer_header('Beddoes-Leishman Unsteady Airfoil Aerodynamics Options [used only when AFAeroMod=2]'))
    f.write(buffer_records(unsteady_options))
    
    # Section 6: Airfoil Information
    f.write(buffer_header('Airfoil Information'))
    f.write(buffer_records(airfoil_info))
    f.write(buffer_list(airfoil_files, True))
    
    # Section 7: Rotor/Blade Properties
    f.write(buffer_header('Rotor/Blade Properties'))
    f.write(buffer_records(rotor_prop))
    
    # Section 8: Tower Influence and Aerodynamics
    f.write(buffer_header('Tower Influence and Aerodynamics'))
    f.write(buffer_records(tower_prop))
    f.write(buffer_table(tower_nodes))
    
    # Section 9: Outputs
    f.write(buffer_header('Outputs'))
    f.write(buffer_records(output_prop))
    f.write(buffer_list(output_params, False))
    
    # close the file
    f.write('END of input file (the word "END" must appear in the first 3 columns of this last OutList line) \n')
    f.write('-' * 80)
    f.close()


   
def set_primary_input():
    
    input_file = 'test_primary_input.dat'
    blade_file = '"test_blade.dat"'
    
    general_options = (('Echo',     'Echo the input to ""<rootname>.AD.ech""?  (flag)',    'FALSE'), \
                    ('DTAero',      'ime interval for aerodynamic calculations {or ""default""} (s)',    '"default"'), \
                    ('WakeMod',     'Type of wake/induction model (switch) {0=none, 1=BEMT}',    1), \
                    ('AFAeroMod',   'Type of blade airfoil aerodynamics model (switch) {1=steady model, 2=Beddoes-Leishman unsteady model} [must be 1 when linearizing]',    1), \
                    ('TwrPotent',   'Type tower influence on wind based on potential flow around the tower (switch) {0=none, 1=baseline potential flow, 2=potential flow with Bak correction}',    0), \
                    ('TwrShadow',   'Calculate tower influence on wind based on downstream tower shadow? (flag)',    'False'), \
                    ('TwrAero',     'Calculate tower aerodynamic loads? (flag)',    'False'), \
                    ('FrozenWake',  'Assume frozen wake during linearization? (flag) [used only when WakeMod=1 and when linearizing]',    'False'), \
                    ('CavitCheck',  'Perform cavitation check? (flag)',    'False')
                    )
    
    env_conditions = (('AirDens',    'Air density (kg/m^3)',                                                    0.9526), \
                      ('KinVisc',    'Kinematic air viscosity (m^2/s)',                                         0.14639), \
                      ('SpdSound',   'Speed of sound (m/s)',                                                    335), \
                      ('Patm',       'Atmospheric pressure (Pa) [used only when CavitCheck=True]',              103500), \
                      ('Pvap',       'Vapour pressure of fluid (Pa) [used only when CavitCheck=True]',          1700), \
                      ('FluidDepth', 'Water depth above mid-hub height (m) [used only when CavitCheck=True]',   0.1)
                    )
    
    bem_options = (('SkewMod',    'Type of skewed-wake correction model (switch) {1=uncoupled, 2=Pitt/Peters, 3=coupled} [used only when WakeMod=1]',   1), \
                    ('TipLoss',   'Use the Prandtl tip-loss model? (flag) [used only when WakeMod=1]',                                                  True), \
                    ('HubLoss',   'Use the Prandtl hub-loss model? (flag) [used only when WakeMod=1]',                                                  True), \
                    ('TanInd',    'Include tangential induction in BEMT calculations? (flag) [used only when WakeMod=1]',                               True), \
                    ('AIDrag',    'Include the drag term in the axial-induction calculation? (flag) [used only when WakeMod=1]',                        False), \
                    ('TIDrag',    'Include the drag term in the tangential-induction calculation? (flag) [used only when WakeMod=1 and TanInd=TRUE]',   False), \
                    ('IndToler',  'Convergence tolerance for BEMT nonlinear solve residual equation {or ""default""} (-) [used only when WakeMod=1]',   0.00005), \
                    ('MaxIter',   'Maximum number of iteration steps (-) [used only when WakeMod=1]',                                                   10000)
                )
    
    unsteady_options = (('UAMod',   'Unsteady Aero Model Switch (switch) {1=Baseline model (Original), 2=Gonzalezs variant (changes in Cn,Cc,Cm), 3=Minemma/Pierce variant (changes in Cc and Cm)} [used only when AFAeroMod=2]',    3), \
                        ('FLookup', 'Flag to indicate whether a lookup for f_prime will be calculated (TRUE) or whether best-fit exponential equations will be used (FALSE); if FALSE S1-S4 must be provided in airfoil input files (flag) [used only when AFAeroMod=2]', True)
                        )
    
    airfoil_info = (('InCol_Alfa',  'The column in the airfoil tables that contains the angle of attack (-)',     1), \
                    ('InCol_Cl',    'The column in the airfoil tables that contains the lift coefficient (-)',    2), \
                    ('InCol_Cd',    'The column in the airfoil tables that contains the drag coefficient (-)',    3), \
                    ('InCol_Cm',    'The column in the airfoil tables that contains the pitching-moment coefficient; use zero if there is no Cm column (-)',    4), \
                    ('InCol_Cpmin', 'The column in the airfoil tables that contains the Cpmin coefficient; use zero if there is no Cpmin column (-)',           0), \
                    ('NumAFfiles',  'Number of airfoil files used (-)',    8)
                )
    
    airfoil_files = ('AFNames',     'Airfoil file names (NumAFfiles lines) (quoted strings)', \
                         ('"airfoils\Cylinder1_1000Hz.dat"', \
                          '"airfoils\Cylinder2_1000Hz.dat"', \
                          '"airfoils\DU40_A17_1000Hz.dat"', \
                          '"airfoils\DU35_A17_1000Hz.dat"', \
                          '"airfoils\DU30_A17_1000Hz.dat"', \
                          '"airfoils\DU25_A17_1000Hz.dat"', \
                          '"airfoils\DU21_A17_1000Hz.dat"', \
                          '"airfoils\NACA64_A17_1000Hz.dat"'
                          )
                     )
    
    rotor_prop = (('UseBlCm',       'Include aerodynamic pitching moment in calculations?  (flag)',    True), \
                  ('ADBlFile(1)',   'Name of file containing distributed aerodynamic properties for Blade #1 (-)',    blade_file), \
                  ('ADBlFile(2)',   'Name of file containing distributed aerodynamic properties for Blade #2 (-) [unused if NumBl < 2]',    blade_file), \
                  ('ADBlFile(3)',   'Name of file containing distributed aerodynamic properties for Blade #3 (-) [unused if NumBl < 3]',    blade_file)
            )
    
    tower_prop = (('NumTwrNds',    'Number of tower nodes used in the analysis  (-) [used only when TwrPotent/=0, TwrShadow=True, or TwrAero=True]',    0), )
    
    tower_nodes = (('TwrElev', '(m)',   []), \
                   ('TwrDiam', '(m)',   []), \
                   ('TwrCd',   '(-)',   [])
                   )
    
    
    output_prop = (('SumPrint',   'Generate a summary file listing input options and interpolated properties to "<rootname>.AD.sum"?  (flag)',    False), \
                   ('NBlOuts',    'Number of blade node outputs [0 - 9] (-)',       5), \
                   ('BlOutNd',    'Blade nodes whose values will be output  (-)',   '4,5,6,8,9'), \
                   ('NTwOuts',    'Number of tower node outputs [0 - 9]  (-)',      0), \
                   ('TwOutNd',    'Tower nodes whose values will be output  (-)',   ''), \
            )
    
    
    output_params = ('OutList',     'The next line(s) contains a list of output parameters.  See OutListParameters.xlsx for a listing of available output channels, (-)', \
                         ('"B1N1Alpha, B1N1Cl, B1N2Alpha, B1N2Cl"', \
                          '"B1N2Cd, B1N2Cm, B2N8Cl"', \
                         '"RtAeroCp, RtAeroCq, RtAeroCt, RtTSR"'
                          )
                     )
    
    gen_primary_input(input_file, general_options, env_conditions, bem_options, unsteady_options, \
                      airfoil_info, airfoil_files, rotor_prop, tower_prop, tower_nodes, output_prop, output_params)










    
    
    
    
    
    
# ------------------- BLADE FILE ----------------------------    
def gen_blade_file(blade_file, blade_properties, blade_span):
    
    # create the file to write
    f = open(blade_file + '.dat', 'w')
    
    
    # Section 1: Definition
    f.write(buffer_header('AERODYN v15.00.* BLADE DEFINITION INPUT FILE'))
    f.write('NREL 5.0 MW offshore baseline aerodynamic blade input properties \n')
    
    
    # Section 2: Blade Properties
    f.write(buffer_header('Blade Properties'))
    f.write(buffer_records(blade_properties))
    f.write(buffer_table(blade_span))
    
    
    # close the file
    f.close()


   
def set_blade_file():
    
    blade_file = 'test_blade'
    
    blade_properties = (('NumBlNds',     'Number of blade nodes used in the analysis (-)',    13), )
    
    # make sure the number of elements in the values column is same as NumBlNds
    #                Title        Unit     Values (row)
    blade_span = ( ('BlSpn',    '(m)',   [00.00, 04.10, 06.83, 10.25, 14.35, 18.45, 22.55, 26.65, 30.75, 34.85, 38.95, 43.05, 61.50]), \
                   ('BlCrvAC',  '(m)',   [00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00]), \
                   ('BlSwpAC',  '(m)',   [00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00]), \
                   ('BlCrvAng', '(deg)', [00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00, 00.00]), \
                   ('BlTwist',  '(deg)', [13.31, 13.31, 13.31, 13.31, 11.48, 10.16, 09.01, 07.80, 06.54, 05.36, 04.19, 03.12, 00.11]), \
                   ('BlChord',  '(m)',   [03.54, 03.85, 04.17, 04.56, 04.65, 04.46, 04.25, 04.01, 03.75, 03.50, 03.26, 03.01, 01.42]), \
                   ('BlAFID',   '(-)',   [    1,     1,     2,     3,     4,     4,     5,     6,     6,     7,     7,     8,     8])
            )
    
    
    
    gen_blade_file(blade_file, blade_properties, blade_span)
    
    
    
    
        
    


    
if __name__ == "__main__":
    set_input_driver()
    set_blade_file()
    set_primary_input()