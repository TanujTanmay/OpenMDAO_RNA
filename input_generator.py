import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import re
import random

from fixed_parameters import k_air, rho_air, num_nodes, plot_graphs, \
    airfoils_db, airfoil_folder, aerodyn_folder, plots_folder, aerodyn_exe, \
    aerodyn_spanwise_params, aerodyn_rotor_params, aerodyn_timeseries_params




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
        
    # header
    for record in records:
        buffer = buffer + str(record[0]) + '\t'
    buffer = buffer + '\n'
    
    # units
    for record in records:
        buffer = buffer + str(record[1]) + '\t'
    buffer = buffer + '\n'
    
    # records
    if(isinstance(record[2], list)):
        for i in xrange(len(records[0][2])):
            for record in records:
                buffer = buffer + str(record[2][i]) + '\t'      
            buffer = buffer + '\n'
    else:
        for record in records:
            buffer = buffer + str(record[2]) + '\t'      
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


   
def set_input_driver(root_name, NumBlades, HubRad, HubHt, Overhang, ShftTilt, Precone, \
                     WndSpeed, ShearExp, RotSpd, Pitch, Yaw, dT, Tmax):
    
    driver_file = aerodyn_folder + root_name + '.dvr'
    primary_input = '"' + root_name + '_primary_input.dat"'
    
    input_config = (('Echo',        'Echo input parameters to "<rootname>.ech"?',   True), \
                    ('AD_InputFile','Name of the primary AeroDyn input file',       primary_input)
                    )
    
    turbine_data = (('NumBlades',   'Number of blades (-)',    NumBlades), \
                    ('HubRad',      'Hub radius (m)',          HubRad), \
                    ('HubHt',       'Hub height (m)',          HubHt), \
                    ('Overhang',    'Overhang (m)',            Overhang), \
                    ('ShftTilt',    'Shaft tilt (deg)',        ShftTilt), \
                    ('Precone',     'Blade precone (deg)',     Precone)
                    )
    
    io_setting = (('OutFileRoot',   'Root name for any output files (use "" for .dvr rootname) (-)',    '""'), \
                  ('TabDel',        'When generating formatted output (OutForm=True), make output tab-delimited (fixed-width otherwise) (flag)',    True), \
                  ('OutFmt',        'Format used for text tabular output, excluding the time channel.  Resulting field should be 10 characters. (quoted string)',   '"ES15.6E3"'), \
                  ('Beep',          'Beep on exit (flag)',  True)
                 )
    
    cc_count = len(WndSpeed) if(isinstance(WndSpeed, list)) else 1
    cc_analysis = (('NumCases',     'Number of cases to run',    cc_count), )
    
    # make sure the number of elements in the values column is same as NumCases
    #            Title        Unit     Values (row)
    cases = (  ('WndSpeed', '(m/s)',   WndSpeed), \
               ('ShearExp', '(-)',     ShearExp), \
               ('RotSpd',   '(rpm)',   RotSpd), \
               ('Pitch',    '(deg)',   Pitch), \
               ('Yaw',      '(deg)',   Yaw), \
               ('dT',       '(s)',     dT), \
               ('Tmax',     '(s)',     Tmax)
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


   
def set_primary_input(root_name, BlAFID):
    
    input_file = aerodyn_folder + root_name + '_primary_input.dat'
    blade_file = '"' + root_name + '_blade.dat"'
    
    
    af_folder = '..\\' + airfoil_folder.replace('//', '\\')  # change folder separator to CMD style (slash -> backslash)
    Airfoils = ['"' + af_folder + airfoils_db[x-1] + '"' for x in BlAFID] # BlAFID is in AeroDyn format which starts at 1
    
    
    # select output nodes
    NBlOuts = min(9, num_nodes) # AeroDyn can only have up to 9 output nodes
    
    if num_nodes <= 9:
        BlOutNd = ','.join([str(x) for x in range(1, num_nodes + 1)])
    else:
        # select first 3, last 3 and random 3 in middle
#         BlOutNd = ','.join([str(x) for x in range(1, 4)]) + ',' + \
#                   ','.join([str(x) for x in sorted(random.sample(range(4, num_nodes - 2),3))]) + ',' + \
#                   ','.join([str(x) for x in range(num_nodes - 2, num_nodes + 1)])
                  
        # select first, last and random 7 in middle
        BlOutNd = ','.join([str(x) for x in range(1, 2)]) + ',' + \
                  ','.join([str(x) for x in sorted(random.sample(range(2, num_nodes), 7))]) + ',' + \
                  ','.join([str(x) for x in range(num_nodes, num_nodes + 1)])          
    
    
    
    
    OutList = []
    OutList.append(aerodyn_rotor_params)
    
    for i in range(1, NBlOuts+1):
        OutList.append(['B1N' + str(i) + x for x in aerodyn_spanwise_params])
        
    for i in range(len(OutList)):
        outlist = ', '.join(OutList[i])
        OutList[i] = '"' + outlist + '"'    
    
    general_options = (('Echo',     'Echo the input to ""<rootname>.AD.ech""?  (flag)',    True), \
                    ('DTAero',      'Time interval for aerodynamic calculations {or ""default""} (s)',    '"default"'), \
                    ('WakeMod',     'Type of wake/induction model (switch) {0=none, 1=BEMT}',    1), \
                    ('AFAeroMod',   'Type of blade airfoil aerodynamics model (switch) {1=steady model, 2=Beddoes-Leishman unsteady model} [must be 1 when linearizing]',    1), \
                    ('TwrPotent',   'Type tower influence on wind based on potential flow around the tower (switch) {0=none, 1=baseline potential flow, 2=potential flow with Bak correction}',    0), \
                    ('TwrShadow',   'Calculate tower influence on wind based on downstream tower shadow? (flag)',    False), \
                    ('TwrAero',     'Calculate tower aerodynamic loads? (flag)',    False), \
                    ('FrozenWake',  'Assume frozen wake during linearization? (flag) [used only when WakeMod=1 and when linearizing]',    False), \
                    ('CavitCheck',  'Perform cavitation check? (flag)',    False)
                    )
    
    env_conditions = (('AirDens',    'Air density (kg/m^3)',                                                    rho_air), \
                      ('KinVisc',    'Kinematic air viscosity (m^2/s)',                                         k_air), \
                      ('SpdSound',   'Speed of sound (m/s)',                                                    335), \
                      ('Patm',       'Atmospheric pressure (Pa) [used only when CavitCheck=True]',              103500), \
                      ('Pvap',       'Vapour pressure of fluid (Pa) [used only when CavitCheck=True]',          1700), \
                      ('FluidDepth', 'Water depth above mid-hub height (m) [used only when CavitCheck=True]',   0.1)
                    )
    
    bem_options = (('SkewMod',    'Type of skewed-wake correction model (switch) {1=uncoupled, 2=Pitt/Peters, 3=coupled} [used only when WakeMod=1]',   1), \
                    ('TipLoss',   'Use the Prandtl tip-loss model? (flag) [used only when WakeMod=1]',                                                  True), \
                    ('HubLoss',   'Use the Prandtl hub-loss model? (flag) [used only when WakeMod=1]',                                                  True), \
                    ('TanInd',    'Include tangential induction in BEMT calculations? (flag) [used only when WakeMod=1]',                               True), \
                    ('AIDrag',    'Include the drag term in the axial-induction calculation? (flag) [used only when WakeMod=1]',                        True), \
                    ('TIDrag',    'Include the drag term in the tangential-induction calculation? (flag) [used only when WakeMod=1 and TanInd=TRUE]',   True), \
                    ('IndToler',  'Convergence tolerance for BEMT nonlinear solve residual equation {or ""default""} (-) [used only when WakeMod=1]',   0.00005), \
                    ('MaxIter',   'Maximum number of iteration steps (-) [used only when WakeMod=1]',                                                   100)
                )
    
    unsteady_options = (('UAMod',   'Unsteady Aero Model Switch (switch) {1=Baseline model (Original), 2=Gonzalezs variant (changes in Cn,Cc,Cm), 3=Minemma/Pierce variant (changes in Cc and Cm)} [used only when AFAeroMod=2]',    3), \
                        ('FLookup', 'Flag to indicate whether a lookup for f_prime will be calculated (TRUE) or whether best-fit exponential equations will be used (FALSE); if FALSE S1-S4 must be provided in airfoil input files (flag) [used only when AFAeroMod=2]', True)
                        )
    
    airfoil_info = (('InCol_Alfa',  'The column in the airfoil tables that contains the angle of attack (-)',     1), \
                    ('InCol_Cl',    'The column in the airfoil tables that contains the lift coefficient (-)',    2), \
                    ('InCol_Cd',    'The column in the airfoil tables that contains the drag coefficient (-)',    3), \
                    ('InCol_Cm',    'The column in the airfoil tables that contains the pitching-moment coefficient; use zero if there is no Cm column (-)',    4), \
                    ('InCol_Cpmin', 'The column in the airfoil tables that contains the Cpmin coefficient; use zero if there is no Cpmin column (-)',           0), \
                    ('NumAFfiles',  'Number of airfoil files used (-)',    len(Airfoils))
                )
    
    airfoil_files = ('AFNames',     'Airfoil file names (NumAFfiles lines) (quoted strings)', Airfoils)
    
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
    
    
    output_prop = (('SumPrint',   'Generate a summary file listing input options and interpolated properties to "<rootname>.AD.sum"?  (flag)',    True), \
                   ('NBlOuts',    'Number of blade node outputs [0 - 9] (-)',       NBlOuts), \
                   ('BlOutNd',    'Blade nodes whose values will be output  (-)',   BlOutNd), \
                   ('NTwOuts',    'Number of tower node outputs [0 - 9]  (-)',      0), \
                   ('TwOutNd',    'Tower nodes whose values will be output  (-)',   ''), \
            )
    
    
    output_params = ('OutList',     'The next line(s) contains a list of output parameters.  See OutListParameters.xlsx for a listing of available output channels, (-)', \
                      OutList
                     )
    
    gen_primary_input(input_file, general_options, env_conditions, bem_options, unsteady_options, \
                      airfoil_info, airfoil_files, rotor_prop, tower_prop, tower_nodes, output_prop, output_params)










    
    
    
    
    
    
# ------------------- BLADE FILE ----------------------------    
def gen_blade_file(blade_file, blade_properties, blade_span):
    
    # create the file to write
    f = open(blade_file, 'w')
    
    
    # Section 1: Definition
    f.write(buffer_header('AERODYN v15.00.* BLADE DEFINITION INPUT FILE'))
    f.write('NREL 5.0 MW offshore baseline aerodynamic blade input properties \n')
    
    
    # Section 2: Blade Properties
    f.write(buffer_header('Blade Properties'))
    f.write(buffer_records(blade_properties))
    f.write(buffer_table(blade_span))
    
    
    # close the file
    f.close()


   
def set_blade_file(root_name, BlSpn, BlTwist, BlChord, BlAFID):
    
    blade_file = aerodyn_folder + root_name + '_blade.dat'
    BlCrvAC = [0.0] * num_nodes
    BlSwpAC = [0.0] * num_nodes
    BlCrvAng = [0.0] * num_nodes
    
    blade_properties = (('NumBlNds',     'Number of blade nodes used in the analysis (-)',    num_nodes), )
    
    # make sure the number of elements in the values column is same as NumBlNds
    #                Title      Unit     Values (nodewise)
    blade_span = ( ('BlSpn',    '(m)',   BlSpn), \
                   ('BlCrvAC',  '(m)',   BlCrvAC), \
                   ('BlSwpAC',  '(m)',   BlSwpAC), \
                   ('BlCrvAng', '(deg)', BlCrvAng), \
                   ('BlTwist',  '(deg)', BlTwist), \
                   ('BlChord',  '(m)',   BlChord), \
                   ('BlAFID',   '(-)',   [int(x) for x in BlAFID])
                   )
    
    
    
    gen_blade_file(blade_file, blade_properties, blade_span)
    
    
    









    
# ------------------- AIRFOIL FILE ----------------------------         

def read_airfoil(airfoil_file, file_type):
    
    if file_type == 'AeroDyn':
        with open(airfoil_file) as f:
            for line in f:
                # search for Reynold's number
                reynolds_match = re.search('(?<=Reynolds number,)\w+', line)
                if reynolds_match:
                    reynolds_number = reynolds_match.group(0)
                
                # skip rows until you reach the main table
                table_start = re.match('![\s]+Alpha', line)
                if table_start:
                    break
                
            table = pd.read_table(f, delim_whitespace=True, index_col=False, header=None, \
                                  names=['Alpha','Cl','Cd', 'Cm'], usecols=[0, 1, 2, 3], \
                                  skiprows=1, skipfooter=3, engine='python')
        
    else:
        with open(airfoil_file) as f:
            for line in f:
                # search for Reynold's number
                reynolds_match = re.search('(?<=Reynolds number,)\w+', line)
                if reynolds_match:
                    reynolds_number = reynolds_match.group(0)
                
                # skip rows until you reach the main table
                table_start = re.match('Alpha', line)
                if table_start:
                    break
                
            table = pd.read_table(f, sep=',', index_col=False, header=None, \
                                  names=['Alpha','Cl','Cd', 'Cm'], usecols=[0, 1, 2, 4], \
                                  skiprows=0, skipfooter=0, engine='python')
            
            
    
    alpha =  table.as_matrix(columns=['Alpha'])
    alpha = alpha.T[0]
    
    cl =  table.as_matrix(columns=['Cl'])
    cl = cl.T[0]
    
    cd =  table.as_matrix(columns=['Cd'])
    cd = cd.T[0]
    
    cm =  table.as_matrix(columns=['Cm'])
    cm = cm.T[0]
    
    result = [float(reynolds_number), alpha, cl, cd, cm]    
    return result



def gen_airfoil_file(airfoil_file, polar):
    
    # create the file to write
    f = open(airfoil_file, 'w')
    
    
    # Section 1: Definition
    f.write('! ------------ AirfoilInfo v1.01.x Input File ----------------------------------\n')
    f.write('! NREL 5.0 MW offshore baseline aerodynamic blade input properties \n')
    f.write('! ------------------------------------------------------------------------------\n')
    
    # Section 2: Airfoil Parameters
    f.write('"DEFAULT"    InterpOrd   \t ! Interpolation order to use for quasi-steady table lookup {1=linear; 3=cubic spline; "default"} [default=3]\n')
    f.write('1            NonDimArea  \t ! The non-dimensional area of the airfoil (area/chord^2) (set to 1.0 if unsure or unneeded)\n')
    f.write('0            NumCoords   \t ! The number of coordinates in the airfoil shape file.  Set to zero if coordinates not included.\n')
    f.write('! ......... x-y coordinates are next if NumCoords > 0 .............\n')
    f.write('1            NumTabs     \t ! Number of airfoil tables in this file.  Each table must have lines for Re and Ctrl.\n')
    
    # Section 3: Polar Parameters
    f.write('! ------------------------------------------------------------------------------\n')
    f.write('! data for table 1\n')
    f.write('! ------------------------------------------------------------------------------\n')
    f.write(str(polar.Re/1e6) + '\t Re \t ! Reynolds number in millions\n')
    f.write('0 \t Ctrl \t ! Control setting (must be 0 for current AirfoilInfo)\n')
    f.write('False \t InclUAdata \t ! Is unsteady aerodynamics data included in this table? If TRUE, then include 30 UA coefficients below this line)\n')
    f.write('!........................................\n')
    
    # Section 4: Polar Coordinates
    f.write('! Table of aerodynamics coefficients\n')
    f.write(str(len(polar.alpha)) + '\t NumAlf \t ! Number of data lines in the following table\n')
    f.write('! \t Alpha \t Cl \t Cd \t Cm\n')
    f.write('! \t (deg) \t (-) \t (-) \t (-)\n')
    
    for i in xrange(len(polar.alpha)):
        f.write(format(round(polar.alpha[i], 2)) + '\t' + \
                format(round(polar.cl[i], 4)) + '\t' + \
                format(round(polar.cd[i], 4)) + '\t' + \
                format(round(polar.cm[i], 4)) + '\n')
    
    
    # close the file
    f.close()


def set_airfoil_file(airfoil_file):
    #from airfoilprep import Polar, Airfoil
    
    airfoil_url = 'Airfoils/' + airfoil_file
    output_file = 'corr_' + airfoil_file
    output_url = 'Airfoils/' + output_file
    #aspect_ratio = 17
    
    [Re, alpha, cl, cd, cm]  = read_airfoil(airfoil_url, 'Non_AeroDyn')
    p1 = Polar(Re, alpha, cl, cd, cm)
    
    af = Airfoil([p1])
    #af = af.extrapolate(max(cd), aspect_ratio, min(cd))
    
    #af = af.correction3D(0.5, 0.15, 9)
    #af = af.interpToCommonAlpha(np.arange(-180, 181))
    gen_airfoil_file(output_url, af.polars[0])
    return output_file
    
    
        













# ------------------- OUTPUT FILE ----------------------------  

def read_output(summary_file, output_file, BlSpn):

    # get list of output nodes
    #print 'Reading: ' + summary_file
    with open(summary_file) as f:
        # skip rows until you reach the main table
        for line in f:            
            search_out_nodes = re.search('(\d*)\s+(NBlOuts)', line)
            if search_out_nodes:
                num_out_nodes = int(search_out_nodes.group(1))
                break
            
        nodes = pd.read_table(f, sep='\s+', index_col=False,  header=None, \
                              names=['output_node', 'analysis_node'], usecols=[0, 1], \
                              skiprows=2, nrows=num_out_nodes+1, engine='python')
            
    nodes = nodes.drop([0]).reset_index(drop=True) # drop first row that has the units
    nodes = nodes.astype(int) # convert all data to int
    output_nodes = nodes['analysis_node'].tolist()
    
    
    # read the AeroDyn output
    #print 'Reading: ' + output_file
    with open(output_file) as f:
        # skip rows until you reach the main table
        for line in f:            
            table_start = re.match('[\s]*Case', line)
            if table_start:
                break
            
        output = pd.read_table(f, sep='\s+', index_col=False,  header=0, \
                              skiprows=1, skipfooter=0, engine='python')
            
    output = output.drop([0]).reset_index(drop=True) # drop first row that has the units
    output = output.astype(float) # convert all data to float
    
    

    # get radial distance of output nodes
    output_node_r = []
    for i in output_nodes:
        output_node_r.append(BlSpn[i-1])
    

    
    # save rotor parameters at t=0
    rotor = {}
    for param in aerodyn_rotor_params:
        rotor[param] = output.ix[0, param]
         
    
    # save spanwise parameters at t=0 
    spanwise = {}
    for param in aerodyn_spanwise_params:
        param_list  = [] # to store the values at all nodes as defined in input
        output_list = [] # to store the values at AeroDyn output nodes
        
        # read the results from the output blade nodes
        for i in range(1, len(output_nodes) + 1):
            output_list.append(output.ix[0, 'B1N' + str(i) + param])    
          
        # interpolate the results for all blade nodes as defined in input
        for i in range(1, num_nodes + 1):
            if i in output_nodes:
                # get the values directly from output file
                param_val = output_list[output_nodes.index(i)]
            else:
                # interpolate the values
                param_val = np.interp(BlSpn[i-1], output_node_r, output_list)
            param_list.append(param_val)    
          
        spanwise[param] = param_list
        
        # save the plots
        if plot_graphs:
            plt.close('all')
            plt.plot(BlSpn, param_list, label=param)
            plt.xlabel('Span [m]')
            plt.ylabel(param)
            #plt.show()
            plt.savefig(plots_folder + 'Spanwise_' + param + '.png')
         
         
     
    # save time series parameters
    timeseries = []  
    time =  output.ix[:, 'Time'] 
    timeseries.append(time)
     
    for param in aerodyn_timeseries_params:
        if plot_graphs:
            plt.close('all')
            
        for i in range(1, len(output_nodes) + 1):
            param_list = output.ix[:, 'B1N' + str(i) + param]
            timeseries.append(param_list)    
     
            # save the plots
            if plot_graphs:
                plt.plot(time, param_list, label='Node '+str(i))     
    
        if plot_graphs:
            plt.xlabel('Time [s]')
            plt.ylabel(param)
            plt.legend()
            #plt.show()
            plt.savefig(plots_folder + 'TS_' + param + '.png')                   
     
    result = [rotor, spanwise, timeseries] 
    return result


    


    
if __name__ == "__main__":
    
    
    import os
    from time import time
    
    start = time()
    
     
    BlSpn =   [0,    1.36665,    4.09995,    6.83325,    10.24995,    14.34995,    18.44995,    22.54995,    26.64995,    30.74995,    34.84995,    38.94995,    43.04995,    47.14995,    51.24995,    54.66665,    57.39995,    60.13325,    61.4999] 
    BlTwist = [13.308,    13.308,    13.308,    13.308,    13.308,    11.48,    10.162,    9.011,    7.795,    6.544,    5.361,    4.188,    3.125,    2.319,    1.526,    0.863,    0.37,    0.106,    0.106]
    BlChord = [3.542,    3.542,    3.854,    4.167,    4.557,    4.652,    4.458,    4.249,    4.007,    3.748,    3.502,    3.256,    3.01,    2.764,    2.518,    2.313,    2.086,    1.419,    1.419]
    BlAFID =  [1, 1, 1, 2, 3, 4, 4, 5, 6, 6, 7, 7, 8, 8, 8, 8, 8, 8, 8]
    
    root_name = 'reference'
    set_input_driver(root_name, 3, 1.5, 90, -5.0191, -5, -2.5, \
                         7, 0, 7.43, 0, 0, 0.138, 12.7935)
    set_primary_input(root_name, BlAFID)
    set_blade_file(root_name, BlSpn, BlTwist, BlChord, BlAFID)
  
    command = './/' + aerodyn_folder + aerodyn_exe + ' ' + aerodyn_folder + root_name + '.dvr'
    command = command.replace('//', '\\') # change folder separator to CMD style (slash -> backslash)
    print 'Executing: ' + command     
    os.system(command)
    
    summary_file = aerodyn_folder + 'test.AD.sum'
    output_file = aerodyn_folder + 'test.1.out'
    [rotor, spanwise, timeseries] = read_output(summary_file, output_file, BlSpn)

    print rotor['RtTSR'], rotor['RtAeroCp'], rotor['RtAeroCt']
    print [rotor['RtAeroMxh'], rotor['RtAeroMyh'], rotor['RtAeroMzh']]
    fxy = [[x,y] for (x,y) in zip(spanwise['Fx'], spanwise['Fy'])]
    print fxy
    print np.array(fxy).shape
    print 'Done in ' + str(time() - start) + ' seconds'
    
    
    