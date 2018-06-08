import matplotlib.pyplot as plt

def plot_pc():
    # plots
#         fig = plt.figure()
#         
#         x1 = fig.add_subplot(221)
#         x1.set_xlabel('Number of integrated disciplines [-]')
#         x1.set_ylabel('Normalized mass [-]')
#         
#         x2 = fig.add_subplot(222)
#         x2.set_xlabel('Number of integrated disciplines [-]')
#         x2.set_ylabel('Thickness factor [-]')
#         
#         x3 = fig.add_subplot(223)
#         y3 = x3.twinx()
#         x3.set_xlabel('Number of integrated disciplines [-]')
#         x3.set_ylabel('Power coefficient[-]')
#         y3.set_ylabel('Tip speed ratio [-]')
#         
#         x4 = fig.add_subplot(224)
#         x4.set_xlabel('Number of integrated disciplines [-]')
#         x4.set_ylabel('LCOE [cents/kWh]')
#         
#         x = [1,2,3,4]
#         tf = [1,    0.486,    0.641,    1.0955]
#         tsr = [8.088,    5.711,    8.157,    8.01]
#         cp = [0.5098,    0.4334,    0.4993,    0.5061]
#         lcoe = [8.1089,    8.476,    8.1416,    8.1208]
#         tsr_ref = 7.0
#         lcoe_ref = 8.1458
#         cp_ref = 0.4967
#         
#         rotor_mass = [0.9486,    0.5238,    0.7008,    1.0687]
#         gb_mass = [0.873,    1.1712,    0.8597,    0.8794]
#         rna_mass = [0.9423,    0.8393,    0.8226,    1.0031]
#         
#         x1.plot(x, rotor_mass, label='Rotor mass')
#         x1.plot(x, gb_mass, label='Gearbox mass')
#         x1.plot(x, rna_mass, label='RNA mass')
#         
#         x2.plot(x, tf)
#         
#         x3.plot(x, cp)
#         y3.plot(x, tsr)
#         
#         x4.plot(x, lcoe)
#         
#         x1.legend(loc='best')
#         x2.legend(loc='best')
#         x3.legend(loc='best')
#         x4.legend(loc='best')

        span_r = [3.0375, 6.1125, 9.1875, 12.2625, 15.3375, 18.4125, 21.4875, 24.5625, 27.6375, 30.7125, 33.7875, 36.8625, 39.9375, 43.0125, 46.0875, 49.1625, 52.2375, 55.3125, 58.3875, 61.4625]
        c_ref = [3.5615, 3.9127, 4.2766, 4.5753, 4.6484, 4.5489, 4.3819, 4.2206, 4.0382, 3.8449, 3.6549, 3.4713, 3.2868, 3.1022, 2.9178, 2.7332, 2.5487, 2.3691, 2.1346, 1.4683]
        t_ref = [13.235, 13.235, 13.235, 13.1066, 11.6516, 10.5523, 9.6506, 8.7896, 7.876, 6.937, 6.0226, 5.1396, 4.2562, 3.428, 2.735, 2.1466, 1.5521, 0.9525, 0.3813, 0.0477]

        c1 = [3.7159, 4.1025, 4.4892, 4.8759, 4.9789, 4.935, 4.7145, 4.494, 4.2735, 4.053, 3.8325, 3.612, 3.3915, 3.171, 2.9505, 2.73, 2.5095, 2.289, 2.0685, 1.848]
        t1 = [12.2408, 12.2408, 12.2408, 12.2408, 10.7078, 9.1749, 7.6419, 6.109, 5.4567, 5.0348, 4.6128, 4.1909, 3.769, 3.3471, 2.8318, 2.2654, 1.6991, 1.1327, 0.5664, 0.0]

        c2 = [2.6272, 3.5659, 4.5046, 5.4432, 5.7752, 5.8324, 5.5811, 5.3299, 5.0787, 4.8274, 4.5762, 4.3249, 4.0737, 3.8225, 3.5712, 3.32, 3.0687, 2.8175, 2.5663, 2.315]
        t2 = [14.8995, 14.8995, 14.8995, 14.8995, 13.1144, 11.3292, 9.5441, 7.759, 6.8809, 6.2401, 5.5992, 4.9584, 4.3175, 3.6766, 3.0551, 2.4441, 1.833, 1.222, 0.611, -0.0]

        c3 = [2.739, 3.7908, 4.8426, 5.8944, 6.2567, 6.3015, 5.9865, 5.6714, 5.3564, 5.0414, 4.7264, 4.4114, 4.0964, 3.7814, 3.4663, 3.1513, 2.8363, 2.5213, 2.2063, 1.8913]
        t3 = [13.6792, 13.6792, 13.6792, 13.6792, 12.3531, 11.0271, 9.701, 8.3749, 7.5003, 6.7438, 5.9872, 5.2307, 4.4741, 3.7176, 3.0551, 2.4441, 1.833, 1.222, 0.611, 0.0]

        c4 = [2.7357, 3.7842, 4.8327, 5.8812, 6.2417, 6.2851, 5.9689, 5.6527, 5.3365, 5.0202, 4.704, 4.3878, 4.0716, 3.7554, 3.4392, 3.123, 2.8068, 2.4906, 2.1744, 1.8581]
        t4 = [14.0073, 14.0073, 14.0073, 14.0073, 12.4422, 10.8771, 9.312, 7.7469, 6.9101, 6.2638, 5.6175, 4.9712, 4.3249, 3.6786, 3.0551, 2.4441, 1.833, 1.222, 0.611, 0.0]

        #c1 = [3.4, 3.4, 3.4, 4.4507, 4.5599, 4.572, 4.5012, 4.3616, 4.167, 3.931, 3.6667, 3.3866, 3.1027, 2.8263, 2.5679, 2.3373, 2.1436, 1.9951, 1.8993, 1.8631]
        #c2 = [3.4, 3.4, 3.4, 4.678, 4.9328, 5.1058, 5.1998, 5.2179, 5.1632, 5.039, 4.8488, 4.5961, 4.2846, 3.9177, 3.4991, 3.0323, 2.5206, 1.9676, 1.3763, 0.75]
        #c3 = [3.4, 3.4, 3.4, 4.6798, 4.9371, 5.1141, 5.2142, 5.2405, 5.1967, 5.0863, 4.9132, 4.681, 4.3936, 4.0547, 3.6682, 3.2376, 2.7666, 2.2586, 1.7168, 1.1446]
        #c4 = [3.4, 3.4, 3.4, 4.1259, 4.2553, 4.3328, 4.3611, 4.3433, 4.2822, 4.1809, 4.0426, 3.8702, 3.6669, 3.4355, 3.1789, 2.9, 2.6014, 2.2856, 1.9551, 1.6122]


        #span_chord = [3.5615, 3.9127, 4.2766, 4.5753, 4.6484, 4.5489, 4.3819, 4.2206, 4.0382, 3.8449, 3.6549, 3.4713, 3.2868, 3.1022, 2.9178, 2.7332, 2.5487, 2.3691, 2.1346, 1.4683]
        #span_twist = [13.235, 13.235, 13.235, 13.1066, 11.6516, 10.5523, 9.6506, 8.7896, 7.876, 6.937, 6.0226, 5.1396, 4.2562, 3.428, 2.735, 2.1466, 1.5521, 0.9525, 0.3813, 0.0477]
        r2 = [4.0853, 8.2211, 12.3569, 16.4926, 20.6284, 24.7642, 28.8999, 33.0357, 37.1715, 41.3073, 45.443, 49.5788, 53.7146, 57.8503, 61.9861, 66.1219, 70.2576, 74.3934, 78.5292, 82.6649]
        c2 = [4.7901, 5.2624, 5.7519, 6.1536, 6.252, 6.1182, 5.8935, 5.6766, 5.4313, 5.1713, 4.9157, 4.6687, 4.4206, 4.1724, 3.9243, 3.6761, 3.428, 3.1864, 2.8709, 1.9748]
        t2 = [13.235, 13.235, 13.235, 13.1066, 11.6516, 10.5523, 9.6506, 8.7896, 7.876, 6.937, 6.0226, 5.1396, 4.2562, 3.428, 2.735, 2.1466, 1.5521, 0.9525, 0.3813, 0.0477]

        r3 = [4.0857, 8.2219, 12.3581, 16.4942, 20.6304, 24.7666, 28.9027, 33.0389, 37.1751, 41.3112, 45.4474, 49.5836, 53.7197, 57.8559, 61.9921, 66.1282, 70.2644, 74.4006, 78.5367, 82.6729]
        c3 = [4.7906, 5.2629, 5.7525, 6.1542, 6.2526, 6.1188, 5.8941, 5.6771, 5.4318, 5.1718, 4.9162, 4.6692, 4.4211, 4.1728, 3.9247, 3.6764, 3.4283, 3.1867, 2.8712, 1.9749]
        t3 = [13.235, 13.235, 13.235, 13.1066, 11.6516, 10.5523, 9.6506, 8.7896, 7.876, 6.937, 6.0226, 5.1396, 4.2562, 3.428, 2.735, 2.1466, 1.5521, 0.9525, 0.3813, 0.0477]


        fig = plt.figure()
         
        x1 = fig.add_subplot(121)
        x1.set_title('Chord distribution')
        x1.set_xlabel('Radial span [m]')
        x1.set_ylabel('Chord length [m]')
         
        x2 = fig.add_subplot(122)
        x2.set_title('Twist distribution')
        x2.set_xlabel('Radial span [m]')
        x2.set_ylabel('Twist angle [deg]')

        
        
#         x1.plot(span_r, c_ref,  linestyle='-',  linewidth=2, marker='^',  label='Reference')
#         x1.plot(span_r, c1,     linestyle='-',  linewidth=2, marker='s', label='Max Cp')
#         x1.plot(span_r, c2,     linestyle='-',  linewidth=2, marker='o', label='Max Cp/Blade mass')
#         x1.plot(span_r, c3,     linestyle='-',  linewidth=2, marker='+', label='Max Cp/RNA mass')
#         x1.plot(span_r, c4,     linestyle='-',  linewidth=2, marker='*', label='Min LCOE')        
#         x1.legend(loc='best')
#         
#         x2.plot(span_r, t_ref,  linestyle='-',  linewidth=2, marker='^',  label='Reference')
#         x2.plot(span_r, t1,     linestyle='-',  linewidth=2, marker='s', label='Max Cp')
#         x2.plot(span_r, t2,     linestyle='-',  linewidth=2, marker='o', label='Max Cp/Blade mass')
#         x2.plot(span_r, t3,     linestyle='-',  linewidth=2, marker='+', label='Max Cp/RNA mass')
#         x2.plot(span_r, t4,     linestyle='-',  linewidth=2, marker='*', label='Min LCOE')        
#         x2.legend(loc='best')        
        
        
        x1.plot(span_r, c_ref,  linestyle='-',  linewidth=2, marker='^',  label='Reference')
        x1.plot(r2, c2,     linestyle='-',  linewidth=2, marker='s', label='Rotor')
        x1.plot(r3, c3,     linestyle='-',  linewidth=2, marker='o', label='Rotor+Layout')
        #x1.plot(span_r, c3,     linestyle='-',  linewidth=2, marker='+', label='Max Cp/RNA mass')
        #x1.plot(span_r, c4,     linestyle='-',  linewidth=2, marker='*', label='Min LCOE')        
        x1.legend(loc='best')
        
        x2.plot(span_r, t_ref,  linestyle='-',  linewidth=2, marker='^',  label='Reference')
        x2.plot(r2, t2,     linestyle='-',  linewidth=2, marker='s', label='Rotor')
        x2.plot(r3, t3,     linestyle='-',  linewidth=2, marker='o', label='Rotor+Layout')
        #x2.plot(span_r, t3,     linestyle='-',  linewidth=2, marker='+', label='Max Cp/RNA mass')
        #x2.plot(span_r, t4,     linestyle='-',  linewidth=2, marker='*', label='Min LCOE')        
        x2.legend(loc='best')   
        
        plt.show()
        
   
    
if __name__ == "__main__":
    plot_pc() 