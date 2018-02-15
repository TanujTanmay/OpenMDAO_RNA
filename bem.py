from fixed_parameters import rho_air
from math import pi, atan, degrees, radians, acos, sqrt, cos, sin, exp
import numpy as np
import pandas as pd

def bem(wind_speed, n_blades, rotor_radius, hub_radius, tsr, pitch, \
        r, dr, chord, twist, airfoil, \
        is_prandtl, is_glauert):
    
    n_itr = 100
    itr_tol = 0.0001
    itr_relax = 0.75
    
    CT1 = 1.816
    CT2 = 2*sqrt(CT1)-CT1
    
    mu = r/rotor_radius
    mu_root = hub_radius/rotor_radius
    area = 2 * pi * r * dr
    tsr_r = tsr * mu
    
    # initial estimates of induction factors
    aA = 0.3
    aT = 0.0
    
    for i in range(n_itr):
        phi = degrees(atan((1-aA)/(tsr_r * (1+aT))))
        alpha = phi - twist - pitch
        
        cl = np.interp(alpha, airfoil.ix[:, 'Alpha'], airfoil.ix[:, 'Cl'])
        cd = np.interp(alpha, airfoil.ix[:, 'Alpha'], airfoil.ix[:, 'Cd'])
        
        if np.isnan(cl) or cl < 1.0e-6 :
            cl = 1.0e-6
            cd = 0
            
        w = wind_speed * sqrt( (1-aA)**2 + (tsr_r*(1+aT))**2 ) 
        lift = 0.5 * chord * rho_air * (w**2) * cl
        drag = 0.5 * chord * rho_air * (w**2) * cd
        
        fx = (lift*cos(radians(phi))) + (drag*sin(radians(phi)))
        fy = (lift*sin(radians(phi))) - (drag*cos(radians(phi)))
        cx = fx/(0.5 * rho_air * (wind_speed**2) * rotor_radius)
        cy = fy/(0.5 * rho_air * (wind_speed**2) * rotor_radius)
        
        ct = (fx * n_blades * dr)/(0.5 * rho_air * (wind_speed**2) * area)
        cq = (fy * n_blades * dr * mu)/(0.5 * rho_air * (wind_speed**2) * area)
        cp = cq * tsr
        
        # Prandtl correction for tip and root losses        
        if(is_prandtl):
            f_tip   = (2.0/pi) * acos(exp(-(n_blades/2.0)*((1-mu)/mu)*sqrt(1+(tsr_r/(1-aA))**2)))
            f_root  = (2.0/pi) * acos(exp(-(n_blades/2.0)*((mu-mu_root)/mu)*sqrt(1+(tsr_r/(1-aA))**2)))
        else:
            f_tip   = 1
            f_root  = 1
            
        f = f_tip*f_root
        f = 0.0001 if f<0.0001 else f
           
        # Glauert correction for heavily loaded rotor
        if(is_glauert):
            aA_new =  0.5 - 0.5*sqrt(1 - ct) if (ct < CT2) else 1 + 0.25*(ct - CT1)/(sqrt(CT1) - 1)
        else:
            aA_new = 0.5 - 0.5*sqrt(1 - ct) if(ct <= 0.96) else 0.4
        
        aA_new = aA_new/f
        
        # Bound the value of axial induction
        if (aA_new > 0.96):
            aA_new = 0.96
            ct = CT1 - 4*(sqrt(CT1) - 1)*(1-aA_new)
        
        aA = itr_relax*aA + (1-itr_relax)*aA_new
        
        aT = (fy * n_blades)/(2 * 2*pi*r * rho_air * (wind_speed**2) * (1-aA) * tsr_r)
        aT = aT/f
        
        # Bound the value of tangential induction
        if (aT > aA*(1-aA)/(tsr_r**2)):
            aT = aA*(1-aA)/(tsr_r**2)    
        
        # Check convergence
        if(abs((aA_new - aA)) <= itr_tol):
            break      
        
    result = [phi, alpha, aA, aT, f, cl, cd, lift, drag, fx, fy, cx, cy, ct, cq, cp]
    print i, r, mu, dr, area, tsr_r, chord, twist, f_tip, f_root, f, rotor_root
    print aA, aT
    return result    
         
    
if __name__ == "__main__":
    airfoil = pd.read_csv("Airfoils/ah.csv", skiprows=9)
    #print airfoil.keys
    print bem(10.0, 50.0, 0.2, 3, 0.0, 8.5, airfoil, 20.0, 1.0, 3.5, 2.0, 1, 1)