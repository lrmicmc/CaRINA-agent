# -*- coding: utf-8 -*-
#import numpy as np
# from math import *
#from numpy import *

import numpy as np
import math
import copy
import os
# import matplotlib.pyplot as plt

path = os.path.dirname(os.path.realpath(__file__)) + '/'

with open(path + "Clothoid_LT.txt","r") as text_file:
    lines = text_file.read().split('\t')

text_file.close()
len_lines = len(lines)
end_list_float = len_lines/2
end_list = int(end_list_float)
x_L = [0]*end_list
y_L = [0]*end_list
C_min = 5.944748225238571e-04
C_max = 10.237604306862353e+03
K_max = 1.196835466593837e+02
C_L = 1.
sL = 1.223430214110182e+02
Delta_sL = 0.002660093525200
c_L = 1.
x_L = (lines[0::2])
y_L = (lines[1::2])


##############################################################################
# GetBasicClothoidCoords( s )
##############################################################################
def GetBasicClothoidCoords( s ):
# GETBASICCLOTHOIDCOORDS Calculates clothoid basic coordinates from a lookup
# table with previous computed coordinates. These coordinates are used later
# to compute general clothoid coordinates.
#
# Filename: clothoid_basic.py
#
# Description: Given the travelled distance along the basic clothoid, this
#       function returns the basic clothoid coordinates. The pseudocode was
#       implemented by Misel Brezak and Ivan Petrovic and it was described
#       in the paper Real-time Approximation of Clothoids With Bounded Error
#         for Path Planning Applications, IEEE Transactions on Robotics, 2014.
#        IEEE, v. 30, n. 2, p. 507–515, 2014.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 28/06/17
#
# Release: 1.0
#
# Input (s):
# [0] s        - travelled distance along the basic clothoid in meters

# Output (s):
# [0] x_Lb     - abscissa coordinate in meters.
# [1] y_Lb     - ordinate coordinate in meters.
#
# Copyright owned by Universidade de São Paulo

    k = np.floor(abs(s)/Delta_sL)
    k = int(k)
    sk = (k)*Delta_sL
    # rk = math.pow(c_L*(sk + 0.5*Delta_sL),-1)
    rk = 1./(c_L*(sk + 0.5*Delta_sL))
    theta_Lk = 0.5*c_L*sk**2
    x_Lk = float(x_L[k])
    y_Lk = float(y_L[k])
    ds = abs(s) - sk
    x_Lb = x_Lk + 2*rk*np.cos(theta_Lk + ds/(2*rk))*np.sin(ds/(2*rk))
    y_Lb = y_Lk + 2*rk*np.sin(theta_Lk + ds/(2*rk))*np.sin(ds/(2*rk))
    if s < 0:
        x_Lb = -x_Lb
        y_Lb = -y_Lb
    return [x_Lb, y_Lb]

##############################################################################
# GetGeneralClothoidCoords( x0, y0, theta0, kappa0, c, s ):
##############################################################################
def GetGeneralClothoidCoords( x0, y0, theta0, kappa0, c, s ):
# GETGENERALCLOTHOIDCOORDS Calculates clothoid general coordinates.
#
# Filename: clothoid_basic.py
#
# Description: Given the clothoid initial parameters as well as the curvature
#        derivate and travelled distance along the clothoid, this function
#       computes abscissa and ordinate coordinates. The pseudocode was
#       implemented by Miˇsel Brezak and Ivan Petrovi´c and it was described
#       in the paper Real-time Approximation of Clothoids With Bounded Error
#         for Path Planning Applications, IEEE Transactions on Robotics, 2014.
#        IEEE, v. 30, n. 2, p. 507–515, 2014.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 28/06/17
#
# Release: 1.0
#
# Input (s):
# [0] x0       - initial abscissa coordinate (m)
# [0] y0       - initial ordinate coordinate (m)
# [0] theta0   - initial tangent angle (rad)
# [0] kappa0   - initial curvature (1/m)
# [0] c        - curvature derivative (constante by definition) (1/m^2)
# [0] s        - travelled distance along the clothoid (m)

# Output (s):
# [0] x        - abscissa coordinate (m).
# [1] y        - ordinate coordinate (m).
#
# Copyright owned by Universidade de São Paulo

    theta0 = theta0 + 3*math.pi/2 + np.sign(c)*math.pi/2
    C = 1./(np.sqrt(abs(c)))
    K = kappa0*C
    s1 = C_L*(s/C + K)
    s2 = C_L*K
    if (C < C_min) and (abs(s1) > sL or abs(s2) > sL):
        x = x0
        y = y0
    elif  kappa0 == 0 and C > C_max:
        x = x0 + np.cos(theta0)*s
        y = y0 + np.sin(theta0)*s
    elif abs(K) > K_max:
        x = x0 + (1./kappa0)*(-np.sin(theta0) + np.sin(theta0 + kappa0*s))
        y = y0 + (1./kappa0)*(np.cos(theta0) - np.cos(theta0 + kappa0*s))
    else:
        # theta_rot = (-math.pow(kappa0,2)/2)/c + theta0
        theta_rot = (-kappa0**2/2)/c + theta0
        r11 = np.cos(theta_rot)
        r22 = r11
        r12 = -np.sin(theta_rot)
        r21 = -r12
        [x1,y1] = GetBasicClothoidCoords(s1)
        [x2,y2] = GetBasicClothoidCoords(s2)
        xz = (x1 - x2)*C/C_L
        yz = (y1 - y2)*(C/C_L)*np.sign(c)
        x = x0 + r11*xz + r12*yz
        y = y0 + r21*xz + r22*yz
    return [x, y]

##############################################################################
# cosC( delta )
##############################################################################
def cosC( delta ):
# COSC Calculates the "clothoid cossine" of delta
#
# Filename: clothoid_basic.py
#
# Description: Given the angle delta, this function computes the
#         "clothoid cossine" of delta. The pseudocode was implemented by Doran
#         K. Wilde and it was described in the paper Computing clothoid segments
#         for trajectory generation. In: IEEE. 2009 IEEE/RSJ International
#         Conference on Intelligent Robots and Systems. 20010. p. 2440–2445.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 28/06/17
#
# Release: 1.0
#
# Input (s):
# [0] delta         - input angle (rad)

# Output (s):
# [0] cosC_delta    - "clothoid cossine" (rad).
#
# Copyright owned by Universidade de São Paulo

    if delta > 0:
        t1 = 0
        t2 = math.sqrt(2*delta/math.pi)
        [C,S] = GetGeneralClothoidCoords(0,0,0,0,math.pi,(t2-t1))
        cosC_delta = (np.cos(delta)*C+np.sin(delta)*S)/t2
    elif delta == 0:
        cosC_delta = 1
    else:
        t1 = 0
        t2 = math.sqrt(2*delta/math.pi)
        [C,S] = GetGeneralClothoidCoords(0,0,0,0,-math.pi,(t2-t1))
        cosC_delta = (np.cos(-delta)*C+np.sin(-delta)*S)/t2
    return cosC_delta

##############################################################################
# sinC( delta )
##############################################################################
def sinC( delta ):
# SINC Calculates the "clothoid sine" of delta
#
# Filename: clothoid_basic.py
#
# Description: Given the angle delta, this function computes the
#         "clothoid sine" of delta. The pseudocode was implemented by Doran
#         K. Wilde and it was described in the paper Computing clothoid segments
#         for trajectory generation. In: IEEE. 2009 IEEE/RSJ International
#         Conference on Intelligent Robots and Systems. 20010. p. 2440–2445.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 28/06/17
#
# Release: 1.0
#
# Input (s):
# [0] delta         - input angle (rad)

# Output (s):
# [0] sinC_delta    - "clothoid sine" (rad).
#
# Copyright owned by Universidade de São Paulo

    if delta > 0:
        t1 = 0
        t2 = math.sqrt(2*delta/math.pi)
        [C,S] = GetGeneralClothoidCoords(0,0,0,0,math.pi,(t2-t1))
        sinC_delta = (-np.sin(delta)*C+np.cos(delta)*S)/t2
    elif delta == 0:
        sinC_delta = 0
    else:
        t1 = 0
        t2 = math.sqrt(2*delta/math.pi)
        [C,S] = GetGeneralClothoidCoords(0,0,0,0,-math.pi,(t2-t1))
        sinC_delta = (np.sin(-delta)*C-np.cos(-delta)*S)/t2
    return sinC_delta

##############################################################################
# max_values_1( delta, x )
##############################################################################
def max_values_1( delta, x ):
# MAX_VALUES_1 Calculates the maximum curvature, curvature derivative and arc
#       length required in a turn using a single clothoid.
#
# Filename: clothoid_basic.py
#
# Description: Given the defleciotn angle delta and the longiturnal travelled
#        distance, this function computes the length, maximum curvature and
#        curvature derivative of the clothoid. These parameters are used later
#        to compute the clothoid path. The pseudocode was implemented by Doran
#         K. Wilde and it was described in the paper Computing clothoid segments
#         for trajectory generation. In: IEEE. 2009 IEEE/RSJ International
#         Conference on Intelligent Robots and Systems. 20010. p. 2440–2445.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 28/06/17
#
# Release: 1.0
#
# Input (s):
# [0] delta         - input angle (rad)
# [1] x             - longitudinal travelled distance (m)

# Output (s):
# [0] L             - clothoid arc length.
# [1] kappa_max     - maximum curvature (1/m).
# [2] sigma_max     - curvature derivative (constante by definition) (1/m^2)
#
# Copyright owned by Universidade de São Paulo

    if delta<0:
        delta = -delta

    t1 = 0
    t2 = math.sqrt(2*delta/math.pi)

    [C,S] = GetGeneralClothoidCoords(0,0,0,0,math.pi,(t2-t1))

    cosC_delta = cosC(delta)

    L = x/cosC_delta
    kappa_max = 2*delta/L
    sigma_max = kappa_max/L
    return [L, kappa_max, sigma_max]

##############################################################################
# max_values_2( delta, kappa_max, x )
##############################################################################
def max_values_2( delta, kappa_max, x ):
# MAX_VALUES_2 Calculates the clothoid length, circular arc length and curvature
#       derivative required in a turn with bounded curvature.
#
# Filename: clothoid_basic.py
#
# Description: Given the deflection angle delta, the longiturnal travelled
#        distance and the maximum curvature allowed, this function computes
#        the length, maximum curvature and curvature derivative of the clothoid.
#        These parameters are used later to compute the clothoid path. The
#        pseudocode was implemented by Doran K. Wilde and it was described in
#         the paper Computing clothoid segments for trajectory generation. In:
#        IEEE. 2009 IEEE/RSJ International Conference on Intelligent Robots and
#         Systems. 20010. p. 2440–2445.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 29/06/17
#
# Release: 1.0
#
# Input (s):
# [0] delta         - input angle (rad)
# [1] kappa_max     - Maximum curvature allowed (1/m)
# [1] x             - longitudinal travelled distance (m)

# Output (s):
# [0] Lc            - clothoid arc length (m).
# [1] Larc          - circular arc length (m).
# [2] sigma_max     - curvature derivative (constante by definition) (1/m^2)
#
# Copyright owned by Universidade de São Paulo

    if delta<0:
        delta = -delta
    lambdA = delta/2
    lambdA_ant = 10
    while abs(lambdA - lambdA_ant) > 1e-6: #Convergence criteria: 1e-6 rad
        lambdA_ant = lambdA
        lambdA = lambdA-((2*lambdA-2*delta)+((x*kappa_max-np.sin(lambdA))/(cosC(delta-lambdA)*np.cos(lambdA)+sinC(delta-lambdA)*np.sin(lambdA))))
    deltac = delta - lambdA
    Lc = 2*(deltac)/kappa_max
    Larc = lambdA/kappa_max
    sigma_max = kappa_max/Lc
    return [Lc, Larc, sigma_max]

##############################################################################
# norm_ang( theta )
##############################################################################
def norm_ang( theta ):
# NORM_ANG Calculates the normalized angle (-pi to pi) of theta
#
# Filename: clothoid_basic.py
#
# Description: This function computes the normalized angle (-pi to pi) of a
#         given angle theta. For example, the normalizad angle of 3*pi/2 is
#         -pi/2.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 29/06/17
#
# Release: 1.0
#
# Input (s):
# [0] theta     - input angle (rad)

# Output (s):
# [0] theta_n   - normalized angle (rad).
#
# Copyright owned by Universidade de São Paulo

    if theta > math.pi:
        theta_n = theta - 2*math.pi
    elif theta < -math.pi:
        theta_n = 2*math.pi + theta
    else:
        theta_n = theta
    return theta_n

##############################################################################
# final_points( inital_parameters, sharp_length )
##############################################################################
def final_points( inital_parameters, sharp_length ):
# FINAL_POINTS Calculates the final configuration (x, y, theta and
#         kappa) of a path constituded of clothoids, circular arc and straight
#         lines. Also returns the initial configuration.
#
# Filename: clothoid_basic.py
#
# Description: The final configuration is computed from the initial configuration
#       x0, y0, theta0 and kappa0 and from all curvature derivative and length of
#       all curve segments present in the path. As a piecewise linear path, the
#       curvature derivative is constant along all segments.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 29/06/17
#
# Release: 1.0
#
# Input (s):
# [0] inital_parameters - unidimensional numpy array: [x0, y0, theta0, kappa0]
# [1] sharp_length      - numpy array nx2, where n is the number of segments.
#       The first column has all curvature derivative (sharpness) and the
#       second one has all segments length.
#
# Output (s):
# [0] pin               - unidimensional numpy array: [x0, y0, theta0, kappa0]
# [1] pf                - unidimensional numpy array: [xf, yf, thetaf, kappaf]
#        with the final configuration.
#
# Copyright owned by Universidade de São Paulo

    # Load initial states (parameters)
    x0 = inital_parameters[0]
    y0 = inital_parameters[1]
    theta0 = inital_parameters[2]
    kappa0 = inital_parameters[3]

    pin = np.array([x0, y0, theta0, kappa0]) #Save initial paramters to return
    n = len(sharp_length)
    for k in range(0, n):
        sigma = sharp_length[k, 0]
        L = sharp_length[k, 1]
        if abs(sigma) >= 1e-10: # Compute clothoid
            if sigma < 0:
                L = -L
            [x0,y0] = GetGeneralClothoidCoords(x0,y0,theta0,kappa0,sigma,L);
        elif abs(kappa0) > 1e-6: # Compute circular arc
            alfa = kappa0*L
            if alfa > 0:
                x0 = (1/kappa0)*np.cos(theta0 + math.pi/2) + np.cos(theta0+alfa-math.pi/2)/abs(kappa0) + x0
                y0 = (1/kappa0)*np.sin(theta0 + math.pi/2) + np.sin(theta0+alfa-math.pi/2)/abs(kappa0) + y0
            else:
                x0 = (1/kappa0)*np.cos(theta0 + math.pi/2) + np.cos(theta0+alfa+math.pi/2)/abs(kappa0) + x0
                y0 = (1/kappa0)*np.sin(theta0 + math.pi/2) + np.sin(theta0+alfa+math.pi/2)/abs(kappa0) + y0
        else: # Compute straight line
            x0 = x0 + L*np.cos(theta0)
            y0 = y0 + L*np.sin(theta0)
        L = abs(L)
        theta0 = theta0 + kappa0*L +(sigma*L*L)/2
        kappa0 = kappa0 + sigma*L
    pf = np.array([x0, y0, theta0, kappa0])
    return [pin, pf]

##############################################################################
# CC_Trig( p1, T1_ang, p2, T2_ang, kappa_max )
##############################################################################
def CC_Trig( p1, T1_ang, p2, T2_ang, kappa_max ):
# CC_TRIG Calculates the paremeters required to perfom a turn with G2 continuity.
#
# Filename: clothoid_basic.py
#
# Description: Given the start and goal configurations, this function computes
#         the curvature derivatives and lengths of all segments required to
#         perform the configuration change. Also, the maneuver is made with
#         bounded curvature passed as input.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 29/06/17
#
# Release: 1.0
#
# Input (s):
# [0] p1             - unidimensional numpy array [x0, y0]: start point
# [1] T1_ang         - start tangent angle (rad)
# [2] p2             - unidimensional numpy array: [xf, yf]: goal point
# (4) T2_ang         - goal tangent angle (rad)
# (5) kappa_max      - maximum curvature allowed (1/m)
#
# Output (s):
# [0] initParam      - unidimensional numpy array: [x0, y0, theta0, kappa0]
# [1] sharp_length   - numpy array nx2, where n is the number of segments.
#       The first column has all curvature derivative (sharpness) and the
#       second one has all segments length.
#
# Copyright owned by Universidade de São Paulo

    D = p2 - p1
    mod_D = math.sqrt(D[0]*D[0]+D[1]*D[1])
    d = mod_D
    D = D/mod_D #Faz T1 ficar unitário
    D_ang = norm_ang(math.atan2(D[1],D[0]))

    alfa1 = norm_ang(D_ang - T1_ang)
    alfa2 = norm_ang(T2_ang - D_ang)
    ###########################################################################
    # Trecho novo!!! Deve ser retirada caso se deseje o algoritmo original
    alfa = abs(norm_ang(T1_ang-T2_ang)/2)
    if abs(alfa1)>=abs(alfa2):
        z = np.sin(abs(alfa1)-alfa)*d;
        y = z/(1e-8+np.tan(alfa))
        if z!=0:
            x = (z-y*np.tan(abs(alfa1)-alfa))/np.tan(abs(alfa1)-alfa)
        else:
            x = d
    else:
        z = np.sin(abs(alfa2)-alfa)*d
        y = z/np.tan(alfa)
        if z!=0:
            x = (z-y*np.tan(abs(alfa2)-alfa))/np.tan(abs(alfa2)-alfa)
        else:
            x = d
    ###########################################################################
    delta = alfa1+alfa2
    flag = False
    x = x/2
    delta = delta/2
    if delta<0:
        delta = -delta
        flag = True

    [L,kappa,sigma] = max_values_1(delta,x)
    if kappa > kappa_max:
        [a,a,sigma] = max_values_2(delta,kappa_max,x)
        delta_min = kappa_max*kappa_max/sigma
        Lc = kappa_max/sigma
        if flag:
            sigma = -sigma
        sh_len = np.array([[sigma, Lc],[0, (2*delta-delta_min)/kappa_max],[-sigma, Lc]])
    else:
        if flag:
            sigma = -sigma
        sh_len = np.array([[sigma, L],[-sigma, L]])

    x0 = p1[0]
    y0 = p1[1]
    theta0 = T1_ang
    kappa0 = 0

    #Compute the lenghts of the straight lines
    [_, pfin] = final_points( np.array([x0, y0, theta0, kappa0]), sh_len )
    xf = pfin[0]
    yf = pfin[1]

    #Calculo da translação
    # if abs(norm_ang(T1_ang)) != math.pi/2:
    #     xp = (p2[1]-yf+np.tan(T1_ang)*xf-np.tan(T2_ang)*p2[0])/(np.tan(T1_ang)-np.tan(T2_ang))
    #     yp = np.tan(T1_ang)*(xp - xf)+yf
    # else:
    #     xp = xf
    #     yp = np.tan(T2_ang)*(xp - p2[0])+p2[1]

    if abs(norm_ang(T1_ang)) != math.pi/2:
        xp = (p2[1]-yf+np.tan(T1_ang)*xf-np.tan(T2_ang)*p2[0])/(np.tan(T1_ang)-np.tan(T2_ang)+1e-8)
    else:
        xp = xf
    if abs(norm_ang(T2_ang)) != math.pi/2:
        yp = np.tan(T2_ang)*(xp - p2[0])+p2[1]
    else:
        yp = yf

    r1 = math.sqrt(pow(xp-xf,2)+pow(yp-yf,2)) #length of first straight line
    r2 = math.sqrt(pow(xp-p2[0],2)+pow(yp-p2[1],2)) #length of second straight line

    initParam = np.array([x0, y0, theta0, kappa0])
    if r1 < 1e-6 and r2 > 1e-6:
        sl2 = np.zeros((1,2))
        sl2[0,0] = 0
        sl2[0,1] = r2
        sharp_length = np.concatenate((sh_len,sl2))
    elif r1 > 1e-6 and r2 < 1e-6:
        sl1 = np.zeros((1,2))
        sl1[0,0] = 0
        sl1[0,1] = r1
        sharp_length = np.concatenate((sl1,sh_len))
    elif r1 > 1e-6 and r2 > 1e-6:
        sl1 = np.zeros((1,2))
        sl1[0,0] = 0
        sl1[0,1] = r1
        sl2 = np.zeros((1,2))
        sl2[0,0] = 0
        sl2[0,1] = r2
        sharp_length = np.concatenate((sl1,sh_len,sl2))
    else:
        sharp_length = sh_len

    return [initParam, sharp_length]

##############################################################################
# CC_S( p1, T1_ang, p2, T2_ang, kappa_max )
##############################################################################
def CC_S( p1, T1_ang, p2, T2_ang, kappa_max ):
# CC_S Calculates the parameters required to perfom a S-shape maneuver with G2
#       continuity.
#
# Filename: clothoid_basic.py
#
# Description: Given the start and goal configurations, this function computes
#         the curvature derivatives and lengths of all segments required to
#         perform S-shape interpolation. Also, the maneuver is made with
#         bounded curvature passed as input.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 29/06/17
#
# Release: 1.0
#
# Input (s):
# [0] p1             - unidimensional numpy array [x0, y0]: start point
# [1] T1_ang         - start tangent angle (rad)
# [2] p2             - unidimensional numpy array: [xf, yf]: goal point
# (4) T2_ang         - goal tangent angle (rad)
# (5) kappa_max      - maximum curvature allowed (1/m)
#
# Output (s):
# [0] initParam      - unidimensional numpy array: [x0, y0, theta0, kappa0]
# [1] sharp_length   - numpy array nx2, where n is the number of segments.
#       The first column has all curvature derivative (sharpness) and the
#       second one has all segments length.
#
# Copyright owned by Universidade de São Paulo

    D = p2 - p1
    mod_D = math.sqrt(D[0]*D[0]+D[1]*D[1])
    d = mod_D
    D = D/mod_D #Faz T1 ficar unitário
    D_ang = norm_ang(math.atan2(D[1],D[0]))

    alfa1 = norm_ang(D_ang - T1_ang)
    alfa2 = norm_ang(T2_ang - D_ang)
    # print "\033[91m {}, {}, {}, {}, {}, {} \033[0m".format(alfa1, D_ang, T1_ang, mod_D, p2, p1) 
    if np.sign(alfa1) == np.sign(alfa2) and abs(alfa1) > 0 and abs(alfa2) > 0:
        [it1,sharp_length] = CC_Trig( p1, T1_ang, p2, T2_ang, kappa_max )
    elif alfa1 > 1e-6:
        fi1 = (3*abs(alfa1)+abs(alfa2))/4
        fi2 = -(abs(alfa1)-abs(alfa2)-2*abs(fi1))/2
        l3 = d/(np.cos(abs(fi1)-abs(alfa1))+np.cos(abs(alfa2)-abs(fi2)))
        npoint = p1+l3*np.array([np.cos(T1_ang+abs(fi1)), np.sin(T1_ang+abs(fi1))])
        [it1,sl1] = CC_Trig( p1, T1_ang, npoint, T1_ang+2*abs(fi1), kappa_max )
        [_, sl2] = CC_Trig( npoint, T1_ang+2*abs(fi1), p2, T2_ang, kappa_max )
        sharp_length = np.concatenate((sl1, sl2))
    elif alfa1 < 1e-6:
        fi1 = (3*abs(alfa1)+abs(alfa2))/4;
        fi2 = -(abs(alfa1)-abs(alfa2)-2*abs(fi1))/2
        l3 = d/(np.cos(abs(fi1)-abs(alfa1))+np.cos(abs(alfa2)-abs(fi2)))
        npoint = p1+l3*np.array([np.cos(T1_ang-abs(fi1)), np.sin(T1_ang-abs(fi1))])
        [it1,sl1] = CC_Trig( p1, T1_ang, npoint, T1_ang-2*abs(fi1), kappa_max )
        [_, sl2] = CC_Trig( npoint, T1_ang-2*abs(fi1), p2, T2_ang, kappa_max )
        sharp_length = np.concatenate((sl1, sl2))
    else:
        it1 = np.array([p1[0], p1[1], T1_ang, 0])
        sharp_length = np.zeros((1,2))
        sharp_length[0,0] = 0
        sharp_length[0,1] = d
    return [it1,sharp_length]

# p1 = array([0, 0])
# p2 = array([5, 30])
# T1_ang = 0
# T2_ang = 0.1
# kappa_max = .2
# [it1, sharp_lengt] = CC_S( p1, T1_ang, p2, T2_ang, kappa_max )

##############################################################################
# CC_Turn_dif_sigma( p1, T1_ang, p2, T2_ang, kappa_max, sigma_max1, sigma_max2)
##############################################################################
def CC_Turn_dif_sigma( p1, T1_ang, p2, T2_ang, kappa_max, sigma_max1, sigma_max2):
# CC_TURN_DIF_SIGMA Calculates the paremeters required to perfom a turn with G2
#       with nonsymmetrical clothoids clothoids (different curvature derivatives).
#
# Filename: clothoid_basic.py
#
# Description: Given the start and goal configurations, this function computes
#         the lengths of all segments required to perform the configuration
#         change. Also, the maneuver is made with bounded curvature passed as
#         input.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 29/06/17
#
# Release: 1.0
#
# Input (s):
# [0] p1             - unidimensional numpy array [x0, y0]: start point
# [1] T1_ang         - start tangent angle (rad)
# [2] p2             - unidimensional numpy array: [xf, yf]: goal point
# (4) T2_ang         - goal tangent angle (rad)
# (5) kappa_max      - maximum curvature allowed (1/m)
# (6) sigma_max1     - curvature derivative of the first clothoid (1/m^2)
# (7) sigma_max2     - curvature derivative of the second clothoid (1/m^2)
#
# Output (s):
# [0] initParam      - unidimensional numpy array: [x0, y0, theta0, kappa0]
# [1] sharp_length   - numpy array nx2, where n is the number of segments.
#       The first column has all curvature derivative (sharpness) and the
#       second one has all segments length.
#
# Copyright owned by Universidade de São Paulo

    D = p2 - p1
    mod_D = math.sqrt(D[0]*D[0]+D[1]*D[1])
    D = D/mod_D;#Faz T1 ficar unitário
    D_ang = norm_ang(np.arctan2(D[1],D[0]));

    alfa1 = norm_ang(D_ang - T1_ang)
    alfa2 = norm_ang(T2_ang - D_ang)

    delta = alfa1+alfa2
    flag = False
    if delta<0:
        delta = -delta
        flag = True
    s1 = kappa_max/sigma_max1
    s2 = kappa_max/sigma_max2
    delta_min = kappa_max*kappa_max/(2*sigma_max1)+kappa_max*kappa_max/(2*sigma_max2)
    s_C = np.zeros((1,3))
    s_C[0,0] = s1
    s_C[0,1] = (delta - delta_min)*1/kappa_max
    s_C[0,2] = s2
    c_C = np.zeros((1,3))
    c_C[0,0] = sigma_max1
    c_C[0,1] = 0
    c_C[0,2] = -sigma_max2
    if flag:
        c_C = -c_C
    sh_len = np.transpose(np.concatenate((c_C, s_C)))

    x0 = p1[0]
    y0 = p1[1]
    theta0 = T1_ang
    kappa0 = 0


    #Compute the lenghts of the straight lines
    [_, pfin] = final_points(np.array([x0, y0, theta0, kappa0]), sh_len)
    xf = pfin[0]
    yf = pfin[1]

    #Calculo da translação
    if abs(norm_ang(T1_ang)) != math.pi/2:
        xp = (p2[1]-yf+np.tan(T1_ang)*xf-np.tan(T2_ang)*p2[0])/(np.tan(T1_ang)-np.tan(T2_ang))
        yp = np.tan(T1_ang)*(xp - xf)+yf
    else:
        xp = xf
        yp = np.tan(T2_ang)*(xp - p2[0])+p2[1]

    r1 = np.sqrt(pow(xp-xf,2)+pow(yp-yf,2)) #length of first straight line
    r2 = np.sqrt(pow(xp-p2[0],2)+pow(yp-p2[1],2)) #length of second straight line

    initParam = np.array([x0, y0, theta0, kappa0])
    if r1 < 1e-6 and r2 > 1e-6:
        sl2 = np.zeros((1,2))
        sl2[0,0] = 0
        sl2[0,1] = r2
        sharp_length = np.concatenate((sh_len,sl2))
    elif r1 > 1e-6 and r2 < 1e-6:
        sl1 = np.zeros((1,2))
        sl1[0,0] = 0
        sl1[0,1] = r1
        sharp_length = np.concatenate((sl1,sh_len))
    elif r1 > 1e-6 and r2 > 1e-6:
        sl1 = np.zeros((1,2))
        sl1[0,0] = 0
        sl1[0,1] = r1
        sl2 = np.zeros((1,2))
        sl2[0,0] = 0
        sl2[0,1] = r2
        sharp_length = np.concatenate((sl1,sh_len,sl2))
    else:
        sharp_length = sh_len
    return [initParam, sharp_length]

##############################################################################
# GetClothoidPath( initPar, sharp_length, ds )
##############################################################################
def GetClothoidPath( initPar, sharp_length, ds ):
# GETCLOTHOIDPATH Calculates a path made of clothoids, circular arcs and straigh
#         straight lines with a given step.
#
# Filename: clothoid_basic.py
#
# Description: The path is computed from the initial configuration
#       x0, y0, theta0 and kappa0 and from all curvature derivative and length of
#       all curve segments present in the path. As a piecewise linear path, the
#       curvature derivative is constant along all segments. The step, i.e.,
#       the distance between each point is passed as input.
#
# Author:
#        Júnior Anderson Rodrigues da Silva
#        juniorars@gmail.com
#
# Date: 29/06/17
#
# Release: 1.0
#
# Input (s):
# [0] initParam      - unidimensional numpy array: [x0, y0, theta0, kappa0]
# [1] sharp_length   - numpy array nx2, where n is the number of segments.
#       The first column has all curvature derivative (sharpness) and the
#       second one has all segments length.

# Output (s):
# [0] [xc, yc, thetac, kappac, sigmac, sc]
#       - xc     - x coordinate points (m)
#       - yc     - y coordinate points (m)
#       - thetac - heading at the point (rad)
#       - kappac - curvature at the point (1/m)
#       - sigmac - curvature derivative at the point (1/m2)
#       - sc     - distance travelled along the path (arclength) at the point (m)

# Copyright owned by Universidade de São Paulo

    s_C = sharp_length[:,1]

    #Create empty arrays
    number_lines = round(sum(s_C)/ds)+3
    number_lines = int(number_lines)
    xc = np.zeros((number_lines,1))
    yc = np.zeros((number_lines,1))
    thetac = np.zeros((number_lines,1))
    thetac_teste = np.zeros((number_lines,1))
    kappac = np.zeros((number_lines,1))
    sigmac = np.zeros((number_lines,1))
    sc = np.zeros((number_lines,1))

    aux_sh_len = np.zeros((1,2))

    #Start and stop criteria with respect to length
    L_inital = 0
    L_final = sum(s_C)
    #L_final = 80;
    L = L_inital

    #Find the start location on the map
    S = 0
    len_sharp_length = len(sharp_length)
    # for p in range(0, len_sharp_length):
    #     S = S + sharp_length[p,1]
    #     if S > L_inital:
    #         if p==0:
    #             s = L_inital #Compute initial s
    #         else:
    #             s = L_inital - sum(sharp_length[0:p-1,1]) #Compute initial s
    #         break
    # if p != 0:
    #     [a,initPar] = final_points(initPar,sharp_length[0:p-1,:])

    s = 0
    p = 0

    #Load parameters
    x0 = initPar[0]
    y0 = initPar[1]
    theta0 = initPar[2]
    kappa0 = initPar[3]
    c_C = sharp_length[p:len_sharp_length,0]
    s_C = sharp_length[p:len_sharp_length,1]
    c = c_C[0]

    #Initiate counters
    k = 0 #parameters counter
    m = 0 #coordinates counter

    #Adjust step according to sharpness
    if c != 0:
        ds = abs(ds)*np.sign(c)
        s = abs(s)*np.sign(c)

    while (L <= L_final):
        if c!=0:
            [xc[m],yc[m]] = GetGeneralClothoidCoords(x0 , y0, theta0, kappa0, c, s)
        elif abs(kappa0) > 1e-6:
            alfa = kappa0*s
            if alfa > 0:
                xc[m] = (1/kappa0)*np.cos(theta0 + math.pi/2) + np.cos(theta0+alfa-math.pi/2)/abs(kappa0) + x0
                yc[m] = (1/kappa0)*np.sin(theta0 + math.pi/2) + np.sin(theta0+alfa-math.pi/2)/abs(kappa0) + y0
            else:
                xc[m] = (1/kappa0)*np.cos(theta0 + math.pi/2) + np.cos(theta0+alfa+math.pi/2)/abs(kappa0) + x0
                yc[m] = (1/kappa0)*np.sin(theta0 + math.pi/2) + np.sin(theta0+alfa+math.pi/2)/abs(kappa0) + y0
        else:
            xc[m] = x0 + s*np.cos(theta0)
            yc[m] = y0 + s*np.sin(theta0)


        # thetac[m] = norm_ang(math.pi/2 - (theta0 + kappa0*abs(s) +(c*s*s)/2))
        # if thetac[m] > 2*math.pi:
        #     thetac[m] = thetac[m] - 2*math.pi
        # elif thetac[m] < 0:
        #     thetac[m] = thetac[m] + 2*math.pi
        # if m > 1:
        #     thetac[m] = norm_ang(math.atan2(yc[m]-yc[m-1],xc[m]-xc[m-1]))
        # thetac_teste[m] = norm_ang(theta0 + kappa0*abs(s) +(c*s*s)/2)
        #thetac[m] = thetac[m] - thetac_teste[m]
        thetac[m] = (theta0 + kappa0*abs(s) +(c*s*s)/2)
        kappac[m] = kappa0 + c*abs(s)
        sigmac[m] = c
        sc[m] = L

        s = s + ds
        L = L + abs(ds)
        if L+abs(ds) > L_final:
            aux_sh_len[0,0] = c
            aux_sh_len[0,1] = s_C[k]
            [_, pf] = final_points(np.array([x0, y0, theta0, kappa0]),aux_sh_len)
            xc[m+1] = pf[0]
            yc[m+1] = pf[1]
            thetac[m+1] = pf[2]
            kappac[m+1] = pf[3]
            sigmac[m+1] = c
            # sc[m+1] = sum(s_C[:,1])
            sc[m+1] = L
            break
        while abs(s) > s_C[k]:
            #Intial parameters of the next segment
            aux_sh_len[0,0] = c
            aux_sh_len[0,1] = s_C[k]
            [_, pf] = final_points(np.array([x0, y0, theta0, kappa0]),aux_sh_len)
            x0 = pf[0]
            y0 = pf[1]
            theta0 = pf[2]
            kappa0 = pf[3]

            #Uptade length, sharpness and step
            s = abs(s) - s_C[k]
            k = k + 1
            c = c_C[k]
            #Adjust step according to sharpness
            if c != 0:
                ds = abs(ds)*np.sign(c)
                s = abs(s)*np.sign(c)
            else:
                ds = abs(ds)
        m = m + 1

    return np.concatenate((xc[0:m+1], yc[0:m+1], kappac[0:m+1], thetac[0:m+1], sc[0:m+1], np.zeros((m+1, 1)), sigmac[0:m+1]), axis=1)

def roundabout_path(rd_rad,centro,ang_ent,ang_saida,p1,p2,r_offset1,r_offset2,delta1,delta2):

    # Input (s):
    # [0] rd_rad         - roundabout path radius
    # [1] centro         - roundabout center point: [xc, yc]
    # [2] ang_ent        - Entrance heading (theta_ent) (T1_ang)
    # (4) ang_saida      - Leaving heading (theta_lea) (T2_ang)
    # (5) p1             - Entrance point: [x_ent, y_ent] - Last point of the lane
    # (6) p2             - Leaving point: [x_lea, y_lea] - First point of the next lane
    # (7) r_offset1      - Offset radius on the inflection point in meters (entrance) (r_off_i)
    # (8) r_offset2      - Offset radius on the inflection point in meters (exit) (r_off_o)
    # (9) delta1         - Inflection point angle (entrance) (lambda_i)
    # (10) delta2        - Inflection point angle (exit) (lambda_o)

        # p1 = current_lane['final'][0,0:2]
        # T1_ang = current_lane['final'][0,2]
        # roff_i = current_lane['final'][0,4]
        # lambda_i = current_lane['final'][0,5]
        # next_lane = Road_goal['L'][0,0][0,0]
        # p2 = next_lane['init'][0,0:2]
        # T2_ang = next_lane['init'][0,2]
        # roff_o = next_lane['init'][0,4]
        # lambda_o = next_lane['init'][0,5]
    #

    #Cáculo do ângulo referente à entrada
    #Q1 = p1
    #Q2 = 4*rd_rad*a[cos(ang_ent) sin(ang_ent)]+p1
    P = centro
    d1 = abs((np.cos(ang_ent)*(centro[1]-p1[1])-np.sin(ang_ent)*(centro[0]-p1[0])))
    #d1 = abs(det([Q2-Q1;P-Q1]))/norm(Q2-Q1)
    delta = math.asin(d1/rd_rad)
    alfa = ang_ent+math.pi + delta

    delta = delta1;
    r_offset = r_offset1

    beta1 = alfa+delta

    x = (rd_rad+r_offset)*np.cos(beta1)+centro[0]
    y = (rd_rad+r_offset)*np.sin(beta1)+centro[1]

    pp1 = np.array([x, y])

    p = pp1 - centro
    alfa = norm_ang(math.atan2(p[1],p[0])+math.pi/2)

    #Calcula retas tangentes ao circulo que passem por x e y
    x = x - centro[0]
    y = y - centro[1]
    delta_m=(2*x*y)*(2*x*y)-4*(x*x-rd_rad*rd_rad)*(y*y-rd_rad*rd_rad)
    m1=(2*x*y+math.sqrt(delta_m))/(2*(x*x-rd_rad*rd_rad))
    m2=(2*x*y-math.sqrt(delta_m))/(2*(x*x-rd_rad*rd_rad))

    x1 = -(2*m1*y-2*m1*m1*x)/(2*(m1*m1+1))#Calcula os pontos de tangência
    y1 = m1*(x1-x)+y
    x1 = x1+centro[0]
    y1 = y1+centro[1]

    x2 = -(2*m2*y-2*m2*m2*x)/(2*(m2*m2+1))
    y2 = m2*(x2-x)+y
    x2= x2+centro[0]
    y2 = y2+centro[1]
    x = x + centro[0]
    y = y + centro[1]

    T1 = np.array([x1-x, y1-y])
    T2 = np.array([x2-x, y2-y])
    ang_T1 = norm_ang(math.atan2(T1[1],T1[0]));
    ang_T2 = norm_ang(math.atan2(T2[1],T2[0]));

    if norm_ang(ang_T1-ang_ent)<0:
        U1 = ang_T1
    else:
        U1 = ang_T2

    #Cáculo do angulo da tangente no ponto de saída
    # Q1 = p2;
    # Q2 = 4*rd_rad*[cos(ang_saida) sin(ang_saida)]+p2;
    # P = centro;
    # d2 = abs(det([Q2-Q1;P-Q1]))/norm(Q2-Q1);
    P = centro
    d2 = abs((np.cos(ang_saida)*(centro[1]-p2[1])-np.sin(ang_saida)*(centro[0]-p2[0])))
    delta = math.asin(d2/rd_rad)
    alfa = ang_saida - delta

    r_offset = r_offset2
    delta = delta2
    beta2 = alfa-delta

    x = (rd_rad+r_offset)*np.cos(beta2)+centro[0]
    y = (rd_rad+r_offset)*np.sin(beta2)+centro[1]

    pp2 = np.array([x, y])

    p = pp2 - centro
    alfa = norm_ang(math.atan2(p[1],p[0])+math.pi/2)

    #Calcula retas tangentes ao circule que passem por x e y
    x = x - centro[0]
    y = y - centro[1]
    delta_m=(2*x*y)*(2*x*y)-4*(x*x-rd_rad*rd_rad)*(y*y-rd_rad*rd_rad)
    m1=(2*x*y+math.sqrt(delta_m))/(2*(x*x-rd_rad*rd_rad))
    m2=(2*x*y-math.sqrt(delta_m))/(2*(x*x-rd_rad*rd_rad))

    x1 = -(2*m1*y-2*m1*m1*x)/(2*(m1*m1+1)) #Calcula os pontos de tangência
    y1 = m1*(x1-x)+y
    x1 = x1+centro[0]
    y1 = y1+centro[1]

    x2 = -(2*m2*y-2*m2*m2*x)/(2*(m2*m2+1))
    y2 = m2*(x2-x)+y
    x2= x2+centro[0]
    y2 = y2+centro[1]
    x = x + centro[0]
    y = y + centro[1]

    T1 = np.array([x1-x, y1-y])
    T2 = np.array([x2-x, y2-y])
    ang_T1 = norm_ang(math.atan2(T1[1],T1[0]))
    ang_T2 = norm_ang(math.atan2(T2[1],T2[0]))

    if norm_ang(ang_saida-(ang_T1+math.pi))<0:
        U2 = ang_T1+math.pi
    else:
        U2 = ang_T2+math.pi

    k_max = 0.16

    T1_ang = ang_ent
    T2_ang = ang_saida
    delta = T2_ang - T1_ang

    x = np.linalg.norm(centro-pp1)
    delta_trig = norm_ang(math.atan2(centro[1]-pp1[1],centro[0]-pp1[0]) - U1)
    U3 = U1+delta_trig-math.pi/2
    [Lc1, Larc, sig_max] = max_values_2(delta_trig, 1/rd_rad, x)
    aux_sh_len = np.zeros((2,2))
    aux_sh_len[0,0] = sig_max
    aux_sh_len[0,1] = (1/rd_rad)/sig_max
    aux_sh_len[1,0] = 0
    aux_sh_len[1,1] = Larc
    [_, pf]=final_points(np.array([pp1[0], pp1[1], U1, 0]),aux_sh_len)
    pnew = np.array(pf[0:2])
    delta_min1 = (1/rd_rad)*(1/rd_rad)/(2*sig_max)
    sigma1 = sig_max
    newcentro = rd_rad*np.array([np.cos(U3+math.pi), np.sin(U3+math.pi)]) + pnew
    v1 = np.linalg.norm(newcentro-centro)
    angr1 = math.asin(v1/(rd_rad+r_offset))

    x = np.linalg.norm(centro-pp2)
    delta_trig = norm_ang((U2+math.pi) - math.atan2(centro[1]-pp2[1],centro[0]-pp2[0]))
    U3 = U2 - (delta_trig)
    [Lc2,Larc,sig_max] = max_values_2(delta_trig, 1/rd_rad, x)
    aux_sh_len = np.zeros((2,2))
    aux_sh_len[0,0] = -sig_max
    aux_sh_len[0,1] = (1/rd_rad)/sig_max
    aux_sh_len[1,0] = 0
    aux_sh_len[1,1] = Larc
    [_, pf]=final_points(np.array([pp2[0], pp2[1], norm_ang(U2+math.pi), 0]),aux_sh_len)
    pnew = np.array(pf[0:2])
    delta_min2 = (1/rd_rad)*(1/rd_rad)/(2*sig_max)
    sigma2 = sig_max
    newcentro = rd_rad*np.array([np.cos(U3+math.pi/2), np.sin(U3+math.pi/2)]) + pnew
    v1 = np.linalg.norm(newcentro-centro)
    angr2 = math.asin(v1/(rd_rad+r_offset))

    U1 = U1-angr1
    U2 = U2+angr2
    delta_min = delta_min1+delta_min2

    [_ ,sl_turn_aux] = CC_Trig(p1, T1_ang, pp1, U1, k_max)
    [_ ,sl_turn2_aux] = CC_Trig(pp2, U2, p2, T2_ang, k_max)
    sl_turn = abs(sl_turn_aux[1,0])
    sl_turn2 = abs(sl_turn2_aux[1,0])

    if (norm_ang(delta) > -math.pi/3 or norm_ang(delta) < -(math.pi/2+math.pi/6)):
        [it1,sl1] = CC_Trig(p1, T1_ang, pp1, U1, k_max)
        U2_aux = norm_ang(U2 - U1)
        if U2_aux < 0:
            delta_U = U2_aux + 2*math.pi
        else:
            delta_U = U2_aux


        if (delta_U - delta_min) > 0:
            sl2 = np.array([[sigma1, 0, -sigma2], [Lc1, (delta_U - delta_min)/(1/rd_rad), Lc2]])
            sl2 = np.transpose(sl2)
            [_, pfin] = final_points([pp1[0], pp1[1], U1, 0],sl2)
            xf = pfin[0]
            yf = pfin[1]
        else:
            [a,sl2] = CC_Trig(pp1, U1, pp2, U2, 1/rd_rad)
            xf = pp2[0]
            yf = pp2[1]
        [_, sl3] = CC_Trig(np.array([xf, yf]), U2, p2, T2_ang, k_max)

        sharp_length=np.concatenate(([sl1,sl2,sl3]))
    elif norm_ang(U2-U1) > 0:
        [it1,sl1] = CC_Trig(p1, T1_ang, pp1, U1, k_max)
        [_, sl2] = CC_Trig(pp1, U1, pp2, U2, 1/rd_rad)
        [_, sl3] = CC_Trig(pp2, U2, p2, T2_ang, k_max)

        sharp_length = np.concatenate(([sl1, sl2, sl3]))
    else:
        ptan = np.array([.0, .0])
        ptan[0] = rd_rad*np.cos(beta1+norm_ang(beta2-beta1)/2)+centro[0]
        ptan[1] = rd_rad*np.sin(beta1+norm_ang(beta2-beta1)/2)+centro[1]

        if np.abs(T1_ang) != np.pi/2:
            xp = (p2[1]-p1[1]+np.tan(T1_ang)*p1[0]-np.tan(T2_ang)*p2[0])/(np.tan(T1_ang)-np.tan(T2_ang))
        else:
            xp = p1[0]
        if np.abs(T2_ang) != np.pi/2:
            yp = np.tan(T2_ang)*(xp - p2[0])+p2[1]
        else:
            yp = p1[1]

        ang_ptan = beta1+norm_ang(beta2-beta1)/2+math.pi/2

        d = np.linalg.norm(np.array([xp, yp])-ptan)
        shif_ang =  math.pi/2 - norm_ang(ang_ent - ang_saida)/2
        x = np.sin(shif_ang)/(1-np.sin(shif_ang))*d
        # yp = xp
        # print(p1[1])
        # print(np.linalg.norm(np.array([xp, yp])-p2)-(1/x)/sl_turn2-1, np.linalg.norm(np.array([xp, yp])-p1)-(1/x)/sl_turn-1, x)
        # print(xp, yp)

        if (x < np.linalg.norm(np.array([xp, yp])-p1)-(1/x)/sl_turn) and (x < np.linalg.norm(np.array([xp, yp])-p2)-(1/x)/sl_turn2):
            # print('here')
            [it1,sharp_length] = CC_Turn_dif_sigma(p1, T1_ang, p2, T2_ang, 1/x, sl_turn, sl_turn2)
        else:
            ang_ptan = U1 - norm_ang(U1 - U2)/2

            r_offset = (r_offset1 + r_offset2)/2
            ptan[0] = (rd_rad+r_offset)*np.cos(beta1+norm_ang(beta2-beta1)/2)+centro[0]
            ptan[1] = (rd_rad+r_offset)*np.sin(beta1+norm_ang(beta2-beta1)/2)+centro[1]

            [it1,sl1] = CC_Trig(p1, T1_ang, ptan, ang_ptan, 0.16)
            [_, sl2] = CC_Trig(ptan, ang_ptan, p2, T2_ang, 0.16)
            sharp_length=np.concatenate([sl1, sl2])



    return it1, sharp_length

def solver_state( x0, y0, theta0, kappa0, Xc, kappaf, p ):

    iter = 0
    C = ([np.inf, np.inf, np.inf])
    s1 = 4
    s2 = 2
    s3 = 4

    # try:
    while (abs(C[0]) > 0.01) or (abs(C[1]) > 0.01) or (abs(C[2]) > np.deg2rad(1)):
        delta_der = 0.00001
        p_ant = p
        init_state = ([x0, y0, theta0, kappa0])

        kappa_midle = kappa0 + p[0]*p[2]/s1 + p[1]*p[2]/s2
        sigma3 = (kappaf - kappa_midle)/(p[2]/s3)

        param1 = np.array([[p[0]+delta_der, p[2]/s1], [p[1], p[2]/s2], [sigma3, p[2]/s3]])
        param2 = np.array([[p[0]-delta_der, p[2]/s1], [p[1], p[2]/s2], [sigma3, p[2]/s3]])
        [_,fp1] = final_points(init_state, param1)
        [_,fp2] = final_points(init_state, param2)
        dkink_dsig1 = fp1 - fp2
        dx_dsig1 = dkink_dsig1[0]/(2*delta_der)
        dy_dsig1 = dkink_dsig1[1]/(2*delta_der)
        dtheta_dsig1 = dkink_dsig1[2]/(2*delta_der)

        param1 = np.array([[p[0], p[2]/s1], [p[1]+delta_der, p[2]/s2], [sigma3, p[2]/s3]])
        param2 = np.array([[p[0], p[2]/s1], [p[1]-delta_der, p[2]/s2], [sigma3, p[2]/s3]])
        [_,fp1] = final_points(init_state, param1)
        [_,fp2] = final_points(init_state, param2)
        dkink_dsig2 = fp1 - fp2
        dx_dsig2 = dkink_dsig2[0]/(2*delta_der)
        dy_dsig2 = dkink_dsig2[1]/(2*delta_der)
        dtheta_dsig2 = dkink_dsig2[2]/(2*delta_der)

        param1 = np.array([[p[0], (p[2]+delta_der)/s1], [p[1], (p[2]+delta_der)/s2], [sigma3, (p[2]+delta_der)/s3]])
        param2 = np.array([[p[0], (p[2]-delta_der)/s1], [p[1], (p[2]-delta_der)/s2], [sigma3, (p[2]-delta_der)/s3]])
        [_,fp1] = final_points(init_state, param1)
        [_,fp2] = final_points(init_state, param2)
        dkink_ds1 = fp1 - fp2
        dx_ds1 = dkink_ds1[0]/(2*delta_der)
        dy_ds1 = dkink_ds1[1]/(2*delta_der)
        dtheta_ds1 = dkink_ds1[2]/(2*delta_der)


        jac_X = np.array([[dx_dsig1, dx_dsig2, dx_ds1],\
            [dy_dsig1, dy_dsig2, dy_ds1],\
            [dtheta_dsig1, dtheta_dsig2, dtheta_ds1]])

        pseudInv = np.linalg.inv(jac_X)

        param = np.array([[p[0], p[2]/s1], [p[1], p[2]/s2],[sigma3, p[2]/s3]])
        [_, kink] = final_points(init_state, param)
        C = Xc - kink[0:3]

        C[2] = norm_ang(C[2])
        p = - np.dot(pseudInv, np.transpose(-C)) +  p_ant
        iter = iter + 1
        if iter > 20:
            p = np.array([0, 0, 0])
            break

    # except IndexError:
    #     p = np.array([0, 0, 0])
    #     print "\033[91m ERROR SOLVER!\033[0m"

    return p, iter

def generate_maneuver( x0, y0, theta0, kappa0, x, y, theta, kappa ):

    # Input (s):
    # [0] x0         - initial x coordinate of maneuver (current lane)
    # [1] y0         - initial y coordinate of maneuver (current lane)
    # [2] theta0     - Inital heading of maneuver (current lane)
    # (4) kappa0     - Initial curvature of maneuver (current lane)

   # Output (s):
   # [0] init           - unidimensional numpy array: [x0, y0, theta0, kappa0]
   # [1] sharp_leng     - numpy array nx2, where n is the number of segments.
   #       The first column has all curvature derivative (sharpness) and the
#       second one has all segments length.

    p1 = np.array([x0, y0])
    p2 = np.array([x, y])
    Xc = np.array([x, y, theta])
    if abs(norm_ang(theta - theta0)) < 0.36:
        [init, sl] = CC_S(p1, theta0, p2, theta, 10)
        p_ant = np.array([sl[0,0], sl[1,0], sum(sl[:,1])]).transpose()
    else:
        [init, sl] = CC_Trig(p1, theta0, p2, theta, 10)
        p_ant = np.array([0, 0, sum(sl[:,1])]).transpose()

    p, iter = solver_state(x0, y0, theta0, kappa0, Xc, kappa, p_ant)

    #If error, return 0
    if sum(p) == 0:
        return np.array([-1, -1, -1, -1]), np.array([[0, 0]])

    kappaf = kappa
    param1 = np.array([[p[0], p[2]/4], [p[1], p[2]/2]])

    init = np.array([x0, y0, theta0, kappa0])
    [_,kink_sigma3] = final_points(init, param1)
    sigma3 = (kappaf - kink_sigma3[3])/(p[2]/4)

    sharp_len = np.array([[p[0], p[2]/4], [p[1], p[2]/2],[sigma3, p[2]/4]])
    # [xt,yt,thetat,kappat,sigmat,st]=GetClothoidPath(init, sharp_len, .1)

    return init, sharp_len

# xt, yt, thetat, kappat, sigmat, st = generate_maneuver(0, 0, 0, 0, 90, 90, 3.14/2, 0)
# plt.plot(xt, yt)
# plt.axis('equal')
# plt.show()
# p1 = np.array([0, 0])
# p2 = np.array([50, 4])
# Xc = np.array([52, 4, 0])
# [init, sl] = CC_S(p1, 0, p2, 0, 10)
# p_ant = np.array([sl[0,0], sl[1,0], sum(sl[:,1])]).transpose()
# p, iter = solver_state(0, 0, 0, 0, Xc, 0, p_ant)
#
# kappaf = 0
# param1 = np.array([[p[0], p[2]/4], [p[1], p[2]/2]])
# [_,kink_sigma3] = final_points(init, param1)
# sigma3 = (kappaf - kink_sigma3[3])/(p[2]/4)
#
# sharp_len = np.array([[p[0], p[2]/4], [p[1], p[2]/2],[sigma3, p[2]/4]])
# [xt,yt,thetat,kappat,sigmat,st]=GetClothoidPath(init, sharp_len, .1)

# plt.plot(xt, yt)
# plt.show()
# print p
# print sl
# print iter
# [xt,yt,thetat,kappat,sigmat,st]=GetClothoidPath(init, shlength, .2)
#
# plt.plot(xt, yt)
# plt.show()
