# -*- coding: utf-8 -*-
import numpy as np
from gurobipy import *
import time
import math
import csv
import copy
import matplotlib.pyplot as plt
from scipy.spatial.distance import pdist
from scipy import signal

# os.chdir(r'C:\Users\Junior Anderson\Dropbox\Doutorado USP\Programas\NonLinearMPC\codigo-python')
def pre_solve( path, kappa, arclength ):

# PRE_SOLVE Generates good initialization for curvature optimization
#
#    Input(s):
#    (1) path      - X and Y position of points, where each line is a (X, Y) pair;
#    (2) kappa     - Column vector with curvatures for each point;
#    (3) arclength - Column vector with arclength from first point to each point.
#
#    Output(s):
#    (1) PreSolveResults - Structure containing the presolve solutions.
#            PreSolveResults.magnitude  - Distance between adjacent points;
#            PreSolveResults.rot_matrix - Rotation matrix from world frame to path frame;
#            PreSolveResults.kappa      - Curvature initialization for optimize_curvature function.
#
#    Author(s):
#    (1) Júnior A. R. Da Silva
#    (2) Carlos M. Massera
#
#    Copyright owned by Universidade de São Paulo

    #print('Pre-solver')
    timer = time.time()

    # Create a new model
    m = Model("lp")

    # Convergence constants
    kMaxDeltaKappa = 0.02  # Maximum curvature variation per iteration [1/m]
    kTolNormInf = 0.001     # Error tolerance for stop criteria [m]
    kMaxIter = 30          # Maximum number of iterations

    # Get data size
    data_size = len(path) - 1

    # Initialize output structure
    # PreSolveResults = struct();

    # Calculate integration matrix
    magnitude = np.diff(arclength)
    # PreSolveResults.magnitude = magnitude;  # Save magnitude since this is used later again

    # Reconstruct points from curvature data for reference
    # Note: Using cumsum instead of matrix S saves a lot of CPU cycles
    theta = np.cumsum(magnitude * kappa);  # Equivalent to: theta = S * kappa;
    x = np.cumsum(magnitude * np.cos(theta));  # Equivalent to: S * cos(theta);
    y = np.cumsum(magnitude * np.sin(theta));  # Equivalent to: S * sin(theta);
    # print(x)
    # Find rotation that recover inital points
    # Note: since each point is a row, rotation matrix mut be transposed and multiplied on
    #       the left hand side.
    r_ang = math.atan2(np.diff(path[0:2,1]), np.diff(path[0:2,0])) - math.atan2(y[0], x[0])
    cr = np.cos(r_ang)
    sr = np.sin(r_ang)
    rot_matrix = np.array([[cr, -sr], [sr, cr]])

    # print(np.array([x, y]) * rot_matrix.transpose())
    # PreSolveResults.rot_matrix = rot_matrix;  # Save rot_matrix since this is used later again
    error = np.linalg.norm( (rot_matrix.dot(np.array([x, y]))).transpose() - path[1:len(path), :], np.inf)

    # If error is sufficently small just exit
    if error < kTolNormInf:
        # PreSolveResults.kappa = kappa  # Return initial curvature values
        #print('    Early completion - Error tolerance met.')
        return rot_matrix, kappa, magnitude

    kappa_out = kappa

    # Execute Sequential Linear Programming
    for i in range(0,kMaxIter):
        timer = time.time()

        # Define parameters
        kappa_lin = kappa_out
        theta_sk = np.cumsum(magnitude * kappa_lin)
        cos_sk = np.cos(theta_sk)
        sin_sk = np.sin(theta_sk)

        # Create a new model
        m = Model("lp")

        # Construct the optimization problem apriori to improve performance
        # Variables
        kappa_v = m.addVars(data_size, lb=-GRB.INFINITY, ub=GRB.INFINITY)       # Optimization variable
        theta_k = m.addVars(data_size, lb=-GRB.INFINITY, ub=GRB.INFINITY)     # (S * (kappa - kappa_lin))
        x_k = m.addVars(data_size+1, lb=-GRB.INFINITY, ub=GRB.INFINITY)        # (S * cos(theta))
        y_k = m.addVars(data_size+1, lb=-GRB.INFINITY, ub=GRB.INFINITY)       # (S * sin(theta))
        norm_slack = m.addVars(data_size+1, lb=-GRB.INFINITY, ub=GRB.INFINITY)  # Slack for norm approximation
        path_k = m.addVars(data_size+1,2, lb=-GRB.INFINITY, ub=GRB.INFINITY)

        # Minimize norm 1 path recovery error error
        obj =  norm_slack.sum('*')
        # obj = 0
        # for p in range(data_size+1):
        #     obj = obj + norm_slack[p]

        m.setObjective(obj, GRB.MINIMIZE)

        for p in range(data_size+1):
            path_k[p,0] = rot_matrix[0,:].dot(([x_k[p],y_k[p]]))# +  rot_matrix[0,1]*y_k[p]
            path_k[p,1] = rot_matrix[1,:].dot(([x_k[p],y_k[p]]))

        # Constraint kappa variations to avoid leaving the linearization point and ensure the defnition
        #     of theta_k holds
        # Note that theta_k = cumsum(maginitude .* (kappa_v - kappa_lin)
        #     using the identity diff(cumsum(x)) = x(2:end)
        #     we can constraint diff(theta_k) = maginitude .* (kappa_v - kappa_lin) for 2:end
        #     and separatedely constraint theta_k(1).
        #     This ensures higher sparsity of the KKT matrix and improve execution times for LP solver
        m.addConstrs( ( -norm_slack[p] <= path_k[p,0] - path[p,0] ) for p in range(data_size+1) )
        m.addConstrs( ( -norm_slack[p] <= path_k[p,1] - path[p,1] ) for p in range(data_size+1) )
        m.addConstrs( ( path_k[p,0] - path[p,0] <= norm_slack[p] ) for p in range(data_size+1) )
        m.addConstrs( ( path_k[p,1] - path[p,1] <= norm_slack[p] ) for p in range(data_size+1) )
        m.addConstr( theta_k[0] == magnitude[0] * (kappa_v[0] - kappa_lin[0]) )
        m.addConstrs( theta_k[p]-theta_k[p-1] == magnitude[p] * (kappa_v[p] - kappa_lin[p]) for p in range(1,data_size) )
        m.addConstrs( x_k[p+1] - x_k[p] == magnitude[p] * (cos_sk[p] - sin_sk[p] * theta_k[p]) for p in range(0, data_size) )
        m.addConstrs( y_k[p+1] - y_k[p] == magnitude[p] * (sin_sk[p] + cos_sk[p] * theta_k[p]) for p in range(0, data_size) )
        m.addConstrs( -kMaxDeltaKappa <= kappa_v[p] - kappa_lin[p] for p in range(0, data_size) )
        m.addConstrs( kappa_v[p] - kappa_lin[p] <= kMaxDeltaKappa for p in range(0, data_size) )

        m.Params.OutputFlag = 0

        # Call the solver
        m.optimize()

        for p in range(data_size):
            kappa_out[p] = kappa_v[p].x

        # Calculate new error
        theta = np.cumsum(magnitude * kappa_out)  # Equivalent to: theta = S * kappa;
        x = np.cumsum(magnitude * np.cos(theta))  # Equivalent to: S * cos(theta);
        y = np.cumsum(magnitude * np.sin(theta))  # Equivalent to: S * sin(theta);

        error = np.linalg.norm( (rot_matrix.dot(np.array([x, y]))).transpose() - path[1:len(path), :], np.inf)
        #print('    Iteration ' + str(i+1) + ' - Error: ' + str(error) + ' Elapsed: ' + \
        #      str((time.time()-timer) * 100000.0 / 100.0) + ' miliseconds.')


        if error < kTolNormInf:
            kappa = kappa_out  # Output optimized kappa
            #print('    Completed - Error tolerance met.')
            return rot_matrix, kappa, magnitude
    #end for
    kappa = kappa_out  # Output optimized kappa
    #print('    Completed - Exceeded number of iterations.')
    return rot_matrix, kappa, magnitude

def optimize_curvature( path, max_deviation ):

# OPTIMIZE_CURVATURE Generates a locally optimal trajectory with bounded deviation from points
#
#    Input(s):
#    (1) path          - X and Y position of points, where each line is a (X, Y) pair;
#    (2) max_deviation - Maximum inifnity norm error between trajectory and original points.
#
#    Output(s):
#    (1) Trajectory - Structure containing the data defining the optimal trajectory.
#            Trajectory.position  - Matrix where each line is the position of a point;
#            Trajectory.theta     - Matrix where each line is the heading of a point;
#            Trajectory.arclength - Matrix where each line is the arclength of a point;
#            Trajectory.kappa     - Matrix where each line is the curvature of a point;
#            Trajectory.dkappa    - Matrix where each line is the sharpness of a point.
#
#    Author(s):
#    (1) Júnior A. R. Da Silva
#    (2) Carlos M. Massera
#
#    Copyright owned by Universidade de São Paulo

    # Set constants for optimization
    kMaxDeltaKappa = 0.02;  # Maximum curvature variation per iteration [1/m]
    kTolNormInf = 5e-4;     # Error tolerance for stop criteria [1/m]
    kMaxIter = 50;          # Maximum number of iterations

    # Get data size
    # Note that size is one smaller than data (you only need n arcs to connect n+1 points)
    data_size = len(path) - 1

    # Check if path is closed
    is_closed = (np.linalg.norm(path[0,:] - path[len(path)-1,:], np.inf) < kTolNormInf)

    # Translate path to origin
    origin = path[0, :]
    path = path - np.ones((data_size+1, 1)) * origin

    # Get initial curvature guess
    arclength, kappa = initial_guess( path )
    kappa = kappa[0:len(kappa)-1]

    # Pre-solve for improved accuracy
    rot_matrix, kappa, magnitude = pre_solve( path, kappa, arclength )
    # print(kappa)
    #print('Sparsity solver')
    timer = time.time()

    # Calculate second derivative matrix
    D = np.eye(data_size)
    D = np.linalg.lstsq(np.diag(magnitude[0:len(magnitude)-1]), (D[0:len(D)-1, :] - D[1:len(D), :]))[0]
    D = np.linalg.lstsq(np.diag(magnitude[0:len(magnitude)-2]), (D[0:len(D)-1, :] - D[1:len(D), :]))[0]
    # print(D)

    kappa_out = kappa
    weight = np.ones(data_size - 2)

    # Execute Sequential Linear Programming
    exit_success = False

    for i in range(0,kMaxIter):#
        timer = time.time()
        # Create a new model
        m = Model("lp")

        # Construct the optimization problem apriori to improve performance
        # Variables
        kappa_v = m.addVars(data_size, lb=-GRB.INFINITY, ub=GRB.INFINITY)       # Optimization variable
        theta_k = m.addVars(data_size, lb=-GRB.INFINITY, ub=GRB.INFINITY)       # (S * (kappa - kappa_lin))
        x_k = m.addVars(data_size+1, lb=-GRB.INFINITY, ub=GRB.INFINITY)         # (S * cos(theta))
        y_k = m.addVars(data_size+1, lb=-GRB.INFINITY, ub=GRB.INFINITY)         # (S * sin(theta))
        norm_slack = m.addVars(data_size-2, lb=-GRB.INFINITY, ub=GRB.INFINITY)  # Slack for norm approximation
        path_k = m.addVars(data_size+1,2, lb=-GRB.INFINITY, ub=GRB.INFINITY)
        x_error = m.addVars(data_size+1,2, lb=-GRB.INFINITY, ub=GRB.INFINITY)
        y_error = m.addVars(data_size+1,2, lb=-GRB.INFINITY, ub=GRB.INFINITY)
        D_kappa_v = m.addVars(data_size-2, lb=-GRB.INFINITY, ub=GRB.INFINITY)

        # Initialize auxiliary variables
        kappa_lin = copy.copy(kappa_out)
        theta_sk = np.cumsum(magnitude * kappa_lin)
        cos_sk = np.cos(theta_sk)
        sin_sk = np.sin(theta_sk)

        #Update weights

        weight = 1 / (weight * ( np.abs(D.dot( kappa_lin )) ) + 1e-6)
        weight = weight * (data_size - 2) / np.sum(weight)


        # Construct point coordinates using first order Taylor expansion
        for p in range(data_size+1):
            path_k[p,0] = rot_matrix[0,:].dot(([x_k[p],y_k[p]]))
            path_k[p,1] = rot_matrix[1,:].dot(([x_k[p],y_k[p]]))

        # Define error variables
        for p in range(data_size+1):
            x_error[p] = path_k[p,0] - path[p,0]
            y_error[p] = path_k[p,1] - path[p,1]

        # Define curvature second derivative variable
        for p in range(data_size-2):
            sum_D_kappa_v = 0
            for q in range(p,p+3):
                sum_D_kappa_v = sum_D_kappa_v + D[p,q]*kappa_v[q]
            D_kappa_v[p] = sum_D_kappa_v

        # Set optimizer object properties
        # Note: We do not care about the first point since it is fixed
        obj = 0
        for p in range(data_size - 2):
            obj = obj + norm_slack[p]*weight[p]
        m.setObjective(obj, GRB.MINIMIZE)

        m.addConstrs( ( -norm_slack[p] <= D_kappa_v[p] ) for p in range(data_size-2) )
        m.addConstrs( ( D_kappa_v[p] <= norm_slack[p] ) for p in range(data_size-2) )
        m.addConstrs( ( -max_deviation <= x_error[p] ) for p in range(data_size+1) )
        m.addConstrs( ( x_error[p] <= max_deviation ) for p in range(data_size+1) )
        m.addConstrs( ( -max_deviation <= y_error[p] ) for p in range(data_size+1) )
        m.addConstrs( ( y_error[p] <= max_deviation ) for p in range(data_size+1) )
        m.addConstr( theta_k[0] == magnitude[0] * (kappa_v[0] - kappa_lin[0]) )
        m.addConstrs( theta_k[p]-theta_k[p-1] == magnitude[p] * (kappa_v[p] - kappa_lin[p]) for p in range(1,data_size) )
        m.addConstrs( x_k[p+1] - x_k[p] == magnitude[p] * (cos_sk[p] - sin_sk[p] * theta_k[p]) for p in range(0, data_size) )
        m.addConstrs( y_k[p+1] - y_k[p] == magnitude[p] * (sin_sk[p] + cos_sk[p] * theta_k[p]) for p in range(0, data_size) )
        m.addConstrs( -kMaxDeltaKappa <= kappa_v[p] - kappa_lin[p] for p in range(0, data_size) )
        m.addConstrs( kappa_v[p] - kappa_lin[p] <= kMaxDeltaKappa for p in range(0, data_size) )

        # If is path closed, enforce continuity constraints
        if is_closed:
            m.addConstr( kappa_v[0] == kappa_v[len(kappa_v)] )
            m.addConstrs( ( path_k[0,p] == path_k[len(path_k),p] ) for p in range(2))

        m.Params.OutputFlag = 0

        # Call the solver
        m.optimize()

        for p in range(data_size):
                kappa_out[p] = kappa_v[p].x

        error = np.max(abs(kappa_out - kappa_lin))
        #print('    Iteration ' + str(i+1) + ' - Error: ' + str(error) + ' Elapsed: ' + \
        #      str((time.time()-timer) * 100000.0 / 100.0) + ' miliseconds.')

        # Check if error is sufficiently small
        if error <= kTolNormInf:
            exit_success = True
            #print('    Completed - Error tolerance met.')
            break

    #if exit_success == False:
    #    print('    Completed - Exceeded number of iterations.')


    # Calculate the last curvature
    last_kappa = kappa_out[data_size-1] + \
                 np.diff(kappa_out[data_size-2:data_size])/ \
                 arclength[data_size-2] * arclength[data_size-1]
    kappa_out = np.concatenate(( kappa_out, last_kappa ))

    # Calculate the sharpness
    dkappa_out = np.diff(kappa_out) / np.diff(arclength)
    last_dkappa_out = ([dkappa_out[len(dkappa_out)-1]])
    dkappa_out = np.concatenate(( dkappa_out, last_dkappa_out ))

    # Initialize auxiliary variables
    theta = np.cumsum(magnitude * kappa_out[0:len(kappa_out)-1])
    x = np.cumsum(magnitude * np.cos(theta))  # Equivalent to: S * cos(theta);
    y = np.cumsum(magnitude * np.sin(theta))  # Equivalent to: S * sin(theta);

    r_ang = math.atan2(np.diff(path[0:2,1]), np.diff(path[0:2,0])) - math.atan2(y[0], x[0])


    x = np.concatenate((np.zeros(1), x), axis=0)
    y = np.concatenate((np.zeros(1), y), axis=0)

    pos = (rot_matrix.dot(np.array([x, y]))).transpose() + \
        np.ones((data_size+1, 1)) * origin

    theta = np.concatenate( (np.zeros(1), theta) ) + \
        np.ones(data_size+1) * r_ang

    # theta = np.zeros(len(theta))
    # theta = np.arctan2( np.diff(y),np.diff(x) )

    # theta = np.arctan2( np.diff(pos[:,1]),np.diff(pos[:,0]) )
    for p in range(len(theta)-1):
        theta[p] = np.arctan2(pos[p+1,1]-pos[p,1], pos[p+1,0]-pos[p,0])
        theta[p] = norm_ang(theta[p])

    return pos, theta, arclength, kappa_out, dkappa_out

def optimize_speed( kapparef, sigmaref, arclength, is_closed ):
# OPTIMIZE_SPEED Calculate longitudinal velocity, acceleration and jerk based on point locations
#
#    Input(s):
#    (1) kappa_vec - Curvature of points, where each line is a point;
#    (2) s_vec     - Arclength of points w.r.t start, where each line is a point;
#    (1) is_closed - Boolean flag indicating if path is closed loop.
#
#    Output(s):
#    (1) Profile - Structure containing the data defining the optimal speed profile.
#            Profile.t - Timestamps for points;
#            Profile.velocity_x - Longitudinal velocity [m/s];
#            Profile.accel_x - Longitudinal acceleration [m/s^2];
#            Profile.jerk_x - Longitudinal jerk [m/s^3].
#
#    Author(s):
#    (1) Júnior A. R. Da Silva
#    (2) Carlos M. Massera
#
#    Copyright owned by Universidade de São Paulo

    s_vec = arclength

    v_ref = 10
    jerkLatMax = 0.4
    accLatMax = 2.0
    kMaximumAx = 1.5
    kMaximumJx = 1.0
    kMaxIter = 50;       # Maximum number of iterations
    kTolNormInf = 1e-2;  # Tolerance for stop criteria

    velocity_x_max = np.zeros(len(kapparef))
    for k in range(0, len(kapparef)):
        # print(min((jl_max/abs(sigmaref[k]))**(1.0/3.0),v_max))
        velocity_x_max[k] = min(min((jerkLatMax/abs(sigmaref[k]))**(1./3.),\
            v_ref),np.sqrt(accLatMax/abs(kapparef[k])))

    # for p in range(1):
    #     velocity_x_max[p] = 0.5
    # plt.figure(2)
    # plt.plot(arclength, velocity_x_max,'r')
    # Data vector size
    n_x = len(kapparef)

    # Arclength variation
    d_s = np.diff(s_vec)

    # Set previous speed to zero
    v_x_prev = np.zeros(len(velocity_x_max))
    v_x_out = copy.copy(velocity_x_max)

    # Initialize jounce weights to one
    # Jounce weighting (for piecewise jerk)
    jounce_w = np.ones(n_x - 3)       # Jounce weighting for zero norm approximation

    # Sequantial LP solver outter loop
    exit_success = False

    for i in range(kMaxIter):

        timer = time.time()

        # Create a new model
        m = Model("lp")

        # Profile variables
        velocity_x = m.addVars(n_x, lb=-GRB.INFINITY, ub=GRB.INFINITY)       # Longitudinal velocity [m/s]
        # velocity_x_prev = m.addVars(n_x, lb=-GRB.INFINITY, ub=GRB.INFINITY)  # Previous longitudinal velocity [m/s]
        accel_x = m.addVars(n_x-1, lb=-GRB.INFINITY, ub=GRB.INFINITY)        # Longitudinal acceleration [m/s^2]
        jerk_x = m.addVars(n_x-2, lb=-GRB.INFINITY, ub=GRB.INFINITY)         # Longitudinal jerk [m/s^3]
        jounce_x = m.addVars(n_x-3, lb=-GRB.INFINITY, ub=GRB.INFINITY)       # Longitudinal jounce [m/s^4]
        d_v = m.addVars(n_x-1, lb=-GRB.INFINITY, ub=GRB.INFINITY)
        d_a = m.addVars(n_x-2, lb=-GRB.INFINITY, ub=GRB.INFINITY)
        d_j = m.addVars(n_x-3, lb=-GRB.INFINITY, ub=GRB.INFINITY)
        abs_jounce_x = m.addVars(n_x-3, lb=-GRB.INFINITY, ub=GRB.INFINITY)

        # Define initial guess as maximum available speed
        velocity_x_prev = copy.copy(v_x_out)

        # Dynamics
        # Since a = dv/dt = (dv/ds)(ds/dt) = (dv/ds) v
        # We've got dv = a ds / v
        for p in range(n_x-1):
            d_v[p] = d_s[p] * (accel_x[p] / velocity_x_prev[p])

        # And similarly for acceleration and jerk
        for p in range(n_x-2):
            d_a[p] = d_s[p] * (jerk_x[p] / velocity_x_prev[p])
        for p in range(n_x-3):
            d_j[p] = d_s[p] * (jounce_x[p] / velocity_x_prev[p])


        # Setup optimization problem
        # Minimize -||velocity_x||_1 + ||jounce_x||_0
        m.addConstrs( ( 0 <= velocity_x[p] ) for p in range(n_x) )
        m.addConstrs( ( velocity_x[p] <= velocity_x_max[p] ) for p in range(n_x) )
        m.addConstrs( ( -kMaximumAx <= accel_x[p] ) for p in range(n_x-1) )
        m.addConstrs( ( accel_x[p] <= kMaximumAx ) for p in range(n_x-1) )
        m.addConstrs( ( -kMaximumJx <= jerk_x[p] ) for p in range(n_x-2) )
        m.addConstrs( ( jerk_x[p] <= kMaximumJx ) for p in range(n_x-2) )
        m.addConstrs( ( velocity_x[p+1] - velocity_x[p] == d_v[p] ) for p in range(n_x-1) )
        m.addConstrs( ( accel_x[p+1] - accel_x[p] == d_a[p] ) for p in range(n_x-2) )
        m.addConstrs( ( jerk_x[p+1] - jerk_x[p] == d_j[p] ) for p in range(n_x-3) )
        for p in range(n_x-3):
            m.addGenConstrAbs( abs_jounce_x[p],  jounce_x[p] )

        # Closed loop constraints
        if is_closed:
            # If trajectory is closed loop, keep speed and accel continuous
            m.addConstr( velocity_x[0] == velocity_x[n_x] )
            m.addConstr( accel_x[0] == accel_x[n_x-1] )
        else:
            m.addConstr( velocity_x[0] == 0.5 )
            m.addConstr( accel_x[0] == 0.5 )
            m.addConstr( jerk_x[0] == 0.5 )
            m.addConstr( velocity_x[n_x-1] == 0 )
            m.addConstr( accel_x[n_x-2] == 0 )

        # We want maximum speed and piecewise linear acceleration, therefore
        objective  = 0
        for p in range(n_x):
            if p < n_x-3:
                objective = objective - velocity_x[p] + (abs_jounce_x[p]*abs(jounce_w[p]))        # And minimize approximation of jounce zero norm
            else:
                objective = objective - velocity_x[p]
        m.setObjective(objective, GRB.MINIMIZE)

        # Call the solver
        # Take half step for better convergence since we don't have step constraints
        m.Params.OutputFlag = 0
        m.optimize()
        v_x_prev = copy.copy(v_x_out)

        for p in range(n_x):
            # print(velocity_x[p].x)
            v_x_out[p] = (velocity_x[p].x + v_x_out[p]) / 2

        # Calculate current values
        t_vec = np.concatenate((np.zeros(1), np.cumsum(d_s / v_x_out[0:len(v_x_out)-1])), axis=0)
        accel_x = np.diff(v_x_out) / np.diff(t_vec)
        jerk_x = np.diff(accel_x) / np.diff(t_vec[0:len(t_vec)-1])
        jounce_x = np.diff(jerk_x) / np.diff(t_vec[0:len(t_vec)-2])

        # Update jounce weights
        jounce_w = 1./(abs(jounce_x) + 1e-5)

        error = np.linalg.norm(v_x_prev - v_x_out, np.inf)
        print('    Iteration ' + str(i+1) + ' - Error: ' + str(error) + ' Elapsed: ' + \
                str((time.time()-timer) * 100000.0 / 100.0) + ' miliseconds.')

        # Stop if error met
        if error <= kTolNormInf:
            exit_success = True
            print('    Completed - Error tolerance met.')
            break

    if exit_success == False:
        print('    Completed - Exceeded number of iterations.')
    return v_x_out

def initial_guess( path ):
# INITIAL_GUESS Calculate curvature and arclength based on point locations
#
#    Input(s):
#    (1) path - X and Y position of points, where each line is a (X, Y) pair.
#
#    Output(s):
#    (1) kappa     - Column vector with curvatures for each point;
#    (2) arclength - Column vector with arclength from first point to each point.
#
#    Author(s):
#    (1) Júnior A. R. Da Silva
#    (2) Carlos M. Massera
#
#    Copyright owned by Universidade de São Paulo

    # Get vector between adjacent points
    vector = np.diff(path[:, 0:2], axis=0)

    # Get heading and magnitude of path vectors
    theta = np.arctan2(vector[:,1], vector[:,0])
    magnitude = np.sqrt(((vector[:,0]**2 + vector[:,1]**2)))

    # Get heading variation
    dtheta = np.diff(theta);

    # Clip between -pi and pi
    dtheta = np.mod(dtheta + math.pi, 2 * math.pi) - math.pi

    # Calculate curvature
    kappa_mag = np.sqrt(magnitude[0:len(magnitude)-1] * magnitude[1:len(magnitude)])
    kappa = 2 * np.sin(dtheta / 2) / kappa_mag

    # Calculate arc length
    arclength = np.concatenate(( [0], np.cumsum(magnitude) ))

    # Initial and end curvature calculation
    #     Initial: Solve for kappa and dkappa using 2nd and 3rd points
    A = ([1, 0],\
         [1, magnitude[1]])
    b = kappa[0:2]
    kappa_1 = np.array([1, -magnitude[0]]).dot(np.linalg.lstsq(A,b)[0])

    #     Final: Solve for kappa and dkappa using the two last available points
    A = ([1, -magnitude[len(magnitude)-2]],\
         [1, 0])
    b = kappa[len(kappa)-2:len(kappa)]
    kappa_end = np.array([1, magnitude[len(magnitude)-1]]).dot( np.linalg.lstsq(A,b)[0])

    #     Concatenate them into one vector
    kappa = np.concatenate(( ([kappa_1]), kappa, ([kappa_end]) ))

    return arclength, kappa

def norm_ang( theta ):
    if theta > math.pi:
        theta_n = theta - 2*math.pi
    elif theta < -math.pi:
        theta_n = 2*math.pi + theta
    else:
        theta_n = theta
    return theta_n

def generate_trajectory( gps_data ):

    # Trajectory parameters
    point_dist_opt = 0.2   # Distance between points for optimization
    max_deviation = 0.1   # Maximum deviation for optimization

    # Make sure we have only X and Y
    gps_data = copy.copy( low_band_filter( gps_data[:,0:2] ) )

    # Filter points by distance
    mag = np.sqrt(np.sum(np.diff(gps_data, axis=0)**2, axis=1))

    # kappa_out = np.concatenate(( kappa_out, last_kappa ))
    mask = np.concatenate(( [True],(np.diff(np.floor(np.cumsum(mag) / point_dist_opt)) != 0), [True]))

    path = gps_data[mask, :]
    arclength, kappa = initial_guess( path )

    # max_deviation = 0.1
    pos, theta, arclength, kappa_out, dkappa_out = optimize_curvature( path, max_deviation )
    v_x_out = np.array([0]*len(kappa_out))#optimize_speed( kappa_out, dkappa_out, arclength, False )
    waypoints = np.zeros((len(path),6))
    waypoints[:,0] = pos[:,0]
    waypoints[:,1] = pos[:,1]
    waypoints[:,2] = kappa_out
    waypoints[:,3] = theta
    waypoints[:,4] = arclength
    waypoints[:,5] = v_x_out

    
    # plt.figure(1)
    # plt.plot(gps_data[:,0], gps_data[:,1],'r.')
    # plt.plot(waypoints[:,0], waypoints[:,1],'b.')
    # #
    # plt.axis('equal')
    # plt.figure(2)
    # #
    # old_theta = np.arctan2( np.diff(gps_data[:,1]),np.diff(gps_data[:,0]) )
    # old_arc, old_kappa = initial_guess( gps_data )
    # #
    # plt.plot(old_arc, old_kappa, 'r.')
    # plt.plot(waypoints[:,4],waypoints[:,2],'b.')
    # # #
    # # plt.figure(3)
    # # plt.plot(waypoints[:,4], waypoints[:,2],'r')
    # # #
    # # plt.figure(4)
    # # plt.plot(waypoints[:,4], waypoints[:,5],'r')
    # plt.show()
    
    return waypoints

def low_band_filter( path ):
    
    M = copy.copy( path )

    #Eliminate outliears
    #Ssome coodinates are multiplied by 1000. We want to correct them
    # x = M[:,0]/0.0002e9
    # y = M[:,1]/0.0076e9

    n = len(M)
    # for k in range(0,n):
    #     if x[k] > 10:
    #         M[k,0] = M[k,0]/1000
    #     if y[k] > 10:
    #         M[k,1] = M[k,1]/1000

    x = M[:,0]
    y = M[:,1]
    m = 0
    k = 0
    while m < n-1:
        if x[m+1] == x[m] and y[m+1] == y[m]:
             m = m + 1
             continue
        M[k,:] = M[m,:]
        m = m + 1
        k = k + 1

    #Apply a low pass band filter to improve curvature quality
    b,a = signal.butter(6,0.05)
    x = signal.filtfilt(b,a,M[:,0])
    y = signal.filtfilt(b,a,M[:,1])
    gps_data = np.zeros((len(x),2))
    gps_data[:,0] = x
    gps_data[:,1] = y
    return gps_data

# read_file =  np.genfromtxt('pose.csv', delimiter=',')
# gps_data = copy.copy(read_file[range(1,500),:])
# waypoints = generate_trajectory( gps_data )
#
# plt.figure(1)
# plt.plot(gps_data[:,0], gps_data[:,1],'r.')
# plt.plot(waypoints[:,0], waypoints[:,1],'b.')
#
# plt.axis('equal')
# plt.figure(2)
#
# old_theta = np.arctan2( np.diff(gps_data[:,1]),np.diff(gps_data[:,0]) )
# old_arc, old_kappa = initial_guess( gps_data )
#
# plt.plot(waypoints[:,4],waypoints[:,3],'r.')
#
# plt.figure(3)
# plt.plot(waypoints[:,4], waypoints[:,2],'r')
#
# plt.figure(4)
# plt.plot(waypoints[:,4], waypoints[:,5],'r')
# plt.show()
#
# file_clothoid = 'mapa.npy'
# np.save(file_clothoid, waypoints)

# path = np.load('mapa.npy')
