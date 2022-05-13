import numpy as np
from matplotlib import pyplot as plt
import trimesh
import pickle
import mpmath 
import math
import decimal
from sympy import *
import pylab
from numpy import random, nanmax, argmax, unravel_index
from scipy.spatial.distance import pdist, squareform
from shapely.geometry import LineString
import fortranformat as ff
import csv
# import copy
# import open3d as o3d
# import pandas as pd
# from shapely.geometry import LineString
# from stl import mesh
# from mpl_toolkits import mplot3d
# from scipy.spatial.distance import euclidean
# get_ipython().run_line_magic('matplotlib', 'inline')
# get_ipython().run_line_magic('pylab', 'inline')
# get_ipython().run_line_magic('config', "InlineBackend.figure_format = 'svg'")


#--------------------------------------------------CONSTANT VARIABLEs---------------------------------
#--------------------------------------------------------------------------------------
#--------------------------------------------------DEFs---------------------------------
def PropellerMesh(propeller_model):
    mesh = trimesh.load_mesh(propeller_model)
    mesh.is_watertight
    mesh.vertices -= mesh.center_mass
    print("mesh loaded successful")
    return mesh

def rotate_and_shift(p, origin=(0, 0), degrees=0):
    angle = np.deg2rad(degrees)
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    o = np.atleast_2d(origin)
    p = np.atleast_2d(p)
#     return p - o
    return np.squeeze((R @ (p.T-o.T)).T)

def rotate_angle_of_airfoil(points):
    chord_length, l, t = LeadingTrailingEdge(points)
    delta_x = points[l][0] - points[t][0]
    delta_y = points[l][1] - points[t][1]
    theta = np.arctan(delta_y/delta_x) / np.pi * 180
    return theta, l, t, chord_length

def PropellerSlice(mesh, PropellerCrossSection, normal_vector):
    slice = mesh.section(plane_origin=[PropellerCrossSection[0], PropellerCrossSection[1], PropellerCrossSection[2]],
                         plane_normal=normal_vector)
    AirfoilSlice2D, to_3D = slice.to_planar()    
    cross_section_points = extract_points_of_slice2D(AirfoilSlice2D)   
    convert_points = [cross_section_points[1], -cross_section_points[0]]
    AirfoilPointSet = []
    for i in range(len(convert_points[0])):
        point = (convert_points[0][i], convert_points[1][i])
        AirfoilPointSet.append(point)
       
    AirfoilChordLength, l, t = LeadingTrailingEdge(AirfoilPointSet)
    
    if AirfoilPointSet[l][0] > 0:
        AirfoilPointSet = [[point[0], -point[1]] for point in AirfoilPointSet]
        AirfoilCentroid = [AirfoilCentroid[0], AirfoilCentroid[1], -AirfoilCentroid[2]]        
    return AirfoilPointSet

#--------------------------------------------------------------------------------------
#-----------------------------SubDEFs--------------------------------------------------
def extract_points_of_slice2D(slice_2D):
    poly_full = slice_2D.polygons_full
    y_poly = np.array(poly_full[0].exterior.coords.xy[1])
    x_poly = np.array(poly_full[0].exterior.coords.xy[0])
    poly_points = np.array(np.array([x_poly, y_poly]))
    return poly_points  # Shape 2 x (number of points)

# def LeadingTrailingEdgeMedianline(AirfoilPointSet, poly_deg, n_points): 
#     Chor, l, t = LeadingTrailingEdge(AirfoilPointSet)
#     as1, as2 = split_upper_and_lower(AirfoilPointSet, l, t)
    
#     # Define median line
#     min_a1_x, max_a1_x = min(as1[:,0]), max(as1[:,0])
#     new_a1_x = np.linspace(min_a1_x, max_a1_x, n_points)
#     a1_coefs = np.polyfit(as1[:,0],as1[:,1], poly_deg)
#     new_a1_y = np.polyval(a1_coefs, new_a1_x)

#     min_a2_x, max_a2_x = min(as2[:,0]), max(as2[:,0])
#     new_a2_x = np.linspace(min_a2_x, max_a2_x, n_points)
#     a2_coefs = np.polyfit(as2[:,0],as2[:,1], poly_deg)
#     new_a2_y = np.polyval(a2_coefs, new_a2_x)

#     midx = [np.mean([new_a1_x[i], new_a2_x[i]]) for i in range(n_points)]
#     midy = [np.mean([new_a1_y[i], new_a2_y[i]]) for i in range(n_points)]  
    
#     camberline = np.array([[x, y] for x, y in zip(midx, midy)])

#     #remove the unwanted points
#     TrailingEdge2D = furthest_point(camberline[-1], AirfoilPointSet)
#     LeadingEdge2D = furthest_point(camberline[0], AirfoilPointSet)

#     del_list = []
#     for i in range(-1, -len(camberline), -1):
#         if distance_two_points(camberline[i-1], TrailingEdge2D) < distance_two_points(camberline[i], TrailingEdge2D):
#             del_list.append(i) 
#         else:
#             break
#     if len(del_list) > 0:
#         camberline = camberline[:-len(del_list)]
    
#     AirfoilChordLength = distance_two_points(TrailingEdge2D, LeadingEdge2D)
#     AirfoilChordLength = Chor
#     TrailingEdge2D = AirfoilPointSet[t]
#     LeadingEdge2D = AirfoilPointSet[l]

#     # Calculate t, F
#     camberline_coefs = np.polyfit(camberline[:,0], camberline[:,1], 3)
#     a0 = camberline_coefs[0]
#     a1 = camberline_coefs[1]
#     a2 = camberline_coefs[2]
#     a3 = camberline_coefs[3]

#     t=symbols('t')
#     roft = [t, a3 + a2 * t + a1 * Pow(t,2) + a0 * Pow(t,3)]
#     rprimet = [diff(roft[0],t), diff(roft[1],t)]
#     toft = [rprimet[0]/sqrt(Pow(rprimet[0],2) + Pow(rprimet[1],2)), rprimet[1]/sqrt(Pow(rprimet[0],2) + Pow(rprimet[1],2))]
#     tprimet = [diff(toft[0],t), diff(toft[1],t)]
#     noft = [tprimet[0]/sqrt(Pow(tprimet[0],2) + Pow(tprimet[1],2)), tprimet[1]/sqrt(Pow(tprimet[0],2) + Pow(tprimet[1],2))]

#     # setup our t values array
#     t_values = np.linspace(camberline[0][0], camberline[-1][0], 50)

#     #turn our r(t) vector value symbolic math function into a python function
#     r_value_functions = [lambdify(t, roft[0]), lambdify(t, roft[1])]

#     #turn our T(t) vector value symbolic math function into a python function
#     tangent_value_functions = [lambdify(t, toft[0]), lambdify(t, toft[1])]

#     #turn our N(t) vector value symbolic math function into a python function
#     normal_value_functions = [lambdify(t, noft[0]), lambdify(t, noft[1])]
#     tset  = []
#     AirfoilCamberlineF = 0
#     # u = 0
#     for i in range(1, len(t_values), 1):
#         pos = t_values[i]
#         nexpos = t_values[i - 1]       

#         point0 = [r_value_functions[0](nexpos),r_value_functions[1](nexpos)]
#         point1 = [r_value_functions[0](pos),r_value_functions[1](pos)]
#         point2 = [r_value_functions[0](pos) + normal_value_functions[0](pos)*100, r_value_functions[1](pos) + normal_value_functions[1](pos)*100]
#         point3 = [r_value_functions[0](pos) - normal_value_functions[0](pos)*100, r_value_functions[1](pos) - normal_value_functions[1](pos)*100]
#         du = distance_two_points(point0, point1)
#         # u += du
#         p1  = intersection([point3, point2], as1) 
#         p2  = intersection([point3, point2], as2)
#         # p1  = normal_thickness_points(point1, point2, as1) 
#         # p2  = normal_thickness_points(point1, point2, as2)
#         if p1 == None or p2 == None:
#             a = None
#         else:
#             t = distance_two_points(p1, p2)
#         AirfoilCamberlineF += du * (t ** 3) / 10 ** 12

#     return AirfoilChordLength, LeadingEdge2D, TrailingEdge2D, camberline, AirfoilCamberlineF

# def normal_thickness_points(point1, point2, uppers):
#     point1 = np.array(point1)
#     point2 = np.array(point2)
#     dis1 = 1000
#     dis2 = 1000
#     p1 = uppers[0]
#     p2 = uppers[-1]
#     for point in uppers:
#         point = np.array(point)
#         dis = np.linalg.norm(np.cross(point2-point1, point1-point))/np.linalg.norm(point2-point1)
#         if dis < dis1 and dis < dis2:
#             p1 = p2
#             p2 = point
#             dis1 = dis2
#             dis2 = dis
#         elif dis < dis1 and dis >= dis2:
#             p1 = point
#             dis1 = dis    
#         p = (p1 + p2)/2         
#     return p

# def intersection(setpoint1, setpoint2):
#     y1 = [point[1] for point in setpoint1]
#     x1 = [point[0] for point in setpoint1]
#     y2 = [point[1] for point in setpoint2]
#     x2 = [point[0] for point in setpoint2]
#     line_1 = LineString(np.column_stack((x1, y1)))
#     line_2 = LineString(np.column_stack((x2, y2)))
#     intersection = line_1.intersection(line_2)
#     if intersection.geom_type == 'Point':
#         return [intersection.xy[0][0], intersection.xy[1][0]]
#     else:
#         return None  

def LeadingTrailingEdge(points):
    D = pdist(points)
    D = squareform(D);
    max_dis = nanmax(D)
    I = np.argmax(D)
    no_1, no_2 = np.unravel_index(I, D.shape)
    if points[no_1][1] < points[no_2][1]:
        return max_dis, no_2, no_1
    return max_dis, no_1, no_2


def distance_two_points(point1, point2):
    dis = np.sqrt((float(point1[0])-float(point2[0])) ** 2 + (float(point1[1])-float(point2[1])) ** 2)
    return dis


def PlotSpline(setpoints, colour):
    setpointsx = [point[0] for point in setpoints]
    setpointsy = [point[1] for point in setpoints]
    plt.plot(setpointsx[:], setpointsy[:], c = colour)
    return


def PlotScatter(p):
    plt.scatter(p[0], p[1])
    return


# def numerical_centroid(slice_2D, n_points_est):
#     return np.mean(slice_2D.sample(n_points_est), axis=0)


def split_upper_and_lower(airfoil, leading_idx, trailing_idx):
    np_airfoil = np.array(airfoil)
    n = np_airfoil.shape[0]
    np_airfoil_shifted = np.roll(np_airfoil, shift=-leading_idx, axis=0)
    leading_idx_shifted = 0
    trailing_idx_shifted = (trailing_idx - leading_idx + n) % n
    if np_airfoil_shifted[1, 1] > np_airfoil_shifted[0, 1]:
        uppers = np_airfoil_shifted[: trailing_idx_shifted + 1]
        lowers = np.roll(np_airfoil_shifted, shift=-1, axis=0)[trailing_idx_shifted-1:, :]
    else:
        np_airfoil_shifted[1:, :] = np.flip(np_airfoil_shifted[1:, :], axis=0)
        trailing_idx_shifted = n - trailing_idx_shifted
        uppers = np_airfoil_shifted[: trailing_idx_shifted + 1]
        lowers = np.roll(np_airfoil_shifted, shift=-1, axis=0)[trailing_idx_shifted-1:, :]    
    return uppers, lowers[::-1, :]

# def DefSaveData(mesh, r, R, normal_vector, step, AirFlowAngle):
#     SaveData(mesh, r, R, normal_vector, step)
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilArea.pickle', 'rb') as handle:
#         AirfoilArea = pickle.load(handle)

#     CalculateWingVolume(AirfoilArea, step)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilCentroid.pickle', 'rb') as handle:
#         AirfoilCentroid = pickle.load(handle)

#     CalculateWingCenterOfMass(AirfoilArea, AirfoilCentroid)

#     # with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilLeadingEdge.pickle', 'rb') as handle:
#     #     AirfoilLeadingEdge = pickle.load(handle)

#     # with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilTrailingEdge.pickle', 'rb') as handle:
#     #     AirfoilTrailingEdge = pickle.load(handle)

#     # CalculateWingdS(AirfoilLeadingEdge, AirfoilTrailingEdge, step)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilPointSet.pickle', 'rb') as handle:
#         AirfoilPointSet = pickle.load(handle)

#     CalculateWingdSMaxdistance(AirfoilPointSet, step)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilTwistAngle.pickle', 'rb') as handle:
#         AirfoilTwistAngle = pickle.load(handle)
    
#     CalculateAirfoilAerodynamicCoefficientDickinson(AirfoilTwistAngle, AirFlowAngle)
#     return

# def SaveData(mesh, r, R, normal_vector, step):
#     # step = 0.01
#     # r = 15
#     # R = 110
#     # normal_vector = [1, 0, 0]
    
#     # AirfoilPointSet, AirfoilLeadingEdge, AirfoilTrailingEdge, 
#     # AirfoilChordLength, AirfoilQuaterChord, AirfoilTwistAngle, AirfoilArea, 
#     # AirfoilCentroid, AirfoilMomentOfInertia, AirfoilUppers, AirfoilLowers, 
#     # PropellerCrossSection
#     cross_section_data = []
#     for i in np.arange(r, R, step):
#         j = round(i,2)
#         print(j)
#         section = PropellerSlice(mesh,[-j, 0, 0], [1, 0, 0])
#         cross_section_data.append(section)

#     # Save AirfoilPointSet
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilPointSet.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[0] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilLeadingEdge
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilLeadingEdge.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[1] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilTrailingEdge
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilTrailingEdge.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[2] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilChordLength
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilChordLength.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[3] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilQuaterChord
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilQuaterChord.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[4] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilTwistAngle
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilTwistAngle.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[5] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilArea
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilArea.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[6] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilCentroid
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilCentroid.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[7] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilMomentOfInertia
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilMomentOfInertia.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[8] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilUppers
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilUppers.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[9] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilLowers
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilLowers.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[10] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # AirfoilCamberline
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilLowers.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[11] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save PropellerCrossSection
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/PropellerCrossSection.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[12] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilElasticModulus
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilElasticModulus.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[13] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilShearModulus
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilShearModulus.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[14] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilCamberlineU
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilCamberlineU.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[15] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)

#     # Save AirfoilCamberlineF
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilCamberlineF.pickle', 'wb') as handle:
#         VairableTemp = [cross_section[16] for cross_section in cross_section_data]
#         pickle.dump(VairableTemp, handle, protocol=pickle.HIGHEST_PROTOCOL)
#     return    

# def CalculateWingVolume(AirfoilArea, step):
#     WingVolume = []
#     Volume = 0
#     for Area in reversed(AirfoilArea):
#         Volume += Area * step
#         WingVolume = WingVolume + [Volume]
#     ReverseWingVolume = WingVolume[::-1]

#     # Save WingVolume
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/WingVolume.pickle', 'wb') as handle:
#         pickle.dump(ReverseWingVolume, handle, protocol=pickle.HIGHEST_PROTOCOL)
#     return

# def CalculateWingCenterOfMass(AirfoilArea, AirfoilCentroid):
#     AirfoilArea = AirfoilArea[::-1]
#     AirfoilCentroid = AirfoilCentroid[::-1]
#     TotalArea = 0
#     Temp = [0, 0, 0]
#     WingCenterOfMass = []
#     for i in range(len(AirfoilArea)):
#         Temp += AirfoilArea[i] * np.array(AirfoilCentroid[i])
#         TotalArea += AirfoilArea[i]
#         WingCenterOfMass = WingCenterOfMass + [Temp/TotalArea]
#     ReverseWingCenterOfMass = WingCenterOfMass[::-1]

#     # Save WingCenterOfMass
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/WingCenterOfMass.pickle', 'wb') as handle:
#         pickle.dump(ReverseWingCenterOfMass, handle, protocol=pickle.HIGHEST_PROTOCOL)
#     return

# def Wingg(AirfoilPointSet):  # return the indices of leading and trailing edges
#     LEset = []
#     TEset = []
#     for idx, airfoil in enumerate(AirfoilPointSet):
#         # print(idx) 
#         np_airfoil = np.array(airfoil)
#         max_idx = np.argmax(np_airfoil, axis=0)[0]  # max x
#         min_idx = np.argmin(np_airfoil, axis=0)[0]  # min x        
#         LE = airfoil[max_idx]
#         TE = airfoil[min_idx]
#         LEset.append(LE)
#         TEset.append(TE)
#     return LEset, TEset

# def CalculateWingdSMaxdistance(AirfoilPointSet, step):
#     LEset, TEset = Wingg(AirfoilPointSet)
#     WingdS = []
#     for l, t in zip(LEset, TEset):
#         dS = np.abs((t[0] - l[0]) * step)
#         WingdS = WingdS + [dS]
#     # Save WingdS
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/WingdS.pickle', 'wb') as handle:
#         pickle.dump(WingdS, handle, protocol=pickle.HIGHEST_PROTOCOL)
#     return WingdS

# def CalculateWingdS(AirfoilLeadingEdge, AirfoilTrailingEdge, step):
#     WingdS = []
#     for l, t in zip(AirfoilLeadingEdge, AirfoilTrailingEdge):
#         dS = np.abs((t[1] - l[1]) * step)
#         WingdS = WingdS + [dS]
#     # Save WingdS
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/WingdS.pickle', 'wb') as handle:
#         pickle.dump(WingdS, handle, protocol=pickle.HIGHEST_PROTOCOL)
#     return WingdS


# def CalculateAirfoilAerodynamicCoefficientDickinson(AirfoilTwistAngle, AirFlowAngle):
#     AirfoilAerodynamicCoefficientDickinson = []
#     for Theta in AirfoilTwistAngle:
#         AirAngleOfAttack =  Theta - AirFlowAngle
#         AirfoilCl = 0.225 + 1.58 * np.sin(math.radians(2.13 * AirAngleOfAttack - 7.2))
#         AirfoilCd = 1.92 - 1.55 * np.cos(math.radians(2.04 * AirAngleOfAttack - 9.82))
#         AirfoilCn = AirfoilCl * np.cos(math.degrees(AirFlowAngle)) + AirfoilCd * np.sin(math.degrees(AirFlowAngle))
#         AirfoilCt = - AirfoilCl * np.sin(math.degrees(AirFlowAngle)) + AirfoilCd * np.cos(math.degrees(AirFlowAngle))
#         AirfoilAerodynamicCoefficientDickinson.append([AirfoilCn, AirfoilCt, AirfoilCl, AirfoilCd])
    
#     # Save AirfoilAerodynamicCoefficientDickinson
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/AirfoilAerodynamicCoefficientDickinson.pickle', 'wb') as handle:
#         pickle.dump(AirfoilAerodynamicCoefficientDickinson, handle, protocol=pickle.HIGHEST_PROTOCOL)
#     return AirfoilAerodynamicCoefficientDickinson


# def CalculatePropellerForces(WingdS, AirfoilAerodynamicCoefficientDickinson, PropellerCrossSection, Airrho, RotationalSpeed):
#     Constant = Airrho * (RotationalSpeed * np.pi * 2 / 60)**2
#     dNormalForce = 0
#     dTangentialForce = 0
#     dLiftForce = 0
#     dDragForce = 0
#     for dS, Coefficients, x in zip(WingdS, AirfoilAerodynamicCoefficientDickinson, PropellerCrossSection):
#         dNormalForce += Coefficients[0] * dS * 1e-6 * (x[0] * 1e-3)**2
#         dTangentialForce += Coefficients[1] * dS * 1e-6 * (x[0] * 1e-3)**2
#         dLiftForce += Coefficients[2] * dS * 1e-6 * (x[0] * 1e-3)**2
#         dDragForce += Coefficients[3] * dS * 1e-6 * (x[0] * 1e-3)**2

#     PropellerNormalForce = Constant * dNormalForce 
#     PropellerTangentialForce = Constant * dTangentialForce 
#     PropellerLiftForce = Constant * dLiftForce 
#     PropellerDragForce = Constant * dDragForce 

#     return PropellerNormalForce, PropellerTangentialForce, PropellerLiftForce, PropellerDragForce

# def NineInchPropEvaluation():
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/AirfoilArea.pickle', 'rb') as handle:
#         AirfoilArea = pickle.load(handle)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/AirfoilCentroid.pickle', 'rb') as handle:
#         AirfoilCentroid = pickle.load(handle)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/WingVolume.pickle', 'rb') as handle:
#         WingVolume = pickle.load(handle)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/WingCenterOfMass.pickle', 'rb') as handle:
#         WingCenterOfMass = pickle.load(handle)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/AirfoilLeadingEdge.pickle', 'rb') as handle:
#         AirfoilLeadingEdge = pickle.load(handle)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/AirfoilTrailingEdge.pickle', 'rb') as handle:
#         AirfoilTrailingEdge = pickle.load(handle)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/WingdS.pickle', 'rb') as handle:
#         WingdS = pickle.load(handle)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/AirfoilTwistAngle.pickle', 'rb') as handle:
#         AirfoilTwistAngle = pickle.load(handle)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/PropellerCrossSection.pickle', 'rb') as handle:
#         PropellerCrossSection = pickle.load(handle)

#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/AirfoilAerodynamicCoefficientDickinson.pickle', 'rb') as handle:
#         AirfoilAerodynamicCoefficientDickinson = pickle.load(handle)
#     NF = []
#     TF = []
#     LF = []
#     DF = []
#     EF = []
#     errors = []
#     ExperimentalRigidForce = [0.6887, 0.7613, 0.8279, 0.9254, 0.9936, 1.093,
#         1.1485, 1.2544, 1.3124, 1.4102, 1.5185, 1.6272, 1.7099]

#     for RotationalSpeed in range(2000, 3300, 100):
#         Forces = CalculatePropellerForces(WingdS, AirfoilAerodynamicCoefficientDickinson, PropellerCrossSection, Airrho, RotationalSpeed)
#         NF.append([RotationalSpeed, Forces[0]])
#         TF.append([RotationalSpeed, Forces[1]])
#         LF.append([RotationalSpeed, Forces[2]])
#         DF.append([RotationalSpeed, Forces[3]])
#         EF.append([RotationalSpeed, ExperimentalRigidForce[int((RotationalSpeed - 2000)/100)]])
#         err = np.abs((Forces[2] - ExperimentalRigidForce[int((RotationalSpeed - 2000)/100)])/ExperimentalRigidForce[int((RotationalSpeed - 2000)/100)]*100)
#         errors.append([RotationalSpeed, err])
#     # PlotSpline(NF, "blue")
#     # [PlotScatter(NFi) for NFi in NF]
#     # PlotSpline(TF, "red")
#     # [PlotScatter(TFi) for TFi in TF]
#     PlotSpline(LF, "blue")
#     [PlotScatter(LFi) for LFi in LF]
#     # PlotSpline(DF, "black")
#     # [PlotScatter(DFi) for DFi in DF]
#     PlotSpline(EF, "red")
#     [PlotScatter(DFi) for DFi in EF]
#     PlotSpline(errors, "black")
#     [PlotScatter(DFi) for DFi in errors]
#     return

# def PropEvaluation(folder_address):

#     with open(f'{folder_address}/AirfoilArea.pickle', 'rb') as handle:
#         AirfoilArea = pickle.load(handle)

#     with open(f'{folder_address}/AirfoilCentroid.pickle', 'rb') as handle:
#         AirfoilCentroid = pickle.load(handle)

#     with open(f'{folder_address}/WingVolume.pickle', 'rb') as handle:
#         WingVolume = pickle.load(handle)

#     with open(f'{folder_address}/WingCenterOfMass.pickle', 'rb') as handle:
#         WingCenterOfMass = pickle.load(handle)

#     with open(f'{folder_address}/AirfoilLeadingEdge.pickle', 'rb') as handle:
#         AirfoilLeadingEdge = pickle.load(handle)

#     with open(f'{folder_address}/AirfoilTrailingEdge.pickle', 'rb') as handle:
#         AirfoilTrailingEdge = pickle.load(handle)

#     with open(f'{folder_address}/WingdS.pickle', 'rb') as handle:
#         WingdS = pickle.load(handle)

#     with open(f'{folder_address}/AirfoilTwistAngle.pickle', 'rb') as handle:
#         AirfoilTwistAngle = pickle.load(handle)

#     with open(f'{folder_address}/PropellerCrossSection.pickle', 'rb') as handle:
#         PropellerCrossSection = pickle.load(handle)

#     with open(f'{folder_address}/AirfoilAerodynamicCoefficientDickinson.pickle', 'rb') as handle:
#         AirfoilAerodynamicCoefficientDickinson = pickle.load(handle)
#     NF = []
#     TF = []
#     LF = []
#     DF = []
#     EF = []
#     errors = []
#     ExperimentalRigidForce = [0.6887, 0.7613, 0.8279, 0.9254, 0.9936, 1.093,
#         1.1485, 1.2544, 1.3124, 1.4102, 1.5185, 1.6272, 1.7099]

#     for RotationalSpeed in range(2000, 3300, 100):
#         Forces = CalculatePropellerForces(WingdS, AirfoilAerodynamicCoefficientDickinson, PropellerCrossSection, Airrho, RotationalSpeed)
#         NF.append([RotationalSpeed, Forces[0]])
#         TF.append([RotationalSpeed, Forces[1]])
#         LF.append([RotationalSpeed, Forces[2]])
#         DF.append([RotationalSpeed, Forces[3]])
#         EF.append([RotationalSpeed, ExperimentalRigidForce[int((RotationalSpeed - 2000)/100)]])
#         err = ((Forces[2] - ExperimentalRigidForce[int((RotationalSpeed - 2000)/100)])/ExperimentalRigidForce[int((RotationalSpeed - 2000)/100)]*100)
#         errors.append([RotationalSpeed, err])
#     # PlotSpline(NF, "blue")
#     # [PlotScatter(NFi) for NFi in NF]
#     # PlotSpline(TF, "red")
#     # [PlotScatter(TFi) for TFi in TF]
#     PlotSpline(LF, "blue")
#     [PlotScatter(LFi) for LFi in LF]
#     # PlotSpline(DF, "black")
#     # [PlotScatter(DFi) for DFi in DF]
#     PlotSpline(EF, "red")
#     [PlotScatter(DFi) for DFi in EF]
#     PlotSpline(errors, "black")
#     [PlotScatter(DFi) for DFi in errors]
#     return

# def PreLoadData():
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/AirfoilTwistAngle.pickle', 'rb') as handle:
#             AirfoilTwistAngle = pickle.load(handle)
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/WingdS.pickle', 'rb') as handle:
#             WingdS = pickle.load(handle)
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/AirfoilAerodynamicCoefficientDickinson.pickle', 'rb') as handle:
#             AirfoilAerodynamicCoefficientDickinson = pickle.load(handle)
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/WingCenterOfMass.pickle', 'rb') as handle:
#             WingCenterOfMass = pickle.load(handle)
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/AirfoilCamberLineU.pickle', 'rb') as handle:
#             AirfoilCamberLineU = pickle.load(handle)
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/AirfoilCamberLineF.pickle', 'rb') as handle:
#             AirfoilCamberLineF = pickle.load(handle)
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/NodusYoungModulus.pickle', 'rb') as handle:
#             NodusYoungModulus = pickle.load(handle)
#     with open('./output/abitrary_propeller_aerodynamic_model/rigid_propeller/visualization/NodusShearModulus.pickle', 'rb') as handle:
#             NodusShearModulus = pickle.load(handle)

#     data = [AirfoilTwistAngle, WingdS, AirfoilAerodynamicCoefficientDickinson, WingCenterOfMass, AirfoilCamberLineU,
#              AirfoilCamberLineF, NodusYoungModulus, NodusShearModulus]

#     return data

