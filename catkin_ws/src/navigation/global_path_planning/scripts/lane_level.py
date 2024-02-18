#!/usr/bin/env python
import xml.etree.ElementTree as ET
import numpy as np
from matplotlib import pyplot as plt
from path_planning import GetClothoidPath, norm_ang, generate_maneuver
import math
from road_network import RoadNetwork

from scipy import interpolate

"""
LaneLevel class:

Class that handle lane-level layer given by OpenDrive xml file
"""
class LaneLevel(object):
    def __init__(self, root, road, ds=1.0):
        """
        Contructor of the class
            Input(s):
            (1) root - ElementTree object that handle the root of the xml file.
            (2) road - ElementTree object that handles a road in OpenDrive format.

            Author(s):
            (1) Júnior A. R. Da Silva

            Copyright owned by Universidade de São Paulo
        """
        self.root = root
        self.road = road
        self.ds =  ds

        #Compute road geometry
        initPar_list, sharp_length_list = self.get_road_geometry_parameters()
        self.reference_line = self.compute_road_reference_line( initPar_list, sharp_length_list )
        self.offset_curve = self.compute_offSet_curve()

    def get_curvature_arclength( self, path ):
        """
        Calculate curvature and arclength based on point locations

           Input(s):
           (1) path - X and Y position of points, where each line is a (X, Y) pair.

           Output(s):
           (1) kappa     - Column vector with curvatures for each point;
           (2) arclength - Column vector with arclength from first point to each point.

           Author(s):
           (1) Júnior A. R. Da Silva
           (2) Carlos M. Massera

           Copyright owned by Universidade de São Paulo
        """
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
        kappa_1 = np.array([1, -magnitude[0]]).dot(np.linalg.lstsq(A,b, rcond=None)[0])

        #     Final: Solve for kappa and dkappa using the two last available points
        A = ([1, -magnitude[len(magnitude)-2]],\
             [1, 0])
        b = kappa[len(kappa)-2:len(kappa)]
        kappa_end = np.array([1, magnitude[len(magnitude)-1]]).dot( np.linalg.lstsq(A,b, rcond=None)[0])

        #     Concatenate them into one vector
        kappa = np.concatenate(( ([kappa_1]), kappa, ([kappa_end]) ))

        return arclength, kappa

    def get_road_geometry_parameters(self):
        """
        Calculate curve parameters to be used in the generation of the road
        geometry curve (see GetClothoidPath in path_planning.py).

           Input(s):

           Output(s):
           (1) initPar_list   - List of unidimensional numpy
                arrays: [x0, y0, theta0, kappa0] that represent the initial
                paremeters of each curve.
           (2) sharp_length_list   - List of numpy arrays nx2, where n is the number
                of segments. The first column has all curvature derivative
                (sharpness) and the second one has all segments length.

           Author(s):
           (1) Júnior A. R. Da Silva

           Copyright owned by Universidade de São Paulo
        """

        initPar_list, sharp_length_list = [], []
        for geometry in self.road.iter('geometry'):
            s = float(geometry.attrib['s'])
            x = float(geometry.attrib['x'])
            y = float(geometry.attrib['y'])
            # if s == 0:
            #     plt.plot(x, y, 'ro')

            hdg = float(geometry.attrib['hdg'])
            length = float(geometry.attrib['length'])
            for primitive_curve in geometry:
                if primitive_curve.tag == 'line':
                    curvStart = 0.
                    sharpness = 0.
                elif primitive_curve.tag == 'arc':
                    curvStart = float(primitive_curve.attrib['curvature'])
                    sharpness = 0.
                elif primitive_curve.tag == 'spiral':
                    curvStart = float(primitive_curve.attrib['curvStart'])
                    curvEnd = float(primitive_curve.attrib['curvEnd'])
                    sharpness = (curvStart - curvEnd)/length

                initPar = np.array([x, y, hdg, curvStart])
                sharp_length = np.array([[sharpness, length]])
                initPar_list.append(initPar)
                sharp_length_list.append(sharp_length)

        return initPar_list, sharp_length_list

    def compute_road_reference_line( self, initPar_list, sharp_length_list ):
        """
        Calculate the road reference line.

           Input(s):
           (1) initPar_list   - List of unidimensional numpy
                arrays: [x0, y0, theta0, kappa0] that represent the initial
                paremeters of each curve.
           (2) sharp_length_list   - List of numpy arrays nx2, where n is the number
                of segments. The first column has all curvature derivative
                (sharpness) and the second one has all segments length.

           Output(s):
           (1) reference_line - Dictionary containing the reference line
           attributes: 'x' and 'y' defines the global localization of each
           point and 'hdg' gives their headings.

           Author(s):
           (1) Júnior A. R. Da Silva

           Copyright owned by Universidade de São Paulo
        """
        line_x = np.array([])
        line_y = np.array([])
        line_hdg = np.array([])
        for initPar, sharp_length in zip(initPar_list, sharp_length_list):
            path = GetClothoidPath(initPar, sharp_length, self.ds)
            line_x = np.concatenate((line_x, path[0]))
            line_y = np.concatenate((line_y, path[1]))
            line_hdg = np.concatenate((line_hdg, path[2]))
            # plt.plot(path[0], path[1], 'b.-')

        reference_line = {}
        reference_line['x'] = line_x
        reference_line['y'] = line_y
        reference_line['hdg'] = line_hdg

        return reference_line

    def compute_offSet_curve( self ):
        """
        Calculate the offset curve of the road. This curve gives a
        displacement to be added to the reference line. It is computed with
        respect to s.

           Input(s):

           Output(s):
           (1) offset_curve - Numpy array with the same length as
           self.reference_line.

           Author(s):
           (1) Júnior A. R. Da Silva

           Copyright owned by Universidade de São Paulo
        """
        laneOffset_list = []
        for laneOffset in self.road.iter('laneOffset'):
            laneOffset_list.append(laneOffset)

        offset_curve = np.zeros(len(self.reference_line['x']))
        s_offset = 0
        i = 0

        for off_idx in range(len(laneOffset_list)-1):
            laneOffset = laneOffset_list[off_idx]
            a = float(laneOffset.attrib['a'])
            b = float(laneOffset.attrib['b'])
            c = float(laneOffset.attrib['c'])
            d = float(laneOffset.attrib['d'])

            laneOffset_next = laneOffset_list[off_idx+1]
            s_next = float(laneOffset_next.attrib['s'])
            while (s_offset < s_next) and (s_next > self.ds) and (i < len(offset_curve)):
                offset_curve[i] = a + b*s_offset + c*s_offset**2 + d*s_offset**3
                s_offset += self.ds
                i += 1

        laneOffset = laneOffset_list[-1]
        a = float(laneOffset.attrib['a'])
        b = float(laneOffset.attrib['b'])
        c = float(laneOffset.attrib['c'])
        d = float(laneOffset.attrib['d'])
        while i < len(offset_curve):
            offset_curve[i] = a + b*s_offset + c*s_offset**2 + d*s_offset**3
            s_offset += self.ds
            i += 1

        return offset_curve

    def compute_lane_width( self, lane ):
        """
        Calculate the lane width curve of the lane. It is computed with
        respect to s.

           Input(s):
           (1) lane - ElementTree object that handles a lane in OpenDrive format.

           Output(s):
           (1) width_curve - Numpy array with the same length as
           self.reference_line.

           Author(s):
           (1) Júnior A. R. Da Silva

           Copyright owned by Universidade de São Paulo
        """
        width_list = lane.findall('./width')

        width_curve = np.zeros(len(self.reference_line['x']))
        s_offset = 0
        i = 0

        for width_idx in range(len(width_list)-1):
            width = width_list[width_idx]
            a = float(width.attrib['a'])
            b = float(width.attrib['b'])
            c = float(width.attrib['c'])
            d = float(width.attrib['d'])

            width_next = width_list[width_idx+1]
            s_next = float(width_next.attrib['sOffset'])
            while (s_offset < s_next) and (s_next > self.ds) and (i < len(width_curve)):
                width_curve[i] = a + b*s_offset + c*s_offset**2 + d*s_offset**3
                s_offset += self.ds
                i += 1

        width = width_list[-1]
        a = float(width.attrib['a'])
        b = float(width.attrib['b'])
        c = float(width.attrib['c'])
        d = float(width.attrib['d'])
        while i < len(width_curve):
            width_curve[i] = a + b*s_offset + c*s_offset**2 + d*s_offset**3
            s_offset += self.ds
            i += 1

        return width_curve


    def compute_lanes_path(self):
        """
        Calculate the geometry of all driving lanes in the road.

           Input(s):

           Output(s):
           (1) lanes_geometry_list - List of dictionaries containing the
           information of the geometry of each lane:
           - 'id': lane id
           - 'x' and 'y': global position of each point
           - 'hdg': heading on each point.
           - 'arclength': travel distance from the start of the lane.
           - 'curvature': curvature on each point.
           - 'travel_dir': travel direction (foward or backward) with respect to
           the road reference line.

           Author(s):
           (1) Júnior A. R. Da Silva

           Copyright owned by Universidade de São Paulo
        """

        ds = self.ds
        lanes_geometry_list = []
        s_laneSection = []
        for laneSection in self.road.iter('laneSection'):
            s_laneSection.append(float(laneSection.attrib['s']))
        s_laneSection.append(float(self.road.attrib['length']))

        lanes = self.road.find("./lanes")
        lane_sections = lanes.findall("laneSection")
        number_of_lane_sections = len(lane_sections)
        lane_section_init = lane_sections[0]
        lanes_for_driving_init = []
        for lane in lane_section_init.iter("lane"):
            if lane.attrib["type"] == "driving":
                lanes_for_driving_init.append(lane.attrib["id"])

        for lane_init in lanes_for_driving_init:
            lane_id = lane_init
            lane_path_x = []
            lane_path_y = []

            for lane_section, k in zip(lane_sections, range(number_of_lane_sections)):
                cum_width = 0
                id_list = []
                for lane in lane_section.iter('lane'):
                    id_list.append(int(lane.attrib["id"]))

                center_idx = id_list.index(0)
                id_left = np.flip(id_list[0:center_idx+1])
                id_right = id_list[center_idx:]

                id_list = np.concatenate((id_left, id_right))
                s_init = s_laneSection[k]
                s_end = s_laneSection[k+1]

                for idx in id_list:
                    lane = lane_section.find(".//lane[@id=\'" + str(idx) + "\']")

                    if idx == 0:
                        cum_width = 0
                        width = 0
                    else:
                        width = self.compute_lane_width(lane)
                        travel_dir = lane.find(".//vectorLane").attrib["travelDir"]

                    if lane.attrib["id"] == lane_id:
                        path_x = self.reference_line['x'] + (self.offset_curve + np.sign(idx)*(cum_width + width/2)) * np.cos(self.reference_line['hdg'] + np.pi/2)
                        path_y = self.reference_line['y'] + (self.offset_curve + np.sign(idx)*(cum_width + width/2)) * np.sin(self.reference_line['hdg'] + np.pi/2)

                        idx_init = int(np.round(s_init/ds))
                        idx_end = int(np.round(s_end/ds))
                        path_x = path_x[idx_init:idx_end]
                        path_y = path_y[idx_init:idx_end]

                        lane_path_x = np.concatenate((lane_path_x, path_x))
                        lane_path_y = np.concatenate((lane_path_y, path_y))
                        current_travel_dir = travel_dir
                        # print(travel_dir)
                    cum_width += width

                if k < (number_of_lane_sections-1):
                    lane = lane_section.find(".//lane[@id=\'" + lane_id + "\']")
                    if lane is None:
                    	continue
                    try:
                    	lane_id = lane.find(".//successor").attrib["id"]
                    except:
                    	print(self.road.attrib["id"], lane_id, lane is None, type(lane))
			

            if current_travel_dir == 'backward':
                lane_path_x = np.flip(lane_path_x)
                lane_path_y = np.flip(lane_path_y)

            #TODO: check if it is necessary
            # if len(lane_path_x) == 3:
            #     interp_x = interpolate.interp1d(np.arange(0,3), lane_path_x, kind='linear', fill_value="extrapolate")
            #     interp_y = interpolate.interp1d(np.arange(0,3), lane_path_y, kind='linear', fill_value="extrapolate")

            #     lane_path_x = interp_x(np.arange(0, 2, self.ds))
            #     lane_path_y = interp_y(np.arange(0, 2, self.ds))
                
            # path = np.concatenate((lane_path_x, lane_path_x), axis=1)
            if len(lane_path_x) > 3:
                path = np.zeros((len(lane_path_x), 2))
                heading = np.zeros((len(lane_path_x)))
                path[:,0] = lane_path_x
                path[:,1] = lane_path_y

                #Compute heading
                for p in range(len(heading)-1):
                    heading[p] = np.arctan2(path[p+1,1]-path[p,1], path[p+1,0]-path[p,0])
                    heading[p] = norm_ang(heading[p])
                heading[-1] = heading[-2]
                #Compute curvature and arclength
                arclength, curvature = self.get_curvature_arclength(path)
                lane_geometry_dic = {}
                lane_geometry_dic["id"] = lane_init
                lane_geometry_dic["x"] = lane_path_x
                lane_geometry_dic["y"] = lane_path_y
                lane_geometry_dic["hdg"] = heading
                lane_geometry_dic["arclength"] = arclength
                lane_geometry_dic["curvature"] = curvature
                lane_geometry_dic["travel_dir"] = current_travel_dir
                # print(current_travel_dir)
                # print()
                lanes_geometry_list.append(lane_geometry_dic)

        return lanes_geometry_list

    def get_signs_global_location( self, s, t ):
        """
        Calculate the global location of a signal or object.

           Input(s):
           (1) s - float
           (2) t - float

           Output(s):
           (1) x, y - float points given the global position.

           Author(s):
           (1) Júnior A. R. Da Silva

           Copyright owned by Universidade de São Paulo
        """
        s_idx = min(int(s/self.ds), len(self.reference_line['x']) - 1)
        hdg = self.reference_line['hdg'][s_idx]
        x = self.reference_line['x'][s_idx] + t*np.cos(hdg + np.pi/2)
        y = self.reference_line['y'][s_idx] + t*np.sin(hdg + np.pi/2)
        return x, y

    def get_traffic_lights(self):
        """
        Calculate the global location and the s coordinate of each traffic light
        of the road.

           Input(s):

           Output(s):
           (1) traffic_lights - List of dictionaries containing the information
           of each traffic light:
           's'          - s coordinate
           'x' and 'y'  - global coordinates
           'orientation'- gives the orientation in which the traffic light takes
           effect. '+' means that the traffic light is valid in the same direction
           as the reference line. '-' gives the opposite.

           Author(s):
           (1) Júnior A. R. Da Silva

           Copyright owned by Universidade de São Paulo
        """
        traffic_lights = []
        for signals in self.road.iter('signals'):
            for signal in signals:
                s = float(signal.get("s"))
                if signal.tag == "signal":
                    t = float(signal.get("t"))
                else:
                    signal_reference = self.root.find(".//signal[@id=\'" + signal.get("id") + "\']")
                    # s = float(signal_reference.get("s"))
                    t = float(signal_reference.get("t"))

                traffic_light_dic = {}
                traffic_light_dic["s"] = s
                traffic_light_dic["orientation"] = signal.get("orientation")
                traffic_light_dic["x"], traffic_light_dic["y"] = self.get_signs_global_location( s, t )
                traffic_lights.append(traffic_light_dic)
        return traffic_lights

    def get_traffic_sings(self):
        """
        Calculate the global location and the s coordinate of each object
        of the road.

           Input(s):

           Output(s):
           (1) traffic_signs - List of dictionaries containing the information
           of each object:
           's'          - s coordinate
           'x' and 'y'  - global coordinates
           'orientation'- gives the orientation in which the traffic sign takes
           effect. '+' means that the traffic light is valid in the same direction
           as the reference line. '-' gives the opposite.
           'type'       - the object type. It can be a speed limit sign, a
           crosswalk, a slice stop or a stop line.

           Author(s):
           (1) Júnior A. R. Da Silva

           Copyright owned by Universidade de São Paulo
        """
        traffic_signs = []
        for objects in self.road.iter('objects'):
            for object in objects.iter('object'):
                object_name = object.get("name")
                s = float(object.get("s"))
                t = float(object.get("t"))
                traffic_sign = {}
                traffic_sign["s"] = s
                traffic_sign["name"] = object.get("name")
                traffic_sign["type"] = object.get("type")
                traffic_sign["orientation"] = object.get("orientation")
                traffic_sign["x"], traffic_sign["y"] = self.get_signs_global_location( s, t )
                traffic_signs.append(traffic_sign)
        return traffic_signs

    def compute_object_lane_attr( self, object, lane_geom, type="traffic_light"):
        """
        Calculate the global location and the s coordinate of each object
        of the road.

           Input(s)
           (1) object - dictionary containing the information of the object:
           's'          - s coordinate on the road
           'x' and 'y'  - global coordinates
           'orientation'- gives the orientation in which the traffic sign takes
           effect. '+' means that the traffic light is valid in the same direction
           as the reference line. '-' gives the opposite.
           'type'           - the object type. It can be a speed limit sign, a
           crosswalk, a slice stop or a stop line.
           (2) lane_geom    - dictionary with the geometry of the lane.
           (3) type         - the type of the object.


           Output(s):
           (1) object_lane:
           's'          - s coordinate on the lane
           'x' and 'y'  - global coordinates of the object
           'type'       - the object type. It can be a speed limit sign, a
           crosswalk, a slice stop or a stop line.

           Author(s):
           (1) Júnior A. R. Da Silva

           Copyright owned by Universidade de São Paulo
        """

        x_obj = object['x']
        y_obj = object['y']
        x = lane_geom["x"]
        y = lane_geom["y"]
        arclength = lane_geom["arclength"]
        all_dist = np.sqrt((x - x_obj)**2 + (y - y_obj)**2)
        idx = np.argmin(all_dist)
        object_lane = {}
        object_lane["s"] = arclength[idx]
        object_lane["x"] = x_obj
        object_lane["y"] = y_obj
        object_lane["type"] = type
        return object_lane

    def get_road_attributes(self):
        """
        Calculate full attribues of the road required for navigation.

           Input(s)
           (1) object - dictionary containing the information of the object:
           's'          - s coordinate on the road
           'x' and 'y'  - global coordinates
           'orientation'- gives the orientation in which the traffic sign takes
           effect. '+' means that the traffic light is valid in the same direction
           as the reference line. '-' gives the opposite.
           'type'           - the object type. It can be a speed limit sign, a
           crosswalk, a slice stop or a stop line.
           (2) lane_geom    - dictionary with the geometry of the lane.
           (3) type         - the type of the object.


           Output(s):
           (1) road_attr - dictionary containing 'forward' and 'backward' lanes.
           Each lane is represent by a lane_attr dictionary, where:
           'geometry'       - gives the lane attrib
           'traffic_light'  - list of all traffic lights at the lane.
           'speed_limit'    - list of all speed limits at the lane.
           'crosswalk'      - list of all crosswalks on the lane.
           'stencil_stop'   - list of all stencil stops on the lane.
           'stop_line'      - list of all stop lines on the lane.
           'others'         - list of all others objects.

           Author(s):
           (1) Júnior A. R. Da Silva

           Copyright owned by Universidade de São Paulo
        """

        lanes_forward = []
        lanes_backward = []
        lane_geom_list = self.compute_lanes_path()
        traffic_light_list = self.get_traffic_lights()
        traffic_sign_list = self.get_traffic_sings()
        for lane_geom in lane_geom_list:
            lane_id = int(lane_geom["id"])
            traffic_light_lane_list = []
            speed_limit_lane_list = []
            stop_line_lane_list = []
            stencil_stop_lane_list = []
            crosswalk_lane_list = []
            other_lane_list = []
            for traffic_light in traffic_light_list:
                orientation = traffic_light["orientation"]
                if (orientation == "+" and lane_id > 0) or (orientation == "-" and lane_id < 0):
                    traffic_light_lane = self.compute_object_lane_attr( traffic_light, lane_geom, type="traffic_light")
                    traffic_light_lane_list.append(traffic_light_lane)
            for traffic_sign in traffic_sign_list:
                orientation = traffic_sign["orientation"]
                if (orientation == "+" and lane_id > 0) or (orientation == "-" and lane_id < 0):
                    if traffic_sign["type"] == "crosswalk":
                        type = "crosswalk"
                    else:
                        type = traffic_sign["name"]
                    traffic_sign_lane = self.compute_object_lane_attr( traffic_sign, lane_geom, type)

                    if type[0:5] == "Speed":
                        speed_limit_lane_list.append(traffic_sign_lane)
                    elif type == "crosswalk":
                        crosswalk_lane_list.append(traffic_sign_lane)
                    elif type == "Stencil_STOP":
                        x = traffic_sign_lane['x']
                        y = traffic_sign_lane['y']
                        x_lane = lane_geom['x']
                        y_lane = lane_geom['y']
                        min_dist = min(np.sqrt((x-x_lane)**2 + (y-y_lane)**2))
                        if min_dist < 0.3:
                            stencil_stop_lane_list.append(traffic_sign_lane)
                    elif type == "StopLine":
                        stop_line_lane_list.append(traffic_sign_lane)
                    else:
                        other_lane_list.append(traffic_sign_lane)

            lane_attr = {}
            lane_attr["geometry"] = lane_geom
            lane_attr["id"] = lane_id
            lane_attr["traffic_light"] = traffic_light_lane_list
            lane_attr["speed_limit"] = speed_limit_lane_list
            lane_attr["crosswalk"] = crosswalk_lane_list
            lane_attr["stencil_stop"] = stencil_stop_lane_list
            lane_attr["stop_line"] = stop_line_lane_list
            lane_attr["others"] = other_lane_list

                         
            if lane_geom["travel_dir"] == "forward":
                lanes_forward.append(lane_attr)
            else:
                lanes_backward.append(lane_attr)

        # if self.road.attrib['id'] == '51':
        #     print('LN:', len(lanes_forward), len(lanes_backward), len(lane_geom_list))
        road_attr = {}
        road_attr["forward"] = lanes_forward
        road_attr["backward"] = lanes_backward
        return road_attr

