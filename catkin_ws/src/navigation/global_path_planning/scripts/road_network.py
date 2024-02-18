#!/usr/bin/env python
import xml.etree.ElementTree as ET
import numpy as np

"""
RoadNetwork class:

Class that handle a road newtork given by OpenDrive xml file
"""
class RoadNetwork(object):
    def __init__(self, root):
        self.root = root

    def get_next_roads( self, current_road_id, travel_dir ):
        """
        Method to create the road network

        """

        road = self.root.find("./road[@id=\'" + current_road_id + "\']")
        next_roads = []

        for link in road:
            if link.tag == "link":
                for child in link:
                    if child.attrib["elementType"] == "road":
                        if child.tag == "predecessor" and travel_dir == "backward":
                            if child.attrib["contactPoint"] == "start":
                                next_roads.append([child.attrib["elementId"], "forward"])
                            elif child.attrib["contactPoint"] == "end":
                                next_roads.append([child.attrib["elementId"], "backward"])

                        elif child.tag == "successor" and travel_dir == "forward":
                            if child.attrib["contactPoint"] == "start":
                                next_roads.append([child.attrib["elementId"], "forward"])
                            elif child.attrib["contactPoint"] == "end":
                                next_roads.append([child.attrib["elementId"], "backward"])

                    elif child.attrib["elementType"] == "junction":
                        junction = self.root.find("./junction[@id=\'" + str(child.attrib["elementId"]) + "\']")
                        for connection in junction.iter('connection'):
                            if connection.attrib['incomingRoad'] == road.attrib["id"]:
                                for laneLink in connection.iter('laneLink'):
                                    if self.is_driving(connection.attrib["connectingRoad"], laneLink.attrib['to']):

                                        if child.tag == "predecessor" and travel_dir == "backward":
                                            if connection.attrib["contactPoint"] == "start":
                                                next_roads.append([connection.attrib["connectingRoad"], "forward"])
                                            elif connection.attrib["contactPoint"] == "end":
                                                next_roads.append([connection.attrib["connectingRoad"], "backward"])

                                        elif child.tag == "successor" and travel_dir == "forward":
                                            if connection.attrib["contactPoint"] == "start":
                                                next_roads.append([connection.attrib["connectingRoad"], "forward"])
                                            elif connection.attrib["contactPoint"] == "end":
                                                next_roads.append([connection.attrib["connectingRoad"], "backward"])

        if len(next_roads)>0:
            return np.unique(next_roads,  axis=0)
        else:
            return []

    def get_random_next_road(self, next_roads):

        rand_idx = 0#np.random.choice(range(len(next_roads)))
        next_road_id, travel_dir = next_roads[rand_idx]
        return next_road_id, travel_dir

    def get_lanes(self, current_road_id, next_road_id, travel_dir):
        """
        According to traffic rules and connection types, the vehcile must be in
        predefine lanes before the next road.

        param: current_road_id
        type: string
        param: next_road_id
        type: s

        return: lanes_id
        type: list of strings
        """
        road = self.root.find("./road[@id=\'" + current_road_id + "\']")
        lanes_id = []
        # print(current_road_id, next_road_id)
        for link in road:
            if link.tag == "link":
                for child in link:
                    if child.attrib["elementType"] == "road" and child.attrib["elementId"] == next_road_id:
                        for lanes in road.iter('lanes'):
                            for lane in lanes.iter('lane'):
                                vectorLane = lane.find(".//vectorLane")
                                if vectorLane is not None and vectorLane.attrib["travelDir"] == travel_dir:
                                    for lane_link in lane.iter("link"):
                                        for lane_link_child in lane_link:
                                            if lane_link_child.tag == child.tag and self.is_driving(next_road_id, lane_link_child.attrib["id"]):
                                                lanes_id.append(lane_link_child.attrib["id"])


                    elif child.attrib["elementType"] == "junction":
                        junction = self.root.find("./junction[@id=\'" + str(child.attrib["elementId"]) + "\']")
                        for connection in junction.iter('connection'):
                            if connection.attrib['incomingRoad'] == current_road_id and connection.attrib['connectingRoad'] == next_road_id:
                                for laneLink in connection.iter('laneLink'):
                                    if self.is_driving(connection.attrib["connectingRoad"], laneLink.attrib['to']):
                                        lanes_id.append(laneLink.attrib['from'])

        return lanes_id


    def find_inital_road_and_lane( self, road_id ):
        """
        Start point of the road network graph. It's required that the initial
        lane to be a 'driving' lane.

        return:
        param: initial road, lane
        type: string
        """

        #Define the initial road as the road "0"
        initial_road = self.root.find("./road[@id=\'" + road_id + "\']")

        #Find a valid lane whose "type" attribute is "driving"
        for lanes in initial_road.iter('lanes'):
            for laneSection in lanes.iter('laneSection'):
                for child in laneSection:
                    for lane in child:
                        type = lane.attrib['type']
                        vectorLane = lane.find(".//vectorLane")
                        if type == "driving":
                            return road_id, vectorLane.attrib['travelDir']

    def is_driving(self, road_id, lane_id):
        """
        Evaluate if the given lane is a "driving" lane

        return:
        param: _
        type: bool
        """
        road = self.root.find("./road[@id=\'" + road_id  + "\']")
        lane = road.find(".//lane[@id=\'" + lane_id  + "\']")

        if lane.attrib["type"] == "driving":
            return True
        else:
            return False