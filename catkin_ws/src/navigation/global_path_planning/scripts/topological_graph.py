import numpy as np
import os
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element
 
from road_network import RoadNetwork
from lane_level import LaneLevel
from path_planning import generate_maneuver, GetClothoidPath
from typing import Dict, NoReturn, Tuple, List, Union
from networkx import MultiDiGraph
import networkx as nx

from scipy import interpolate
from scipy.signal import savgol_filter

from joblib import Parallel, delayed

class TopologicalMap(MultiDiGraph):
    
    def __init__(self, verbose:bool=False,  **kwargs):
        
        super(TopologicalMap, self).__init__(**kwargs)
        
        self.waypoints = None
        self.verbose= verbose
            
        self.map = None
        self.road_network = None
        
        self.built=False
        
    def set_map_path(self, path:str):
        if not os.path.exists(path):
            raise FileNotFoundError(f"file not found: ({path})")
        
        self.name = path.split("/")[-1].split(".")[0]
        tree = ET.parse(path)
        self.map = tree.getroot()
        self.road_network = RoadNetwork(self.map)
    
    def set_map_content(self, content:str):
        if not content:
            raise RuntimeError("You must give the xml content of the OpenDrive file (.xodr)")
        
        tree = ET.ElementTree(ET.fromstring(content))
        self.map = tree.getroot()
        self.road_network = RoadNetwork(self.map)
        
        if  self.map.find("header").attrib["name"]:
            self.name =  self.map.find("header").attrib["name"]
        else:
            self.name = "hd-map"
    
    def is_active(self):
        return self.map is not None
    
    def build_graph_from_waypoints(self, 
                                   waypoints:np.ndarray,
                                   return_waypoints:bool=False,
                                   look_ahead:int=0
    ) -> Union[NoReturn, np.ndarray]:
        
        if self.map is None:
            raise RuntimeError("You must first give the map content using set_map_path or set_map_content!")
        
        self.waypoints = waypoints
        
        __nodes = []
        loc_nodes = []
        edges = []
        nodes = []
        
        visited_nodes = []
        
        __nodes = self.get_nodes_from_waypoints(waypoints=waypoints, 
                                                look_ahead=look_ahead)
        
        if return_waypoints:
            loc_nodes = np.copy(__nodes)  
        
        while len(__nodes) > 0:
            road_id, lane_id, travel_dir, wox, woy = __nodes.pop()
            if road_id in visited_nodes:
                continue
            if  self.has_node(road_id):
                continue
            
            visited_nodes.append(road_id)
            nodes.append([road_id, lane_id, travel_dir, wox, woy])
            
            #self loop
            edges.append([road_id, road_id, 0])
            self.add_node(road_id, 
                          position=[float(wox), float(woy)],
                          road=self.get_road_from_id(road_id))
            
            #neighbors
            next_roads = self.road_network.get_next_roads(road_id, "forward")
            
            for nr in next_roads:
                
                lane_id, lane_way = self.__find_nearest_lane(nr[0], nr[1], wox, woy)
                
                if lane_way is None:
                    continue
                
                __nodes.append([nr[0], lane_id, nr[1], lane_way[0], lane_way[1]])
                
                edges.append([road_id, 
                              nr[0], 
                              round(self.__euclidean_distance([wox, woy], lane_way), 3)])
                
                
        
        #remove repeated
        edges = np.unique(edges, axis=0)
        
        self.add_weighted_edges_from(edges)
        
        self.built = True
        
        if return_waypoints:
            return loc_nodes  
        
    def build_graph_from_map(self)->NoReturn:
        
        if self.map is None:
            raise RuntimeError("You must first give the map content using set_map_path or set_map_content!")
        
        raise NotImplementedError("function not fully implemented!")
          
        def __process_road(idx):
            
            road = self.get_road_from_id(idx)
            wox = 0
            woy = 0
            
            
            self.add_node(idx,
                          position=[wox, woy],
                          road=road)
            
            edges = []
            next_nodes = self.road_network.get_next_roads(idx, "forward")
            
            for nr in next_nodes:
                lane_id, lane_way = self.__find_nearest_lane(nr[0], nr[1], wox, woy)
                
                edges.append([idx, 
                              nr[0], 
                              round(self.__euclidean_distance([wox, woy], lane_way), 3)])
            
            return edges

            
        
    def print_graph(self):
        g = {}
        for n in self.nodes:
            g[n] = [k for k in self.neighbors(n)]
        
        for i in range(0, np.amax([int(n) for n in self.nodes])):
            if str(i) in g:
                print(i, g[str(i)])    
            
    def __euclidean_distance(self, 
                             a:Union[List, np.ndarray], 
                             b:Union[List, np.ndarray]
    ) -> Union[float, np.ndarray]:        
        return np.sqrt(np.power(np.subtract(a,b), 2).sum(axis=0))
    
    def __find_nearest_lane(self, 
                          road_ido:str, 
                          travel_diro:str,
                          wox:float, 
                          woy:float
    )->Union[
        Tuple[int, List[Tuple[float, float]]],
        Tuple[None, None]
     ]:
        
        dist = 1e5
        lane_way = None
        lane_id = None
        flag = False
        
        road = self.get_road_from_id(road_ido)
        
        lane_level = LaneLevel(self.map, road, ds=0.2)
        road_attr = lane_level.get_road_attributes()
        lanes = road_attr[travel_diro]
        # if road_ido == '77':
        #     print(len(lanes))
        #     ll = road_attr['backward']
        #     print(len(ll))
        #     print(road_attr)
        #percorre todas as lanes dessa rua
        euclidean_dist = lambda l: np.sqrt(np.power(wox - l["x"], 2) +\
                                           np.power(woy - l["y"], 2))
        
        for lane in lanes:
            dist_temp = np.amin(euclidean_dist(lane["geometry"]))
                           
            if dist_temp < dist:
                idx = np.argmin(dist)
                dist = dist_temp
                lane_id = lane["id"]
                lane_way = [lane["geometry"]["x"][idx], 
                            lane["geometry"]["y"][idx]]
                    
            
        return lane_id, lane_way
        
    def find_road_and_lane(self, 
                           x:float, 
                           y:float
    ) -> Union[
            Tuple[str, str, str, Dict], #found road
            Tuple[None, None, None, None]#otherwise
        ]:
        min_dist = np.inf
        road_id = None
        lane_id = None
        travel_dir = None
        lane_geometry = None
        
        
        for road in self.map.iter("road"):
            lane_level = LaneLevel(self.map, road, ds=0.1)
            road_attr = lane_level.get_road_attributes()

            for key in road_attr.keys():
                lanes = road_attr[key]
                for lane in lanes:
                    lane_geom = lane["geometry"]
                    x_geom = lane_geom["x"]
                    y_geom = lane_geom["y"]
                    dist = np.amin(np.sqrt( (x_geom - x)**2 + (y_geom - y)**2 ) )
                    
                    if dist < min_dist:
                        min_dist = dist
                        road_id = str(road.attrib["id"])
                        lane_id = str(lane_geom["id"])
                        travel_dir = str(lane_geom["travel_dir"])
                        lane_geometry = lane_geom
        
        return (road_id, lane_id, travel_dir, lane_geometry)
    
    
    def __get_nearest_lane(self, road, way, ways_ahead):
        lane_level = LaneLevel(self.map, road, ds=0.1)
        road_attr = lane_level.get_road_attributes()
        
        min_dist = np.inf
        lane_id = None
        travel_dir = None
        lane_geometry = None
        
        for key in road_attr.keys():
            lanes = road_attr[key]
                
            for lane in lanes:
                lane_geom = lane["geometry"]
                x_geom = lane_geom["x"]
                y_geom = lane_geom["y"]
                
                dist_func = lambda x, y: np.amin(np.sqrt((x_geom - x)**2 + (y_geom - y)**2))
                
                if len(ways_ahead)>0:
                    weights = [i for i in reversed(range(1, len(ways_ahead) + 1))]
                    dist_ahead = [dist_func(w[0], w[1])*i for i, w in zip(weights, ways_ahead)]
                else:
                    weights = [0]
                    dist_ahead = [0]
                    
                dist_way = dist_func(way[0], way[1])
                dist = np.sum(dist_ahead) + dist_way*len(weights)*2
                dist = dist/(np.sum(weights) + len(weights)*2)
                
                if dist < min_dist:
                    min_dist = dist
                    lane_id = str(lane_geom["id"])
                    travel_dir = str(lane_geom["travel_dir"])
                    lane_geometry = lane_geom
        
        return min_dist, lane_id, travel_dir, lane_geometry
    
    def __find_road_and_lane_with_lookahead(self, 
                                          idx:int,
                                          waypoints:np.ndarray, #x, y
                                          look_ahead:int=0
    ) -> Union[
            Tuple[str, str, str, Dict], #found road
            Tuple[None, None, None, None]#otherwise
        ]:
        
        way = waypoints[idx]
        ways_ahead =[]
                
        max_look_ahead = look_ahead + 1    
        
        road_id = None
        lane_id = None
        travel_dir = None
        lane_geometry = None
        
        min_dist = 1e5
        
        for road in self.map.iter("road"):
            dist = min_dist
            road_look_ahead = look_ahead - 1
            
            while (abs(min_dist - dist) < 1.0) and (road_look_ahead <= max_look_ahead):
                road_look_ahead =  road_look_ahead + 1
                
                ways_ahead = [waypoints[idx + w] 
                          for w in range(1, road_look_ahead+1)
                          if (idx + w) < len(waypoints)]
                
                dist, _lane_id, _travel_dir, _lane_geom =\
                    self.__get_nearest_lane(road, way, ways_ahead)
            
            # if (idx == 0) and (_lane_id is None):
            #     _d, _l, _t, _g = self.__get_nearest_lane(road, way, [])
            #     print(road.attrib['id'], _d, _l, _t)
                                  
                
            if dist < min_dist:
                min_dist = dist
                road_id = str(road.attrib["id"])
                lane_id = str(_lane_id)
                travel_dir = str(_travel_dir)
                lane_geometry = _lane_geom
        
        
        if min_dist > 3.:
            n = self.find_road_and_lane(way[0], way[1])
            road_id = n[0]
            lane_id = n[1]
            travel_dir = n[2]
            lane_geometry = n[3]
            
        return (road_id, lane_id, travel_dir, lane_geometry)
        
    
    def get_nodes_from_waypoints(self, 
                                 waypoints:np.ndarray,
                                 look_ahead:int=0
    ) -> List[Tuple[str, str, str, float, float]]:
        
        if self.map is None:
            raise RuntimeError("You must first give the map content using set_map_path or set_map_content!")
        
        
        nodes = []
        
        def __get_node_with_look_ahead(w):
            
            road_id, lane_id, travel_dir, _ =\
                    self.__find_road_and_lane_with_lookahead(idx=w, 
                                                             waypoints=waypoints,
                                                             look_ahead=look_ahead)
                
            way = waypoints[w]                
            
            
                
            return [road_id, lane_id, travel_dir, way[0], way[1]]
            
        nodes = Parallel(n_jobs=-2)(
                    delayed(__get_node_with_look_ahead)(w=w)
                        for w in  range(len(waypoints))
                )
        # for w in range(len(waypoints)):

        #     road_id, lane_id, travel_dir, _=\
        #         self.__find_road_and_lane_with_lookahead(idx=w, 
        #                                                  waypoints=waypoints,
        #                                                  look_ahead=look_ahead)
        #     way = waypoints[w]

        #     nodes.append([
        #         [road_id, lane_id, travel_dir, way[0], way[1]]
        #     ])

        return nodes
        
    def __find_route(self, node_o, node_g):
        path = []
        
        #There is no connection from road 16 to 170 in Town04
        #based on xodr: 0 -> 170 -> 16 (it looks like a wrong connection!)
        #TODO: check if xodr or the code is wrong
        try: 
            path = nx.shortest_path(G=self, 
                                    source=node_o[0], 
                                    target=node_g[0], 
                                    method="bellman-ford")
        except nx.NetworkXNoPath:
            if self.verbose:
                print(f"\033[91mPath not found: ({self.name})(source: {node_o[0]}, target:{node_g[0]})\033[0m")
            path = []
            
        return path
        
    def get_road_from_id(self, idx):
        if self.has_node(idx):
            return self.nodes[idx]["road"]
        else:
            for road in self.map.iter("road"):
                if road.attrib["id"] == idx:
                    return road
            return None
                
    def __get_likely_road_connections(self, 
                                      wo:str,
                                      wg:str,
                                      range_dist:float=5.
    ) -> Union[List[Element], None]:
        
        road_o = self.get_road_from_id(wo[0])
        road_g = self.get_road_from_id(wg[0])
        
        next_o = self.road_network.get_next_roads(road_o.attrib["id"], "forward")
        back_o = self.road_network.get_next_roads(road_o.attrib["id"], "backward")
        
        next_g = self.road_network.get_next_roads(road_g.attrib["id"], "forward")
        back_g = self.road_network.get_next_roads(road_g.attrib["id"], "backward")

        # print(np.shape(next_o), np.shape(next_g), np.shape(back_o), np.shape(back_g))
        # next_roads = np.concatenate([next_o, back_o, next_g, back_g], axis=0)
        
        next_roads = []
        for n in [next_o, back_o, next_g, back_g]:
            if len(n) > 0:
                next_roads.append(n)
        next_roads = np.concatenate(next_roads, axis=0)
        
        if (wo[0] in next_roads[:, 0]) or (wg[0] in next_roads[:, 0]):
            return None
        else:
            lane_o = self.__get_lanes_from_road_id(wo[0], wo[2])
            lane_o = [l for l in lane_o if l["id"]==int(wo[1])][0]
                
            lane_g = self.__get_lanes_from_road_id(wg[0], wg[2])
            lane_g = [l for l in lane_g if l["id"]==int(wg[1])][0]
        
            way_o = [lane_o["geometry"]["x"][-1], lane_o["geometry"]["y"][-1]]
            way_g = [lane_o["geometry"]["x"][0], lane_o["geometry"]["y"][0]]
            
            lane_id = None
            travel_dir = None
            road_id = None
            lane_geom = None
            
            min_dist = 1e5
            for nr in next_roads:
                road_nr = self.get_road_from_id(nr[0])            
                dist, l_id, t_dir, lane_g=\
                    self.__get_nearest_lane(road_nr, way_o, [way_g])  
                        
                if dist < min_dist:
                    min_dist=dist
                    lane_id = l_id 
                    travel_dir = t_dir
                    road_id = nr[0]
                    lane_geom=lane_g
           
            if not self.has_node(road_id):
                self.add_node(road_id,
                              position=[lane_geom["x"][0], lane_geom["y"][0]],
                              road=self.get_road_from_id(road_id))
                
                self.add_edge(wo[0], 
                              road_id, 
                              weight=self.__euclidean_distance(self.nodes[wo[0]]["position"],
                                                               [lane_geom["x"][0], lane_geom["y"][0]])
                              )
            elif not self.has_edge(wo[0], road_id):
                self.add_edge(wo[0],
                              road_id,
                              weight=self.__euclidean_distance(self.nodes[wo[0]]["position"],
                                                               [lane_geom["x"][0], lane_geom["y"][0]])
                              )
            
            return [road_id, lane_id, travel_dir, lane_geom["x"][0], lane_geom["y"][0]]
                    
        
            

    def get_route_from_waypoints(self, waypoints:np.ndarray, look_ahead:int=0, build_graph:bool=False):
        '''
        TODO:
            There is something wrong with the edges of the graph
            
            Steps
            1) check forward and backward definition
            2) check map (xodr)
            3) build a graph from the full map (not only the waypoints)
                if the map is worng, build the graph with full map wont solve the problem
        
        '''
        if (not self.built) or (build_graph):
            nodes = self.build_graph_from_waypoints(waypoints=waypoints, 
                                                    return_waypoints=True,
                                                    look_ahead=look_ahead)
        else:
            nodes = self.get_nodes_from_waypoints(waypoints=waypoints, 
                                                  look_ahead=look_ahead)        
        
        route = []
        
        for w in range(len(nodes)-1):
            wo = nodes[w]
            wg = nodes[w+1]
            
            #straight connection
            if self.has_edge(wo[0], wg[0]):
                route.append(wo)
            else:
                _sub_route = self.__find_route(wo, wg)    
                route.append(wo)
               
                #most likely not enter here
                #TODO: check if all connection in xodr is right!
                # if len(_sub_route) > 0: 
                #     del _sub_route[0]
                #     del _sub_route[-1]
                #     print(f"HAS EDGE: ({wo[0]}, {wg[0]})", end="")
                #     way = [wo[3], wo[4]]
                #     for sn in _sub_route:
                #         print(f"{sn[0]}, ")
                #         road = self.get_road_from_id(sn)
                #         _, lane_id, travel_dir, lane_geom =\
                #                 self.__get_nearest_lane(road, [float(way[0]), float(way[1])], [])
                        
                #         dist_lane = np.sqrt(np.power(lane_geom["x"] - float(way[0]), 2) +
                #                             np.power(lane_geom["y"] - float(way[1]), 2))
                #         idx_lane = np.argmin(dist_lane)
                        
                #         way = [lane_geom["x"][idx_lane],
                #                lane_geom["y"][idx_lane]]
                        
                #         route.append([sn, lane_id, travel_dir, way[0], way[1]])
                #     print("")
                # else:
                if self.verbose:
                    print(f"\033[93mtrying to find connection between roads : {wo[0]} and {wg[0]}!\033[0m")
                
                curr_road = wo
                while curr_road is not None:
                    curr_road = self.__get_likely_road_connections(
                                        curr_road,
                                        wg
                                )
                    if (curr_road is not None) and (not (curr_road[0] in [wo[0], wg[0]])):
                        route.append(curr_road)
                        
                        if self.verbose:
                            wo[3] = round(float(wo[3]), 3)
                            wo[4] = round(float(wo[4]), 3)
                            wg[3] = round(float(wg[3]), 3)
                            wg[4] = round(float(wg[4]), 3)
                            print((f"\033[92mfound new road: road_id:{curr_road[0]}, lane_id:{curr_road[1]}, travel_dir:{curr_road[2]},"
                                f" wo:{[wo[3], wo[4]]}, wm:{[curr_road[3], curr_road[4]]}, wg:{[wg[3], wg[4]]}\033[0m"))
                        
                    # wmo_x = (float(wo[3]) + float(wg[3]))/2.
                    # wmo_y = (float(wo[4]) + float(wg[4]))/2.
                    
                    # wmg_x = float(wg[3])
                    # wmg_y = float(wg[4])
                    
                    # mid_waypoints = np.array([[wmo_x, wmo_y], [wmg_x, wmg_y]])
                    # mid_node = self.get_nodes_from_waypoints(waypoints=mid_waypoints, 
                    #                                          look_ahead=1)[0]
                    
                    # route.append([mid_node[0], mid_node[1], mid_node[2], mid_node[3], mid_node[4]]) 
                    # print((f"\t\033[92m found: road_id:{mid_node[0]}, lane_id:{mid_node[1]}, travel_dir:{mid_node[2]},"
                    #        f" wo:{[wo[3], wo[4]]}, wm:{[mid_node[3], mid_node[4]]}, wg:{[wg[3], wg[4]]}\033[0m"))
              
        route.append(nodes[-1])             
        return route
        
        
    def __get_lane_change(self, 
                          wo:List[Tuple[float, float]], 
                          wg:List[Tuple[float, float]], 
                          lane_o:Dict, 
                          lane_g:Dict, 
                          x_way:List[float], 
                          y_way:List[float], 
                          early_dist:float=15.
    ):
        idx_o = np.argmin(np.sqrt((lane_o["x"] - wo[0])**2+\
                                (lane_o["y"] - wo[1])**2))
                    
        idx_g = np.argmin(np.sqrt((lane_g["x"] - wg[0])**2+\
                                (lane_g["y"] - wg[1])**2)) 
        
        #TODO: check this (it should stated the maneuver early)                
        #end of maneuver
        (xg, yg, tg, kg) = \
            (lane_g['x'][idx_g],
             lane_g['y'][idx_g],
             lane_g['hdg'][idx_g],
             lane_g['curvature'][idx_g])
            
        #start of maneuver
        aux_dist = np.abs(np.sqrt(np.power(x_way - xg, 2) + np.power(y_way - yg, 2)) - early_dist)
        start_idx = np.argmin(aux_dist)
        len_path = len(aux_dist) - start_idx
        
        (xo, yo, to, ko) = \
            (x_way[start_idx],
            y_way[start_idx],
            lane_o['hdg'][idx_o],
            lane_o['curvature'][idx_o])
            
        lc_init, lc_sharp_len= generate_maneuver(xo, yo,to, ko, xg, yg, tg, kg)
        lc = GetClothoidPath(lc_init, lc_sharp_len, ds=0.1)
        
        return lc[0], lc[1], [start_idx, len_path]


    def __get_lane_keeping(self, 
                           wo:List[Tuple[float, float]], 
                           wg:List[Tuple[float, float]], 
                           lane:Dict
    )->Tuple[np.ndarray, np.ndarray]:
        idx_o = np.argmin(np.sqrt((lane["x"] - wo[0])**2+\
                                (lane["y"] - wo[1])**2))
        idx_g = np.argmin(np.sqrt((lane["x"] - wg[0])**2+\
                                (lane["y"] - wg[1])**2)) 
        
        idx_g = len(lane["x"])\
                if np.isclose(idx_g, len(lane["x"]), atol=2)\
                else idx_g

        return lane["x"][idx_o:idx_g], lane["y"][idx_o:idx_g]

    def __get_lanes_from_road_id(self, road_id, travel_dir):
        road = self.get_road_from_id(road_id)
        
        lane_level = LaneLevel(self.map, road, ds=0.1)
        road_attr = lane_level.get_road_attributes()

        return road_attr[travel_dir]
        
        
    def get_path_from_nodes_list(self, nodes:np.ndarray):
            
        x_way = []
        y_way = []
        
        for w in range(0, len(nodes) -1):
            
            #get nodes
            road_ido, lane_ido, travel_diro, wox, woy = nodes[w]
            road_idg, lane_idg, travel_dirg, wgx, wgy = nodes[w + 1]
            
            wox, woy = float(wox), float(woy)
            wgx, wgy = float(wgx), float(wgy)
            
            #get lanes object
            lanes_o = self.__get_lanes_from_road_id(road_ido, travel_diro)
            lanes_g = self.__get_lanes_from_road_id(road_idg, travel_dirg)
            
            lanes_o = [l["geometry"] for l in lanes_o if l["id"] == int(lane_ido)][0]
            lanes_g = [l["geometry"] for l in lanes_g if l["id"] == int(lane_idg)][0]
            
            # print(f"({road_ido}, {lane_ido}) => ({road_idg}, {lane_idg}) => ", end="")
            
            #same road
            if  road_ido == road_idg:
                #lane-keeping
                if lane_ido == lane_idg: 
                    x_lk, y_lk = self.__get_lane_keeping(wo=[wox, woy], 
                                                         wg=[wgx, wgy], 
                                                         lane=lanes_o)
                    if len(x_lk) > 0:
                        x_way.append(x_lk)
                        y_way.append(y_lk)
                        
                    print('x_lk, y_lk',x_lk, y_lk)                 
                #lane change
                else: 
                    x_lc, y_lc, indexes = self.__get_lane_change(wo=[wox, woy],
                                                                 wg=[wgx, wgy],
                                                                 lane_o=lanes_o,
                                                                 lane_g=lanes_g,
                                                                 x_way = x_way[-1],
                                                                 y_way = y_way[-1])
                    
                    x_way[-1][indexes[0]:] = x_lc[:indexes[1]]
                    y_way[-1][indexes[0]:] = y_lc[:indexes[1]]
                    
                    if len(x_lc[indexes[1]:]) > 1:
                        x_way.append(x_lc[indexes[1]:])
                        y_way.append(y_lc[indexes[1]:])
                
            #different road
            else:
                
                #straight connection
                if self.has_edge(road_ido, road_idg):
                    for lane in [lanes_o, lanes_g]:
                        x_lk, y_lk = self.__get_lane_keeping(wo=[wox, woy], 
                                                             wg=[wgx, wgy], 
                                                             lane=lane)
                        if len(x_lk) > 1:
                            x_way.append(x_lk)
                            y_way.append(y_lk)
                            
                            wox = x_lk[-1]
                            woy = y_lk[-1]
                #connection withou edges (??)
                else:
                    #lane change
                    if abs(woy - wgy)>2.5 and abs(woy - wgy)<4.:
                        # print(f"LC: {woy - wgy}")
                        x_lc, y_lc, indexes = self.__get_lane_change(wo=[wox, woy],
                                                                     wg=[wgx, wgy],
                                                                     lane_o=lanes_o,
                                                                     lane_g=lanes_g,
                                                                     x_way = x_way[-1],
                                                                     y_way = y_way[-1])
                    
                        x_way[-1][indexes[0]:] = x_lc[:indexes[1]]
                        y_way[-1][indexes[0]:] = y_lc[:indexes[1]]
                        
                        if len(x_lc[indexes[1]:]) > 0:
                            x_way.append(x_lc[indexes[1]:])
                            y_way.append(y_lc[indexes[1]:])
                    #lane keeping
                    else:
                        for lane in [lanes_o, lanes_g]:
                            x_lk, y_lk = self.__get_lane_keeping(wo=[wox, woy],
                                                                wg=[wgx, wgy],
                                                                lane=lane)
                            
                            if len(x_lk) > 0:
                                x_way.append(x_lk)
                                y_way.append(y_lk)
                                
                                wox = x_lk[-1]
                                woy = y_lk[-1]                       
                        
        x_way = np.concatenate(x_way, axis=0)
        y_way = np.concatenate(y_way, axis=0)
        
        path = np.squeeze(np.dstack([x_way, y_way]))
        interp_path = []
        for p1, p2 in zip(path[:-1], path[1:]):
            dist = np.sqrt(np.power(p1-p2, 2).sum())
            
            if dist>1.5:
                continue
                # pm = (p1+p2)/2.
                # points = np.concatenate([[p1], [pm], [p2]], axis=0)
                # points[:, 0] = savgol_filter(points[:, 0], window_length=len(points), polyorder=1)
                # points[:, 1] = savgol_filter(points[:, 1], window_length=len(points), polyorder=1)
                # print(dist, points, pm)
                # interp_x = interpolate.interp1d(np.arange(0,len(points)), points[:, 0], kind='linear', fill_value="extrapolate")
                # interp_y = interpolate.interp1d(np.arange(0,len(points)), points[:, 1], kind='linear', fill_value="extrapolate")
                
                # x = interp_x(np.arange(0, 1, 0.5/dist))
                # y = interp_y(np.arange(0, 1, 0.5/dist))
                
                # interp_path.append(np.squeeze(np.dstack([x,y])))
            else:
                interp_path.append([p1])
        print(np.shape(interp_path), path.shape)
        interp_path = np.concatenate(interp_path, axis=0)
                        
        return interp_path
        