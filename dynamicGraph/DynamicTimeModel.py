import numpy as np
import networkx as nx

class DynTM:
    #init object
    def __init__(self, max_walk_between_stations, station_coords, walk_speed):
        self.dtm = nx.DiGraph()
        self.__station_coords = station_coords 
        self.max_walk_between_stations = max_walk_between_stations
        self.walk_speed = walk_speed
        #target node without any connection
        self.dtm.add_node('switch_target')

    #add switch nodes to the graph
    def __add_switch_Nodes(self, timetable):
        prev_stations = timetable.next_station.unique()
        next_stations = timetable.prev_station.unique()
        switch_nodes = np.unique(np.concatenate([prev_stations,next_stations], axis=0))
        for node in switch_nodes : 
            #(node type, station_id, line_label)
            node_structure = ('switch', node)
            self.dtm.add_node(node_structure)

    #add  Switch Node --> Departure Node edges :
    def __add_switch_to_departureEdges(self, timetable):
        #add departure switch Node --> departure Node edges :
        for departure_node, departure_time ,line_id, line_label in timetable[['prev_station','horaires_depart','line_id','line_label']].values:
            depSwitch_structure = ('switch', departure_node)
            depNode_structure = ('node', line_id, departure_node, line_label, departure_time)
            self.dtm.add_edge(depSwitch_structure, depNode_structure, weight = 100000)

    #add  Departure Node --> Switch Node edges :
    def __add_departure_to_switchEdges(self, timetable):
        for arrival_node, departure_node, departure_time, weight ,line_id, line_label in timetable[['next_station','prev_station','horaires_depart','datetime_diff','line_id','line_label']].values:
            arrSwitch_structure = ('switch', arrival_node)
            depNode_structure = ('node', line_id, departure_node, line_label, departure_time)
            self.dtm.add_edge(depNode_structure, arrSwitch_structure, weight = weight)

    #add vehicle edges
    def __add_vehicleEdges(self, timetable):
        for departure_node, arrival_node, departure_time, arrival_time, weight, line_id, line_label in timetable[['prev_station','next_station','horaires_depart','horaires_arrivee','datetime_diff','line_id','line_label']].values:
            depNode_structure = ('node', line_id, departure_node, line_label, departure_time)
            arrNode_structure = ('node', line_id, arrival_node, line_label, arrival_time)
            self.dtm.add_edge(depNode_structure, arrNode_structure, weight = weight)
    
    #remove Trips from graph
    def removeTripFromGraph(self, timetable): #still in progress
        for i, departure_node, arrival_node, departure_time, arrival_time, line_id, line_label in enumerate(timetable[['prev_station','next_station','horaires_depart','horaires_arrivee','datetime_diff','line_id','line_label']].values):
            if i%2 == 0:
                depNode_structure = ('node', line_id, departure_node, line_label, departure_time)
                arrNode_structure = ('node', line_id, arrival_node, line_label, arrival_time)
                self.dtm.remove_node(depNode_structure)
                self.dtm.remove_node(arrNode_structure)

    #excute functions in proper order to generate a dynamic timetable model
    def generateTimetableDynamicModel(self, timetable):
        self.__add_switch_Nodes(timetable)    #add switch nodes to the graph
        self.__add_switch_to_departureEdges(timetable)   #add  Switch Node --> Departure Node edges
        self.__add_departure_to_switchEdges(timetable)   #add  Departure Node --> Switch Node edges :
        self.__add_vehicleEdges(timetable)    #add vehicle edges

    #add unrestricted connections to the graph
    def add_unresticted_paths(self, Dmatrix):
        #select unrestricted switch nodes and their respective distances
        Dmatrix = Dmatrix[Dmatrix < self.max_walk_between_stations].dropna(how='all')
        unrestricted_paths = {}
        for i in range(Dmatrix.shape[0]): 
            unrestricted_paths[Dmatrix.iloc[i].name] = (Dmatrix.iloc[i].dropna().index.values,Dmatrix.iloc[i].dropna().values)

        #add unrestricted edges
        for key, value in unrestricted_paths.items():
            for station,distance in zip(value[0],value[1]) :
                weight_time = int(distance/self.walk_speed)
                #add unrestricted paths to the graph in both ways
                key = int(float(key))
                station = int(float(station))
                self.dtm.add_edge(('switch',station) ,('switch',key), weight = weight_time)
                self.dtm.add_edge(('switch',key) ,('switch',station), weight = weight_time)

    def __haversine_distance(self, x, y):
        R = 6378137
        #convert to raduis
        lat1  = x[0] * np.pi/180
        long1 = x[1] * np.pi/180
        lat2  = y[0] * np.pi/180
        long2 = y[1] * np.pi/180
        #calculate haversine distance
        delta_longitude = long1 - long2
        delta_latitude = lat1 - lat2
        a = (np.sin(delta_latitude/2)**2) + np.cos(lat1)*np.cos(lat2)*(np.sin(delta_longitude/2)**2)
        c = 2*np.arctan2(np.sqrt(a),np.sqrt(1-a))
        distance = R*c
        return  distance

    def add_sourceNode(self, coords, max_StationDistance = np.inf ):
        #add node to graph:
        self.dtm.add_node('switch_source')
        #add unrestricted source node paths
        for stop in self.__station_coords.values : 
            #calculate distance between stop and source
            distance =  self.__haversine_distance(coords, stop[1:])
            if distance <= max_StationDistance :
                #add edge to the graph 
                weight_time = distance/self.walk_speed
                self.dtm.add_edge('switch_source', ('switch', stop[0]) , weight = weight_time)

    def add_targetNode(self, coords, max_StationDistance = np.inf ):
        #add node to graph:
        self.dtm.add_node('switch_target')
        #add unrestricted source node paths
        for stop in self.__station_coords.values : 
            #calculate distance between stop and source
            distance =  self.__haversine_distance(coords, stop[1:])
            if distance <= max_StationDistance :
                #add edge to the graph 
                weight_time = distance/self.walk_speed
                self.dtm.add_edge( ('switch', stop[0]), 'switch_target', weight = weight_time)

    def remove_switch(self, switch):
        self.dtm.remove_node(switch)


    def remove_trip(self, vehicle_id):
        for node in self.dtm.items():
            if vehicle_id in node :
                self.dtm.remove_node(node)