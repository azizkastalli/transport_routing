import numpy as np
import networkx as nx

#Note:
#timetable index : 
# 0 --> TripNb
# 1 --> line_id
# 2 --> vehicle_type
# 3 --> currentStop
# 4 --> nextStop
# 5 --> departure_time
# 6 --> nextStop_arrival_time
# 7 --> nextStop_deaprture_time

class DynTM:
    #init object
    def __init__(self, max_walk_between_stations, station_coords, walk_speed):
        self.dtm = nx.DiGraph()
        self.__station_coords = station_coords 
        self.max_walk_between_stations = max_walk_between_stations
        self.walk_speed = walk_speed

    #add switch nodes to the graph
    def add_switch_Nodes(self, timetable):
        for node in timetable[0] :
            #(node_type, station_id)
            switch_node = ('switch', node)
            self.dtm.add_node(switch_node)

    def update_departureEvents(self, timetable):
        for tripNb, line_id, vehicle_type, currentStop, nextStop, departure_time, nextStop_arrival_time, nextStop_departure_time in timetable:
            switch_node = ('switch', nextStop)
            departure_node = ('node', line_id, currentStop, tripNb,departure_time)
            arrival_node   = ('node', line_id, nextStop, tripNb, nextStop_departure_time)
            self.dtm.add_edge(switch_node, departure_node, weight = 100000)
            weight = nextStop_arrival_time - departure_time
            self.dtm.add_edge(departure_node, switch_node, weight = weight)
            weight = nextStop_departure_time - departure_time 
            self.dtm.add_edge(departure_node, arrival_node, weight = weight)
            
    #add  Switch Node --> Departure Node edges :
    def add_switch_to_departureEdges(self, timetable):
        for tripNb, line_id, currentStop, departure_time  in timetable[1][:,[0,1,3,5]]:
            switch_node  = ('switch', currentStop)
            departure_node  = ('node', line_id, currentStop, tripNb, departure_time)
            self.dtm.add_edge(switch_node, departure_node, weight = 100000)

    #add  Departure Node --> Switch Node edges :
    def add_departure_to_switchEdges(self, timetable):
        for tripNb, line_id, currentStop, nextStop, departure_time, nextStop_arrival_time in timetable[1][:,[0,1,3,4,5,6]]:
            switch_node = ('switch', nextStop)
            departure_node = ('node', line_id, currentStop, tripNb,departure_time)
            weight = nextStop_arrival_time - departure_time
            self.dtm.add_edge(departure_node, switch_node, weight = weight)

    #add vehicle edges
    def add_vehicleEdges(self, timetable):
        for tripNb, line_id, currentStop, nextStop, departure_time, nextStop_departure_time in timetable[1][:,[0,1,3,4,5,7]]:
            departure_node = ('node', line_id, currentStop, tripNb, departure_time)
            arrival_node   = ('node', line_id, nextStop, tripNb, nextStop_departure_time)
            weight = nextStop_departure_time - departure_time 
            self.dtm.add_edge(departure_node, arrival_node, weight = weight)
    '''
    #remove Trips from graph
    def removeTrip(self, TripNb, line_id): #still in progress
            depNode_structure = ('node', line_id, departure_node, line_label, departure_time)
            arrNode_structure = ('node', line_id, arrival_node, line_label, arrival_time)
            self.dtm.remove_node(depNode_structure)
            self.dtm.remove_node(arrNode_structure)

    #Add Trips to graph
    def addTrip(self, TripNb, line_id): #still in progress
            depNode_structure = ('node', line_id, departure_node, line_label, departure_time)
            arrNode_structure = ('node', line_id, arrival_node, line_label, arrival_time)
            self.dtm.remove_node(depNode_structure)
            self.dtm.remove_node(arrNode_structure)
    '''
    #excute functions in proper order to generate a dynamic timetable model
    def generateTimetableDynamicModel(self, timetable):
        self.add_switch_Nodes(timetable)    #add switch nodes to the graph
        self.add_switch_to_departureEdges(timetable)   #add  Switch Node --> Departure Node edges
        self.add_departure_to_switchEdges(timetable)   #add  Departure Node --> Switch Node edges :
        self.add_vehicleEdges(timetable)    #add vehicle edges

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

    '''
    def remove_trip(self, vehicle_id):
        for node in self.dtm.items():
            if vehicle_id in node :
                self.dtm.remove_node(node)
    '''