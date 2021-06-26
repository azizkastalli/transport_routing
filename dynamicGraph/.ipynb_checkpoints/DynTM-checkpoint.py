import numpy as np
import networkx as nx
from heapq import heappush, heappop
from itertools import count
from datetime import timedelta,datetime,time

class DynTM:
    #init object
    def __init__(self, max_walk_between_stations, walk_speed):
        self.dtm = nx.DiGraph()
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

    #add vehicule edges
    def __add_vehiculeEdges(self, timetable):
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
        self.__add_vehiculeEdges(timetable)    #add vehicule edges

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
                self.dtm.add_edge(('switch',station) ,('switch',key), weight=weight_time)
                self.dtm.add_edge(('switch',key) ,('switch',station), weight=weight_time)




traduction = {'french' : ["aller à pied jusqu'à votre destination","aller à pied jusqu'à la station ",'descendre du véhicule à la station ','et prendre la ligne ',' à ','de la station '],
             'english' : ['walk to your destination','walk to stop ','get off the vehicule at stop ','and take the route ',' at ','from stop ']}

#translate path
def path_interpretor(path, stations_paths, paths,language='english'): 
    path = path[1:-1]
    msg = traduction[language][1] + stations_paths[stations_paths.station_id == path[0][1] ][language+'_name'].item() + ' '
    prev_node = ''
    for node in path : 
        if prev_node == None:
            msg += traduction[language][1] + stations_paths[stations_paths.station_id == node[1] ][language+'_name'].item() + '\n'
        elif 'switch' in node and 'node' in prev_node:
            msg += traduction[language][2] + stations_paths[(stations_paths.station_id == node[1]) ][language+'_name'].item() + '\n'            
        elif 'node' in node and 'switch' in prev_node:
            msg += traduction[language][3] +'('+node[3]+') : '+ paths[paths.line_id == node[1] ][language+'_name'].item() + traduction[language][4] + str(node[-1]) +'\n'
        elif 'switch' in node and 'switch' in prev_node:
            msg += traduction[language][5] + stations_paths[stations_paths.station_id == prev_node[1] ][language+'_name'].item() + ' ' + \
            traduction[language][1] + stations_paths[stations_paths.station_id == node[1] ][language+'_name'].item() + '\n'
        prev_node = node
    msg += traduction[language][0]
    return msg


# Your method to calculate distance between two samples
def haversine_SD(x,y,max_StationDistance,walk_speed):
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
    if distance < max_StationDistance :
        return  distance/walk_speed


def QHalgorithm(G, source, target, curr_time, stations_coord, walk_speed=1.4, max_StationDistance=300, weight='weight', transfer_time=150, walk_panalisation=0.3 ):
    #get nearest stations to source and target
    #source : ('switch_source', latitude, longitude)
    #target : ('switch_target', latitude, longitude)
    walk_panalisation = 1 - walk_panalisation
    SourceNeighbors = []
    distance_to_target = {}

    for station in stations_coord.values:
        distance_source = haversine_SD(station[1:], source[1:], max_StationDistance, walk_speed)
        #add switch nodes close to source
        if distance_source != None :
            SourceNeighbors.append((('switch',int(station[0])),{'weight' : int(distance_source)}))
        #add distance of all switch nodes to target
        distance_target = haversine_SD(station[1:], target[1:], max_StationDistance, walk_speed)
        if distance_target != None :
            distance_to_target[('switch',int(station[0]))] = {'weight' : int(distance_target)}

    #add source switch 
    distance_target = haversine_SD(source[1:], target[1:], max_StationDistance, walk_speed)
    if distance_target != None :
        distance_to_target['switch_source'] = {'weight' : int(distance_target)}


    push = heappush
    pop = heappop

    # The queue stores priority, node, cost to reach, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guarenteed unique for all nodes in the graph.
    c = count()
    queue = [(0, next(c), 'switch_source', 0, None, curr_time)]

    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}

    while queue:
        # Pop the smallest item from queue.
        _, __, curnode, dist, parent,curr_time = pop(queue)

        if curnode == 'switch_target':
            path = [curnode]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            return path, curr_time

        if curnode in explored:
            continue

        explored[curnode] = parent

        delta_current = timedelta(hours = curr_time.hour, minutes = curr_time.minute, seconds = curr_time.second, microseconds = 0 ) 

        if 'switch_source' in curnode :
            curnode_neighbors = SourceNeighbors
            n = len(curnode_neighbors)
        else :
            n = len(G[curnode].items())
            curnode_neighbors = iter(G[curnode].items())
        
        i = 0
        first = True
        while i < n :
            if 'switch' == curnode[0] and first:
                first = False
                if curnode in distance_to_target:
                    neighbor, w  = 'switch_target', distance_to_target[curnode]
                else:
                    continue
            elif 'switch_source' in curnode:
                neighbor, w  = curnode_neighbors[i]                
                i += 1
            else:
                neighbor, w  = next(curnode_neighbors)
                i += 1

            if neighbor in explored:
                continue

            #skip no valid time events
            if 'switch' in curnode and 'node' in neighbor:
                #add constraint : +2:30 mins of transfer time from one vehicule to another needs to be considered
                delta_neighbor =  timedelta(hours = neighbor[-1].hour, minutes = neighbor[-1].minute, seconds = neighbor[-1].second, microseconds = 0 ) 
                if (delta_neighbor - delta_current).total_seconds() < transfer_time :
                    continue
                else:
                    ncost = dist + int((delta_neighbor - delta_current).total_seconds())      
            #walk
            elif 'switch' in curnode and 'switch' in neighbor:
                ncost = dist + w.get(weight, 1)*walk_panalisation
            else :
                ncost = dist + w.get(weight, 1)
                
            if neighbor in enqueued:
                qcost = enqueued[neighbor]
                    # if qcost < ncost, a longer path to neighbor remains
                    # enqueued. Removing it would need to filter the whole
                    # queue, it's better just to leave it there and ignore
                    # it when we visit the node a second time.
                #if False -> arc relaxation
                if qcost <= ncost:
                    continue

            #arc relaxation
            enqueued[neighbor] = ncost

            #Switch to Node or Node to Node
            if 'node' in neighbor :
                arrival_time = neighbor[-1]
            # Node to Switch or Switch to Switch
            else:
                neighbor_departureTime = datetime(2020,1,1,curr_time.hour,curr_time.minute,curr_time.second)
                arrival_time = neighbor_departureTime + timedelta(seconds =  w.get(weight, 1))
                arrival_time = arrival_time.time().replace(microsecond = 0)
            
            push(queue, (ncost, next(c), neighbor, ncost, curnode, arrival_time))

    raise nx.NetworkXNoPath("Node %s not reachable from %s" % (source, target))