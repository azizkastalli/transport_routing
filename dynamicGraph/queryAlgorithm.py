import numpy as np
import networkx as nx
from heapq import heappush, heappop
from itertools import count
from datetime import timedelta,datetime,time


#calculate distance between two stops


def pareto_paths(G, source, target, curr_time, walk_speed=1.4, transfer_time=150, walk_panalisation=True, unrestricted_maxwalk = 300 ):
    
    if walk_panalisation :
        #scale distance 1km from 0 to 1
        penalisation = lambda x : int(np.exp(1-(1000 - x)/1000)) if x > unrestricted_maxwalk else 1
    else:
        penalisation = lambda x : 1

    push = heappush
    pop = heappop

    # The queue stores priority, node, cost to reach, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guarenteed unique for all nodes in the graph.
    queue = [( curr_time, 0, 0, 0,'switch_source', None)]
    c = count()
    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}

    first_target = True    
    paths = []
    transitions = []
    walked_distance = []
    bounds = [0, np.inf, np.inf]
     
    while queue:
        # Pop the smallest item from queue.
        dist, trans, walk_dist, _, curnode, parent = pop(queue)

        if curnode == 'switch_target' :
    
            if dist > bounds[0] and trans < bounds[1] and trans > 0 and walk_dist < bounds[2]:
                bounds[1] = trans
                path = [curnode]
                node = parent
                while node is not None:
                    path.append(node)
                    node = explored[node]
                path.reverse()
                paths.append(path)  #add path to set of pareto optimal paths
                transitions.append(trans)
                walked_distance.append(walk_dist)                


            if first_target:
                first_target = False
                bounds = [dist, trans, walk_dist]   #boundries for pareto set of optimal paths

        if curnode in explored : 
            continue
        explored[curnode] = parent


        for neighbor, w in G[curnode].items() :

            #Switch -> Node
            if 'switch' in curnode and 'node' in neighbor:
                #add constraint : +2:30 mins of transfer time from one vehicle to another needs to be considered
                if (neighbor[-1] - dist)  < transfer_time :  #skip no valid time events
                    continue
                else:
                    nwalk_dist = walk_dist
                    ntrans  = trans
                    ncost   = neighbor[-1]

            #walk Switch -> Switch
            elif 'switch' in curnode and 'switch' in neighbor:
                nwalk_dist = walk_dist + int(w.get('weight', 1) * walk_speed)     #keep tracking of walked distance in meters
                ntrans  = trans    #keep tracking transitions between stops
                ncost   = dist + w.get('weight', 1) * penalisation(w.get('weight', 1))

            else :
                #Node -> Switch
                if 'switch' in neighbor:
                    ntrans  = trans + 1
                #Node -> Node
                else:
                    ntrans  = trans
                ncost = dist + w.get('weight', 1)
                nwalk_dist = walk_dist

            if neighbor in enqueued:
                qcost, qtrans, qwalk_dist = enqueued[neighbor]
                    # if qcost < ncost, a longer path to neighbor remains
                    # enqueued. Removing it would need to filter the whole
                    # queue, it's better just to leave it there and ignore
                    # it when we visit the node a second time.
                #if False -> arc relaxation

                #bi-criteria optimization (time travel and vehicle transfer)
                if ncost >= qcost :
                    if  not (ntrans < qtrans or nwalk_dist < qwalk_dist) :
                        continue

            #arc relaxation
            enqueued[neighbor] = ncost, ntrans, nwalk_dist
            
            push(queue, (ncost, ntrans, nwalk_dist, next(c), neighbor, curnode))

    return paths, transitions, walked_distance




def earliestArrival_path(G, source, target, curr_time, walk_speed=1.4, transfer_time=150, walk_panalisation=True, unrestricted_maxwalk = 300 ):
    
    if walk_panalisation :
        #scale distance 1km from 0 to 1
        penalisation = lambda x : int(np.exp(1-(1000 - x)/1000)) if x > unrestricted_maxwalk else 1
    else:
        penalisation = lambda x : 1

    push = heappush
    pop = heappop

    # The queue stores priority, node, cost to reach, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guarenteed unique for all nodes in the graph.
    queue = [( curr_time, 0, 0,'switch_source', None)]
    c = count()
    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}


    while queue:
        # Pop the smallest item from queue.
        dist, trans, _, curnode, parent = pop(queue)

        if curnode == 'switch_target' :
            path = [curnode]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            return path, dist, trans      


        if curnode in explored : 
            continue
        explored[curnode] = parent


        for neighbor, w in G[curnode].items() :

            #Switch -> Node
            if 'switch' in curnode and 'node' in neighbor:
                #add constraint : +2:30 mins of transfer time from one vehicle to another needs to be considered
                if (neighbor[-1] - dist)  < transfer_time :  #skip no valid time events
                    continue
                else:
                    ntrans  = trans
                    ncost   = neighbor[-1]

            #walk Switch -> Switch
            elif 'switch' in curnode and 'switch' in neighbor:
                ntrans  = trans    #keep tracking transitions between stops
                ncost   = dist + w.get('weight', 1) * penalisation(w.get('weight', 1))

            else :
                #Node -> Switch
                if 'switch' in neighbor:
                    ntrans  = trans + 1
                #Node -> Node
                else:
                    ntrans  = trans
                ncost = dist + w.get('weight', 1)

            if neighbor in enqueued:
                qcost, qtrans = enqueued[neighbor]
                    # if qcost < ncost, a longer path to neighbor remains
                    # enqueued. Removing it would need to filter the whole
                    # queue, it's better just to leave it there and ignore
                    # it when we visit the node a second time.
                #if False -> arc relaxation

                #bi-criteria optimization (time travel and vehicle transfer)
                if ncost >= qcost and not ntrans < qtrans :
                    continue

            #arc relaxation
            enqueued[neighbor] = ncost, ntrans
            
            push(queue, (ncost, ntrans, next(c), neighbor, curnode))

    raise nx.NetworkXNoPath("Node %s not reachable from %s" % (source, target))