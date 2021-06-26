from heapq import heappush, heappop
from itertools import count
from datetime import timedelta,datetime,time

import networkx as nx
from networkx.utils import not_implemented_for

__all__ = ['astar_path', 'astar_path_length']


@not_implemented_for('multigraph')
def QHalgorithm(G, source, target, heuristic=None, weight='weight',curr_time =None):
    
    #set current time when the query is executed
    if curr_time == None:
        curr_time = datetime.now().time().replace(microsecond = 0)

    if source not in G or target not in G:
        msg = 'Either source {} or target {} is not in G'
        raise nx.NodeNotFound(msg.format(source, target))

    if heuristic is None:
        # The default heuristic is h=0 - same as Dijkstra's algorithm
        def heuristic(u, v):
            return 0

    push = heappush
    pop = heappop

    # The queue stores priority, node, cost to reach, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guarenteed unique for all nodes in the graph.
    c = count()
    queue = [(0, next(c), source, 0, None,curr_time)]

    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}

    while queue:
        # Pop the smallest item from queue.
        _, __, curnode, dist, parent,curr_time = pop(queue)

        if curnode == target:
            path = [curnode]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            return path

        if curnode in explored:
            continue

        explored[curnode] = parent

        ##let the magic begins             
        delta_current = timedelta(hours = curr_time.hour, minutes = curr_time.minute, seconds = curr_time.second, microseconds = 0 ) 

        for neighbor, w in G[curnode].items():
            if neighbor in explored:
                continue

            #skip no valid time events
            if neighbor[0] == 'node':
                if neighbor[-1] < curr_time :
                    continue
                else :
                    #calculate distance
                    delta_neighbor =  timedelta(hours = neighbor[-1].hour, minutes = neighbor[-1].minute, seconds = neighbor[-1].second, microseconds = 0 ) 
                    ncost = dist + int((delta_neighbor - delta_current).total_seconds())              
            else :
                ncost = dist + w.get(weight, 1)
                
            if neighbor in enqueued:
                qcost, h = enqueued[neighbor]
                    # if qcost < ncost, a longer path to neighbor remains
                    # enqueued. Removing it would need to filter the whole
                    # queue, it's better just to leave it there and ignore
                    # it when we visit the node a second time.
                #if False -> arc relaxation
                if qcost <= ncost:
                    continue
            else:
                #heuristic calculation
                h = heuristic(neighbor, target)
            #arc relaxation
            enqueued[neighbor] = ncost, h

            #Switch to Node or Node to Node
            if 'node' in neighbor :
                arrival_time = neighbor[-1]
            # Switch to Node  or Switch to Switch
            else:
                neighbor_departureTime = datetime(2020,1,1,curr_time.hour,curr_time.minute,curr_time.second)
                arrival_time = neighbor_departureTime + timedelta(seconds =  w.get(weight, 1))
                arrival_time = arrival_time.time().replace(microsecond = 0)
            
            push(queue, (ncost + h, next(c), neighbor, ncost, curnode, arrival_time))

    raise nx.NetworkXNoPath("Node %s not reachable from %s" % (source, target))


def astar_path_length(G, source, target, heuristic=None, weight='weight'):
    """Return the length of the shortest path between source and target using
    the A* ("A-star") algorithm.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node for path

    target : node
       Ending node for path

    heuristic : function
       A function to evaluate the estimate of the distance
       from the a node to the target.  The function takes
       two nodes arguments and must return a number.

    Raises
    ------
    NetworkXNoPath
        If no path exists between source and target.

    See Also
    --------
    astar_path

    """
    if source not in G or target not in G:
        msg = 'Either source {} or target {} is not in G'
        raise nx.NodeNotFound(msg.format(source, target))

    path = astar_path(G, source, target, heuristic, weight)
    return sum(G[u][v].get(weight, 1) for u, v in zip(path[:-1], path[1:]))
