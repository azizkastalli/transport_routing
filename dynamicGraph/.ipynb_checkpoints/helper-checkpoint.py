import numpy as np


traduction = {'french' : ["aller à pied jusqu'à votre destination","aller à pied jusqu'à la station ",'descendre du véhicule à la station ','et prendre la ligne ',' à ','de la station '],
             'english' : ['walk to your destination','walk to stop ','get off the vehicle at stop ','and take the route ',' at ','from stop ']}

def reverse_int_to_time(time):
    H = time // 60 // 60
    M = ( time - (H*60*60) ) // 60
    S = time - M*60 - H*60*60
    return str(H)+'H '+str(M)+'m '+str(S)+'s'

#translate path
def path_interpretor(path, stations_paths, paths,language='english'): 
    path = path[1:-1]
    msg = traduction[language][1] + stations_paths[stations_paths.station_id == path[0][1] ][language+'_name'].values[0] + ' '
    prev_node = ''
    for node in path : 
        if prev_node == None:
            msg += traduction[language][1] + stations_paths[stations_paths.station_id == node[1] ][language+'_name'].values[0] + '\n'
        elif 'switch' in node and 'node' in prev_node:
            msg += traduction[language][2] + stations_paths[(stations_paths.station_id == node[1]) ][language+'_name'].values[0] + '\n'            
        elif 'node' in node and 'switch' in prev_node:
            msg += traduction[language][3] +'('+node[3]+') : '+ paths[paths.line_id == node[1] ][language+'_name'].values[0] + traduction[language][4] + reverse_int_to_time(node[-1]) +'\n'
        elif 'switch' in node and 'switch' in prev_node:
            msg += traduction[language][5] + stations_paths[stations_paths.station_id == prev_node[1] ][language+'_name'].values[0] + ' ' + \
            traduction[language][1] + stations_paths[stations_paths.station_id == node[1] ][language+'_name'].values[0] + '\n'
        prev_node = node
    msg += traduction[language][0]
    return msg


def haversine_affinity(x,y):
    R = 6378137 #in meter

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
    
    return distance


def str_time_to_int(ch):
    ch = ch.split(':')
    H  = int(ch[0])*60*60
    M  = int(ch[1])*60
    S  = int(ch[2])
    Time = (H+M+S)%86400
    return Time
