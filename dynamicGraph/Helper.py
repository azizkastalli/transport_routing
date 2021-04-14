# Your method to calculate distance between two samples



traduction = {'french' : ["aller à pied jusqu'à la station ",'descendre du véhicule à la station ','et prendre la ligne ',' à ','de la station '],
             'english' : ['walk to stop ','get off the vehicule at stop ','and take the route ',' at ','from stop '],
             'arabic'  : [' إنطلاقا من محطة ',' على الساعة ','ثم خذ مركبة النقل في الإتجاه : ',' ثم إنزل من مركبة النقل عند محطة ', 'سرْ على الأقدام إلى محطة'] }

def path_interpretor(path,language='english'):    
    msg = ''
    prev_node = ''
    for node in path : 
        if prev_node == None:
            msg += traduction[language][0] + stations_paths[stations_paths.station_id == node[1] ][language+'_name'].item() + '\n'
        elif 'switch' in node and 'node' in prev_node:
            msg += traduction[language][1] + stations_paths[(stations_paths.station_id == node[1]) ][language+'_name'].item() + '\n'            
        elif 'node' in node and 'switch' in prev_node:
            msg += traduction[language][2] +'('+node[3]+') : '+ paths[paths.line_id == node[1] ][language+'_name'].item() + traduction[language][3] + str(node[-1]) +'\n'
        elif 'switch' in node and 'switch' in prev_node:
            msg += traduction[language][4] + stations_paths[stations_paths.station_id == prev_node[1] ][language+'_name'].item() + ' ' + \
            traduction[language][0] + stations_paths[stations_paths.station_id == node[1] ][language+'_name'].item() + '\n'
        prev_node = node
    return msg

