#!/usr/bin/env python3

import networkx as nx
import os

current_dir = os.path.dirname(os.path.realpath(__file__))

attributes = ["normal", "crosswalk", "intersection", "oneway", "highwayLeft", "highwayRight", "roundabout", "stopline", "dotted", "dotted_crosswalk"]
# Load the graphml file
file_path = current_dir + '/maps/Competition_track_graph_new.graphml'
print('Loading graphml file from: ' + file_path)
graph = nx.read_graphml(file_path)

crosswalk = [ 151, 51, 258, 259, 252, 253, 254, 226, 227, 284, 285]
dotted_crosswalk = [ 163, 162, 166, 165, 176, 175, 151]
highwayRight = []
highwayLeft = []
for i in range(444, 463):
    highwayRight.append(i)
for i in range(483, 502):
    highwayLeft.append(i)
for i in range(502, 521):
    highwayLeft.append(i)
for i in range(463, 483):
    highwayRight.append(i)

oneway = [51, 49]
for i in range(188, 199):
    oneway.append(i)
for i in range(263, 271):
    oneway.append(i)
for i in range(223, 264):
    oneway.append(i)
for i in range(271, 290):
    oneway.append(i)
for i in range(296, 302):
    oneway.append(i)

roundabout = []
for i in range(331, 342):
    roundabout.append(i)

intersection = [78, 76, 77, 208, 210, 211, 207, 186, 244, 262, 245, 103, 105, 104, 34, 35, 33, 36, 47, 45, 48, 46, 290, 291, 200, 59, 60, 94,95,96, 22,21,23, 10,11,9,12, 85,86,87]
stopline = [397, 405, 317, 367, 56, 54, 243, 261, 75, 185, 71, 73, 206, 28, 40, 98, 26, 100, 30, 38, 102, 32, 44, 42, 199, 289, 4, 6, 2, 14,18,16, 91, 89, 93, 80, 84, 137, 148]
for node in graph.nodes:
    graph.nodes[node]['new_attribute'] = 0
# Change nodes that have 'dotted' edges to 'dotted' attribute
for edge in graph.edges:
    if graph.edges[edge]['dotted']:
        graph.nodes[edge[0]]['new_attribute'] = 8
        graph.nodes[edge[1]]['new_attribute'] = 8
for node in graph.nodes:
    if int(node) in stopline:
        graph.nodes[node]['new_attribute'] = 7
    elif int(node) in crosswalk:
        if graph.nodes[node]['new_attribute'] == 8: # If dotted
            graph.nodes[node]['new_attribute'] = 9
        else:
            graph.nodes[node]['new_attribute'] = 1
    elif int(node) in highwayLeft:
        graph.nodes[node]['new_attribute'] = 4
    elif int(node) in highwayRight:
        graph.nodes[node]['new_attribute'] = 5
    elif int(node) in oneway:
        graph.nodes[node]['new_attribute'] = 3
    elif int(node) in roundabout:
        graph.nodes[node]['new_attribute'] = 6
    elif int(node) in intersection:
        graph.nodes[node]['new_attribute'] = 2
    elif int(node) in dotted_crosswalk:
        graph.nodes[node]['new_attribute'] = 9

# Save the graphml file
file_path = current_dir + '/Competition_track_graph_modified_new.graphml'
nx.write_graphml(graph, file_path)

print('Done')