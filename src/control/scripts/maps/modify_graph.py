import networkx as nx
import os

current_dir = os.path.dirname(os.path.realpath(__file__))

attributes = ["normal", "crosswalk", "intersection", "oneway", "highwayLeft", "highwayRight", "roundabout", "stopline", "dotted", "dotted_crosswalk"]
# Load the graphml file
file_path = current_dir + '/Competition_track_graph_new.graphml'
print('Loading graphml file from: ' + file_path)
graph = nx.read_graphml(file_path)

crosswalk = [415, 416, 417, 418, 402, 403, 404, 439, 440, 441, 442, 163,164, 165, 67, 66, 166, 167, 7, 152,153, 176,177, 8, 451, 452, 453, 454]
highwayLeft = []
for i in range(263, 282):
    highwayLeft.append(i)
for i in range(216, 235):
    highwayLeft.append(i)
highwayRight = []
for i in range(283, 302):
    highwayRight.append(i)
for i in range(239, 259):
    highwayRight.append(i)

oneway = [30]
for i in range(448, 480):
    oneway.append(i)
for i in range(401, 424):
    oneway.append(i)
for i in range(480, 490):
    oneway.append(i)

roundabout = []
for i in range(358, 369):
    roundabout.append(i)

intersection = [81, 400, 424, 70, 71, 72, 43, 44, 10, 45, 12, 11, 9, 88, 89, 90, 22, 23, 24, 21, 35, 33, 34, 36, 54, 53, 52, 63, 62]
stopline = [470, 422, 407, 385, 445, 6, 30, 486, 447, 60, 47, 58, 28, 26, 18, 32, 4, 2, 42, 38, 76, 40, 20, 85, 14, 16, 49, 51, 83, 74, 87, 78, 65, 69]
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

# Save the graphml file
file_path = current_dir + '/Competition_track_graph_modified.graphml'
nx.write_graphml(graph, file_path)

print('Done')