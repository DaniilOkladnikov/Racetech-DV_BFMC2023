import networkx as nx
import matplotlib.pyplot as plt

# import file with node information
file = 'Competition_track_graph.graphml'

# read file with networkx
mesh = nx.read_graphml(file)

# read coords of one node
def read_node(node):
	return mesh.nodes[str(node)]

# read coords of multiple nodes
def read_nodes(nodes):
	coords = []
	for node in nodes:
		coord = [node, read_node(node)]
		coords.append(coord)

	return coords

# read edges in and out one node
# in and out nodes are stored in special type (because there can be multiple ins and outs...)
def read_edges(node):
	data = [mesh.in_edges(str(node)), mesh.out_edges(str(node))]
	return data

# old tests
#print(read_node(4))
#print(read_edge(79))
#print(read_nodes([101, 28, 35, 25, 122]))
# 5.15, 10.94   5.14, 10.72   4.95, 9.94   4.18, 9.75   3.79, 9.77
"""
# convert all node information into lists (x, y) for matplotlib
all_nodes = mesh.nodes(data=True)
cx = []
cy = []
for node in all_nodes:
	cx.append(node[1]['x'])
	cy.append(node[1]['y'])

print(cx, "\n\n", cy)

# display
plt.cla()
plt.gca().invert_yaxis()	# invert y axis
plt.plot(cx, cy, ".r", label="nodes")
#plt.plot(states.x, states.y, "-b", label="trajectory")
plt.legend()
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.axis("equal")
plt.grid(True)
plt.show()
"""

node_list = [105, 106, 107, 108, 109, 110, 111, 83, 88, 86, 136, 137, 138, 74, 80, 77, 146, 147, 148, 65, 70, 66, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177]
cx = []
cy = []
data = read_nodes(node_list)
for node in data:
	cx.append(node[1]['x'])
	cy.append(node[1]['y'])

print(cx, "\n\n", cy)
# display
plt.cla()
plt.gca().invert_yaxis()	# invert y axis
plt.plot(cx, cy, ".r", label="nodes")
#plt.plot(states.x, states.y, "-b", label="trajectory")
plt.legend()
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.axis("equal")
plt.grid(True)
plt.show()