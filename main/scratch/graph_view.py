import networkx as nx
import matplotlib.pyplot as plt 


G = nx.grid_2d_graph(4,4)

for edge in G.edges:
    G.edges[edge]['weight'] = 1

G.add_edges_from([
    ((x, y), (x+1, y+1))
    for x in range(3)
    for y in range(3)
] + [
    ((x+1, y), (x, y+1))
    for x in range(3)
    for y in range(3)
], weight=1.4)

print(nx.astar_path(G,(1,1),(3,3)))

pos = nx.spring_layout(G, iterations=100)

nx.draw(G, pos, with_labels = True)

plt.show()


