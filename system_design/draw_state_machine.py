from igraph import *

g = Graph(directed=True)
g["name"] = "AlphaPilot State Machine"
"""
0: start
1: explorer
2: gate tracking
3: stop
4: validate gate
5: dead lock
6: search strategy
"""

num_nodes = 7
g.add_vertices(num_nodes)
g.vs["label"] = ["start", "explorer", "gate \n tracker", "stop", "gate \n validator", "deadlock", "planner"]
g.add_edge(source=0, target=1, label="hover")
g.add_edge(source=1, target=6)
g.add_edge(source=6, target=1, label="strategy")
g.add_edge(source=1, target=4)
g.add_edge(source=4, target=1, label="detect gate")
g.add_edge(source=4, target=2, label="has gate")
g.add_edge(source=2, target=1, label="pass \n gate")
g.add_edge(source=2, target=5)
g.add_edge(source=5, target=2)
g.add_edge(source=1, target=3, label="back to start")

layout = g.layout_fruchterman_reingold() #g.layout_kamada_kawai()
plot(g, layout=layout, bbox=(800, 800), margin=50,
     edge_width=1.25, edge_label_cex=0.8,
     vertex_label_dist=0, vertex_size=75, vertex_label_color="blue", vertex_color="white", vertex_order=range(num_nodes))
