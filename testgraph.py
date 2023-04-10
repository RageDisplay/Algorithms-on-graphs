from graph import gr

graph = {'A': set(['B', 'C']),
         'B': set(['A', 'D', 'E']),
         'C': set(['A', 'F']),
         'D': set(['B']),
         'E': set(['B', 'F']),
         'F': set(['C', 'E'])}

gr.dfs(graph, 'A')
print("--------------------------------------------")
gr.bfs(graph, 'A')
print("--------------------------------------------")

graph = {'A': {'B': 2, 'C': 1},
         'B': {'A': 2, 'D': 3, 'E': 1},
         'C': {'A': 1, 'F': 4},
         'D': {'B': 3},
         'E': {'B': 1, 'F': 1},
         'F': {'C': 4, 'E': 1}}

print(gr.dijkstra(graph, 'A'))
print("--------------------------------------------")
print(gr.kruskal(graph))
print("--------------------------------------------")
print(gr.prim(graph, 'A'))
print("--------------------------------------------")


INF = 99999
graph = [[0, 5, INF, 10],
         [INF, 0, 3, INF],
         [INF, INF, 0, 1],
         [INF, INF, INF, 0]]

gr.floydWarshall(graph, 4)
print("--------------------------------------------")