from collections import deque
import heapq

class gr:

    def dfs(graph, start, visited=None):
        # Создаем пустое множество для хранения посещенных вершин
        if visited is None:
            visited = set()
        # Добавляем текущую вершину в множество посещенных
        visited.add(start)
        # Выводим текущую вершину
        print(start)
        # Рекурсивно вызываем dfs для всех смежных вершин
        for next in graph[start] - visited:
            gr.dfs(graph, next, visited)
        # Возвращаем множество посещенных вершин
        return visited



    def bfs(graph, start):
        # Создаем пустую очередь для хранения вершин, которые нужно посетить
        queue = deque([start])
        # Создаем пустое множество для хранения посещенных вершин
        visited = set()
        # Добавляем текущую вершину в множество посещенных
        visited.add(start)
        # Пока очередь не пуста
        while queue:
            # Получаем следующую вершину из очереди
            vertex = queue.popleft()
            # Выводим текущую вершину
            print(vertex)
            # Добавляем все смежные вершины в очередь и множество посещенных
            for neighbor in graph[vertex]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)


    def dijkstra(graph, start):
        # Создаем словарь для хранения расстояний до каждой вершины
        distances = {vertex: float('inf') for vertex in graph}
        # Устанавливаем расстояние до начальной вершины равным 0
        distances[start] = 0
        # Создаем очередь с приоритетом для хранения вершин и расстояний до них
        pq = [(0, start)]
        # Пока очередь не пуста
        while pq:
            # Получаем вершину с наименьшим расстоянием из очереди
            current_distance, current_vertex = heapq.heappop(pq)
            # Если текущее расстояние до вершины больше, чем уже известное расстояние, пропускаем ее
            if current_distance > distances[current_vertex]:
                continue
            # Для каждой смежной вершины
            for neighbor, weight in graph[current_vertex].items():
                # Вычисляем расстояние до смежной вершины через текущую вершину
                distance = current_distance + weight
                # Если это расстояние меньше, чем уже известное расстояние до смежной вершины, обновляем его
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    # Добавляем смежную вершину в очередь с приоритетом
                    heapq.heappush(pq, (distance, neighbor))
        return distances


    def kruskal(graph):
        # Создаем список ребер и сортируем его по возрастанию весов
        edges = [(weight, start, end) for start, others in graph.items()
                for end, weight in others.items()]
        edges.sort()
        # Создаем словарь для хранения множеств вершин
        nodes = {node: set([node]) for node in graph}
        # Создаем пустой список для хранения выбранных ребер
        mst = []
        # Для каждого ребра
        for weight, start, end in edges:
            # Если начальная и конечная вершины не принадлежат одному множеству
            if nodes[start] != nodes[end]:
                # Добавляем ребро в минимальное остовное дерево
                mst.append((start, end, weight))
                # Объединяем множества вершин
                nodes[start].update(nodes[end])
                for node in nodes[end]:
                    nodes[node] = nodes[start]
        return mst


    def prim(graph, start):
        # Создаем словарь для хранения множеств вершин и ребер
        nodes = {node: float('inf') for node in graph}
        edges = {}
        # Устанавливаем начальную вершину
        nodes[start] = 0
        queue = [(0, start)]
        # Пока очередь не пуста
        while queue:
            # Получаем вершину с наименьшим весом
            weight, current = heapq.heappop(queue)
            # Если вершина уже обработана, пропускаем ее
            if nodes[current] < weight:
                continue
            # Для каждой смежной вершины
            for neighbor, neighbor_weight in graph[current].items():
                # Если новый вес меньше старого
                if neighbor_weight < nodes[neighbor]:
                    # Обновляем вес вершины
                    nodes[neighbor] = neighbor_weight
                    # Добавляем ребро в словарь ребер
                    edges[neighbor] = current
                    # Добавляем вершину в очередь с новым весом
                    heapq.heappush(queue, (neighbor_weight, neighbor))
        # Создаем список ребер и добавляем их в минимальное остовное дерево
        mst = [(edges[node], node, weight) for node, weight in nodes.items() if node != start]
        return mst

    # Функция для реализации алгоритма Флойда-Уоршалла
    def floydWarshall(graph, V):
        dist = [[0 for j in range(V)] for i in range(V)]
        INF = 99999
        for i in range(V):
            for j in range(V):
                dist[i][j] = graph[i][j]
    
        for k in range(V):
            for i in range(V):
                for j in range(V):
                    dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j])
    
        print("Матрица кратчайших расстояний:")
        for i in range(V):
            for j in range(V):
                if dist[i][j] == INF:
                    print("{:7s}".format("INF"), end="")
                else:
                    print("{:7d}".format(dist[i][j]), end="")
            print("")

