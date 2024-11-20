import numpy as np
import heapq  # For priority queue in Dijkstra's algorithm
import matplotlib.pyplot as plt

def set_graph(n, m=None):
    """
    Create a grid-based graph represented as an adjacency matrix.
    """
    if m is None:
        m = n

    graph = np.full((n * m, n * m), np.inf)

    for i in range(n * m):
        t = 1 
        if i % n != 0:  # Horizontal connection
            graph[i, i - 1] = t
            graph[i - 1, i] = t

        if i >= n:  # Vertical connection
            graph[i, i - n] = t
            graph[i - n, i] = t

    return graph

def dijkstra(graph, start, end):
    """
    Implements Dijkstra's algorithm to find the shortest path and distance
    from the start node to the end node.
    """
    if start == end:
        return 0, [start]

    n = graph.shape[0]
    dist = np.full(n, np.inf)
    dist[start] = 0
    parent = {start: None}

    # Use a priority queue for efficient minimum distance extraction
    pq = [(0, start)]  # (distance, node)
    visited = set()

    while pq:
        current_dist, x = heapq.heappop(pq)

        if x in visited:
            continue
        visited.add(x)

        # Stop if we reach the destination
        if x == end:
            break

        for y in range(n):
            if graph[x, y] != np.inf and y not in visited:
                new_dist = current_dist + graph[x, y]
                if new_dist < dist[y]:
                    dist[y] = new_dist
                    parent[y] = x
                    heapq.heappush(pq, (new_dist, y))

    if dist[end] == np.inf:
        return np.inf, []  # No path found

    # Reconstruct the path
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()

    return dist[end], path

def update_graph_on_obstacle(graph, i, j):
    """
    Updates the graph to add an obstacle by setting the weight between nodes i and j to infinity.
    """
    graph[i, j] = np.inf
    graph[j, i] = np.inf
    return graph

def plot_grid(n, path=None, new_path=None, obstacle=None):
    """
    Visualizes the grid with optional paths and obstacles.
    """
    plt.figure(figsize=(8, 8))
    for i in range(n):
        for j in range(n):
            node = i * n + j
            plt.scatter(j, -i, c='blue')
            plt.text(j, -i, f'{node}', fontsize=8, ha='center', va='center')

    if path:
        for idx in range(len(path) - 1):
            x1, y1 = path[idx] % n, -np.floor(path[idx] / n)
            x2, y2 = path[idx + 1] % n, -np.floor(path[idx+1] / n)
            plt.plot([x1, x2], [y1, y2], c='red', linewidth=2, label="Old Path" if idx == 0 else "")

    if new_path:
        for idx in range(len(new_path) - 1):
            x1, y1 = new_path[idx] % n, -np.floor(new_path[idx] / n)
            x2, y2 = new_path[idx + 1] % n, -np.floor(new_path[idx+1] / n)
            plt.plot([x1, x2], [y1, y2], c='green', linewidth=2, label="New Path" if idx == 0 else "")

    if obstacle is not None:
        x, y = obstacle % n, -np.floor(obstacle / n)
        plt.scatter(x, y, c='black', s=100, label='Obstacle')

    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    n = 6  # Grid size (6x6)
    graph = set_graph(n)

    # Select origin and destination
    origin, destination = 0, n * n - 1

    # Calculate initial optimal path
    dist, path = dijkstra(graph, origin, destination)

    # Plot grid with the initial path
    print(f"Initial Path: {path} with Distance: {dist}")
    plot_grid(n, path=path)

    # Add an obstacle along the path
    if len(path) > 2:
        obstacle_idx = 2  # Add obstacle on the 3rd vertex in the path
        obstacle = path[obstacle_idx]
        graph = update_graph_on_obstacle(graph, path[obstacle_idx - 1], path[obstacle_idx])

        # Recalculate path
        new_dist, new_path = dijkstra(graph, origin, destination)
        if new_dist == np.inf:
            print("No new path could be found due to the obstacle.")
        else:
            print(f"New Path: {new_path} with Distance: {new_dist}")

        # Plot updated grid
        plot_grid(n, path=path, new_path=new_path, obstacle=obstacle)
