import numpy as np
import heapq  # For priority queue in Dijkstra's algorithm
import matplotlib.pyplot as plt
#doesn't work needs graph input paramsn and the few fixes that come with
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

def path_node_to_turn_translation(node_list,graph_dimensions, initial_direction="forward"):
    #The convention taken is that the robot is always facing towards the north, whatever his original starting and ending nodes could be
    translated_node_list = []
    previous_direction = initial_direction
    #We will use wanted heading and previous direction in order to get 
    heading_direction_conversion_directory= {
              #left                right                forward                  backward
    #east
    "east":{  "left":"backward",   "right":"forward",   "forward":"right",        "backward":"left"     },                   
    #west
    "west":{  "left":"forward",   "right":"backward",    "forward":"left",         "backward":"right"    },  
    #north
    "north":{ "left":"left",       "right":"right",      "forward":"forward",      "backward":"backwards"},  
    #south
    "south":{ "left":"right",      "right":"left",       "forward":"backward",     "backward":"forward"  }
    }
    n,m=graph_dimensions
    print(node_list)
    for i in range(len(node_list)-1):
        current_node = node_list[i]
        next_node = node_list[i+1]
        #print("a", n, m, current_node, next_node)
        if  current_node - next_node == 1:
            heading = "west"
        elif current_node - next_node == -1:
            heading = "east"
        elif current_node - next_node == -n:
            heading = "north"
        elif current_node - next_node == n:
            heading = "south"
        else:
            return ValueError("You serve zero purpose!")
        translated_node_list.append(heading_direction_conversion_directory[heading][previous_direction])
        previous_direction = translated_node_list[-1]
    
    
    return translated_node_list

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
        #Starting point
        starting_node = path[0]
        x, y  = starting_node % n, -np.floor(starting_node / n)
        plt.scatter(x, y, c='red', s=100, label='Starting Point')
        #Destination
        ending_node = path[-1]
        x, y  = ending_node % n, -np.floor(ending_node / n)
        plt.scatter(x, y, c='gree', s=100, label='Destination')
        for idx in range(len(path) - 1):
            x1, y1 = path[idx] % n, -np.floor(path[idx] / n)
            x2, y2 = path[idx + 1] % n, -np.floor(path[idx+1] / n)
            plt.plot([x1, x2], [y1, y2], c='orange', linewidth=2, label="Old Path" if idx == 0 else "")
      

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
    
def plot_current_position_on_grid(grid_dimensions, current_node=None, paths=None, obstacles=None):
    #Needs to be fixed
    """
    Visualizes the grid with optional paths and obstacles.
    """
    #Current path
    n = grid_dimensions[1]
    m = grid_dimensions[0] 
    path = paths[-1]
    plt.figure(figsize=(8, 8))
    for i in range(n):
        for j in range(m):
            node = i * n + j
            plt.scatter(j, -i, c='blue')
            plt.text(j, -i, f'{node}', fontsize=8, ha='center', va='center')
   
   #Initial green color hex value
    current_green_color = '#32CD32'

    #Initial red color hex value  
    current_red_color = '#FF0000'
       
    if path:
        #Starting point
        starting_node = path[0]
        x, y  = starting_node % n, -np.floor(starting_node / m)
        plt.scatter(x, y, c='red', s=100, label='Starting Point')
        #Destination
        ending_node = path[-1]
        x, y  = ending_node % n, -np.floor(ending_node / m)
        plt.scatter(x, y, c='green', s=100, label='Destination')
        for idx in range(len(path) - 1):
            x1, y1 = path[idx] % n, -np.floor(path[idx] / m)
            x2, y2 = path[idx + 1] % n, -np.floor(path[idx+1] / m)
            plt.plot([x1, x2], [y1, y2], c='red', linewidth=2, label="Old Path" if idx == 0 else "")
   
    if current_node is not None:
        x, y = current_node % n, -np.floor(current_node / m)
        plt.scatter(x, y, c='blue', s=100, label='Current_Robot_Position')

    old_paths = paths[:len(paths)-1]
    
    for i in range(len(old_paths)):
        lighten_color()
        new_path = old_paths[i]
        obstacle = obstacle[i]
        current_red_color = lighten_color(current_red_color)

        for idx in range(len(new_path) - 1):
            x1, y1 = new_path[idx] % n, -np.floor(new_path[idx] / m)
            x2, y2 = new_path[idx + 1] % n, -np.floor(new_path[idx+1] / m)
            plt.plot([x1, x2], [y1, y2], c= current_red_color, linewidth=2, label="New_Path_{i}" if idx == 0 else "")

        if obstacle is not None:
            x, y = obstacle % n, -np.floor(obstacle / m)
            plt.scatter(x, y, c='black', s=100, label='Obstacle_{i}')

    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.show()

def lighten_color(hex_color, amount=0.1):
    import matplotlib.colors as mcolors
    # Convert hex to RGB 
    rgb = mcolors.hex2color(hex_color)
    # Lighten the color
    lighter_rgb = [min(1, c + amount) for c in rgb]
    # Convert back to hex 
    lighter_hex = mcolors.to_hex(lighter_rgb)
    return lighter_hex

def setup(n, m, start=0, end=None, initial_direction = "forward" ):
    if end == None:
        end = n*m-1
    graph = set_graph(n, m) 
    _ ,node_path = dijkstra(graph, start, end)
    turning_path = path_node_to_turn_translation(node_path,[n,m], initial_direction )
    return graph, node_path, turning_path

def obstacle_detect_behavior(n,m,graph,node_path,translated_node_list,last_node_reached_idx,last_node_targeted_idx):
    last_direction = translated_node_list[last_node_reached_idx] # maybe fix indexing
    #We do a 180 turn
    if last_direction == "forward":
        last_direction_inverted = "backward"
    elif  last_direction == "backward":
        last_direction_inverted = "forward"
    elif  last_direction == "left":
        last_direction_inverted = "right" 
    elif  last_direction == "right":
        last_direction_inverted = "left"    

    node_number = last_node_reached_idx
    #i,j = node_number % n, int(np.floor(node_number / m)) 
    updated_graph = update_graph_on_obstacle(graph, last_node_reached_idx, last_node_targeted_idx)
    _ ,updated_node_path = dijkstra(updated_graph, last_node_reached_idx, node_path[-1])
    updated_turning_path = path_node_to_turn_translation(updated_node_path,[n,m], last_direction_inverted)
    return  updated_graph,updated_node_path,updated_turning_path #not sure about these outputs maybe need more, needs a pressure test

if __name__ == "__main__":
    # n = 6  # Grid size (6x6)
    # graph = set_graph(n)


    graph_dimensions = [2, 3]
    m, n = graph_dimensions
    graph, node_path, turning_path = setup(graph_dimensions, 3, 2, "forward" )
        # turning_path is a list of intersection directions
    plot_current_position_on_grid(graph_dimensions, 0, [[0]], [[0]])

    # # Select origin and destination
    # origin, destination = 0, n * n - 1

    # # Calculate initial optimal path
    # dist, path = dijkstra(graph, origin, destination)

    # # Plot grid with the initial path
    # print(f"Initial Path: {path} with Distance: {dist}")
    # plot_grid(n, path=path)

    # # Add an obstacle along the path
    # if len(path) > 2:
    #     obstacle_idx = 2  # Add obstacle on the 3rd vertex in the path
    #     obstacle = path[obstacle_idx]
    #     graph = update_graph_on_obstacle(graph, path[obstacle_idx - 1], path[obstacle_idx])

    #     # Recalculate path
    #     new_dist, new_path = dijkstra(graph, origin, destination)
    #     if new_dist == np.inf:
    #         print("No new path could be found due to the obstacle.")
    #     else:
    #         print(f"New Path: {new_path} with Distance: {new_dist}")

    #     # Plot updated grid
    #     plot_grid(n, path=path, new_path=new_path, obstacle=obstacle)
