import networkx as nx
import matplotlib.pyplot as plt
import heapq

def heuristic(a, b):
    """
    Calculates the Manhattan distance between two points
    """
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def a_star_algorithm(graph, start, goal):
    """
    A* Pathfinding Algorithm
    """
    # Initialize both open and closed list
    open_list = []
    closed_list = set()

    # Heapify the open_list and add the start node
    heapq.heapify(open_list) 
    heapq.heappush(open_list, (0, start))
    
    # Start node has f, g, and h values of 0
    g = {start: 0}
    parents = {start: start}
    
    while len(open_list) > 0:
        # Get the current node
        current = heapq.heappop(open_list)[1]
        closed_list.add(current)

        # Found the goal
        if current == goal:
            path = []
            while parents[current] != current:
                path.append(current)
                current = parents[current]
            path.append(start)
            return path[::-1]

        # Generate children
        for neighbor in graph.neighbors(current):
            if neighbor in closed_list:
                continue
            # Create the f, g, and h values
            g_score = g[current] + 1
            f_score = g_score + heuristic(neighbor, goal)

            # Child is already in the open list
            if any(neighbor in pair for pair in open_list):
                if g_score >= g.get(neighbor, 0):
                    continue
            
            # Add the child to the open list
            g[neighbor] = g_score
            parents[neighbor] = current
            heapq.heappush(open_list, (f_score, neighbor))
    
    return []

# Create a graph
graph = nx.grid_2d_graph(5, 5)  # Creates a 5x5 grid graph

# Define start and end points
start, goal = (0, 0), (4, 4)

# Call your A* algorithm
path = a_star_algorithm(graph, start, goal)

# Visualization
pos = dict((n, n) for n in graph.nodes()) # Dictionary of node positions
nx.draw(graph, pos, with_labels=True, node_color='lightblue')

# Draw the path
path_edges = list(zip(path, path[1:]))
nx.draw_networkx_nodes(graph, pos, nodelist=path, node_color='green')
nx.draw_networkx_edges(graph, pos, edgelist=path_edges, edge_color='red')

plt.show()
