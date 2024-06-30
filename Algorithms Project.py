import heapq
from collections import defaultdict
import math

# Heuristic function to estimate the cost from a node to the end node
def heuristic(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)  # Euclidean distance

# A* search algorithm to find the shortest path from start to end
def a_star_search(graph, start, end, delivery_windows):
    open_set = []  # Priority queue to store the nodes to be evaluated
    heapq.heappush(open_set, (0, start))  # Push the start node with an initial cost of 0
    came_from = {}  # Dictionary to store the optimal path
    g_score = {node: float('inf') for node in graph}  # Cost from start to each node
    g_score[start] = 0  # Cost to reach the start node is 0
    f_score = {node: float('inf') for node in graph}  # Estimated cost from start to end through each node
    f_score[start] = heuristic(start, end)  # Heuristic cost from start to end

    while open_set:  # Loop until there are no nodes left to evaluate
        current = heapq.heappop(open_set)[1]  # Get the node with the lowest f_score

        if current == end:  # If the end node is reached
            return reconstruct_path(came_from, current)  # Reconstruct and return the path

        for neighbor, weight in graph[current]:  # For each neighbor of the current node
            tentative_g_score = g_score[current] + weight  # Calculate the cost to reach the neighbor
            if tentative_g_score < g_score[neighbor]:  # If the new cost is lower
                came_from[neighbor] = current  # Update the optimal path
                g_score[neighbor] = tentative_g_score  # Update the cost to reach the neighbor
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)  # Update the estimated cost to reach the end
                if neighbor not in [i[1] for i in open_set]:  # If the neighbor is not in the open set
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))  # Add the neighbor to the open set

    return None  # Return None if no path is found

# Function to reconstruct the path from the start to the end
def reconstruct_path(came_from, current):
    total_path = [current]  # Start with the end node
    while current in came_from:  # Trace back to the start node
        current = came_from[current]  # Move to the previous node
        total_path.append(current)  # Add the node to the path
    total_path.reverse()  # Reverse the path to get the correct order
    return total_path  # Return the reconstructed path

# Example graph with node coordinates and edges with weights
graph = {
    (0, 0): [((1, 1), 1.5), ((2, 2), 2.5)],  # Edges from (0, 0) to (1, 1) and (2, 2) with weights 1.5 and 2.5
    (1, 1): [((2, 2), 1.0), ((3, 3), 2.2)],  # Edges from (1, 1) to (2, 2) and (3, 3) with weights 1.0 and 2.2
    (2, 2): [((3, 3), 1.5)],  # Edge from (2, 2) to (3, 3) with weight 1.5
    (3, 3): []  # No outgoing edges from (3, 3)
}

# Delivery windows (for simplicity, assume all deliveries need to be completed within a certain time frame)
delivery_windows = {
    (0, 0): (0, 10),  # Delivery window for (0, 0)
    (1, 1): (1, 9),  # Delivery window for (1, 1)
    (2, 2): (2, 8),  # Delivery window for (2, 2)
    (3, 3): (3, 7)   # Delivery window for (3, 3)
}

start = (0, 0)  # Start node
end = (3, 3)  # End node

# Find the optimized route using the A* search algorithm
route = a_star_search(graph, start, end, delivery_windows)
print("Optimized delivery route:", route)  # Print the optimized delivery route
