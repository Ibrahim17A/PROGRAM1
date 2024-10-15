import time  # Used to measure the execution time of each search algorithm
import csv  # Used for reading the coordinates from a CSV file
from collections import deque  # Deque (double-ended queue) is used for BFS
import math  # Used for math operations like radians and trigonometric functions
import heapq  # Used for priority queue implementation in Best-First and A* search

# Graph class that stores cities, their coordinates, and adjacency list
class Graph:
    def __init__(self):
        self.adjacency_list = {}  # Dictionary to store the adjacency list (connections between cities)
        self.coordinates = {}  # Dictionary to store coordinates (latitude, longitude) for each city

    # Add a city with its coordinates (latitude, longitude) to the graph
    def add_city(self, city, lat, long):
        self.coordinates[city] = (lat, long)  # Store city and its coordinates

    # Add an edge (connection) between two cities in the adjacency list
    def add_edge(self, city1, city2):
        if city1 not in self.adjacency_list:
            self.adjacency_list[city1] = []  # Initialize empty list if city1 not already in the adjacency list
        if city2 not in self.adjacency_list:
            self.adjacency_list[city2] = []  # Initialize empty list if city2 not already in the adjacency list
        self.adjacency_list[city1].append(city2)  # Add city2 as a neighbor of city1
        self.adjacency_list[city2].append(city1)  # Add city1 as a neighbor of city2 (bidirectional)

# Function to calculate the heuristic distance between two cities using the Haversine formula
# Haversine calculates the shortest distance over the Earth's surface
def haversine(coord1, coord2):
    lat1, lon1 = coord1  # Unpack the coordinates of the first city
    lat2, lon2 = coord2  # Unpack the coordinates of the second city
    R = 6371  # Radius of the Earth in kilometers
    dlat = math.radians(lat2 - lat1)  # Difference in latitudes converted to radians
    dlon = math.radians(lon2 - lon1)  # Difference in longitudes converted to radians
    # Haversine formula
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c  # Return the distance between the two cities in kilometers

# Function to load city coordinates from a CSV file
def load_coordinates(graph, filename):
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            city = row[0].strip()  # City name
            lat = float(row[1].strip())  # Latitude
            lon = float(row[2].strip())  # Longitude
            graph.add_city(city, lat, lon)  # Add the city and its coordinates to the graph

# Function to load the adjacency list from a text file
def load_adjacency_list(graph, filename):
    with open(filename, 'r') as file:
        for line in file:
            cities = line.strip().split()  # Split each line by spaces
            city1 = cities[0].strip()  # The first city in the line
            for city2 in cities[1:]:  # The rest are cities connected to city1
                graph.add_edge(city1, city2)  # Add edges (connections) between city1 and each city2

# 1. Breadth-First Search (BFS)
# A simple and uninformed search algorithm that explores all the neighbors at the present depth level before moving on to nodes at the next depth level
def bfs(graph, start, goal):
    visited = set()  # Keep track of visited cities
    queue = deque([(start, [start])])  # Queue to manage the frontier (cities to explore), storing (city, path) pairs

    while queue:
        city, path = queue.popleft()  # Dequeue a city and its path from the frontier
        if city == goal:  # Goal check
            return path  # Return the path to the goal if found
        
        for neighbor in graph.adjacency_list[city]:  # Explore neighbors of the current city
            if neighbor not in visited:  # Only explore unvisited cities
                visited.add(neighbor)  # Mark city as visited
                queue.append((neighbor, path + [neighbor]))  # Enqueue the neighbor with the updated path
    
    return None  # Return None if no path found

# 2. Depth-First Search (DFS)
# An uninformed search that explores as deep as possible along each branch before backtracking
def dfs(graph, start, goal, path=None, visited=None):
    if path is None:
        path = [start]  # Initialize the path with the starting city
    if visited is None:
        visited = set()  # Initialize the visited set
    
    visited.add(start)  # Mark the current city as visited
    
    if start == goal:  # Goal check
        return path  # Return the path if the goal is found
    
    for neighbor in graph.adjacency_list[start]:  # Explore neighbors of the current city
        if neighbor not in visited:  # Only explore unvisited cities
            new_path = dfs(graph, neighbor, goal, path + [neighbor], visited)  # Recursively explore the neighbor
            if new_path:  # If a valid path is found, return it
                return new_path
    
    return None  # Return None if no path found

# 3. Iterative Deepening Depth-First Search (IDDFS)
# A search algorithm that combines DFS with iterative deepening to avoid DFS's memory limitations
def iddfs(graph, start, goal, max_depth=10):
    for depth in range(max_depth):  # Try different depth limits
        result = dls(graph, start, goal, depth)  # Use depth-limited search (DLS) at each depth level
        if result:
            return result  # Return the path if found
    return None  # Return None if no path found

# Depth-Limited Search (DLS) used by IDDFS
def dls(graph, start, goal, depth):
    return dfs_limited(graph, start, goal, [], set(), depth)

# Limited version of DFS with a depth limit
def dfs_limited(graph, current, goal, path, visited, depth):
    if depth == 0 and current == goal:  # Goal check if depth limit is reached
        return path + [goal]
    if depth > 0:  # Continue exploring if depth limit not reached
        visited.add(current)
        for neighbor in graph.adjacency_list[current]:
            if neighbor not in visited:
                new_path = dfs_limited(graph, neighbor, goal, path + [current], visited, depth - 1)
                if new_path:
                    return new_path
    return None

# 4. Best-First Search
# An informed search algorithm that uses a heuristic (here, Haversine distance) to guide the search toward the goal
def best_first_search(graph, start, goal):
    pq = []  # Priority queue to store cities based on heuristic (h-cost) values
    heapq.heappush(pq, (0, start, [start]))  # Push the start city into the queue with a heuristic value of 0
    visited = set()

    while pq:
        _, city, path = heapq.heappop(pq)  # Pop the city with the lowest heuristic value

        if city == goal:  # Goal check
            return path

        if city not in visited:
            visited.add(city)  # Mark city as visited
            for neighbor in graph.adjacency_list[city]:
                if neighbor not in visited:
                    heuristic = haversine(graph.coordinates[neighbor], graph.coordinates[goal])  # Calculate heuristic
                    heapq.heappush(pq, (heuristic, neighbor, path + [neighbor]))  # Add neighbor to the queue

    return None

# 5. A* Search
# An informed search that combines the actual distance (g-cost) and heuristic (h-cost) to guide the search
def a_star_search(graph, start, goal):
    pq = []  # Priority queue to store cities based on f-cost (g-cost + h-cost)
    heapq.heappush(pq, (0, 0, start, [start]))  # Push the start city with g-cost 0 and f-cost 0
    g_costs = {start: 0}  # Dictionary to store the best-known g-cost for each city
    visited = set()

    while pq:
        f_cost, g_cost, city, path = heapq.heappop(pq)  # Pop the city with the lowest f-cost

        if city == goal:  # Goal check
            return path

        if city not in visited:
            visited.add(city)  # Mark city as visited
            for neighbor in graph.adjacency_list[city]:
                tentative_g_cost = g_cost + haversine(graph.coordinates[city], graph.coordinates[neighbor])  # Calculate g-cost
                if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g_cost  # Update g-cost for the neighbor
                    f_cost = tentative_g_cost + haversine(graph.coordinates[neighbor], graph.coordinates[goal])  # Calculate f-cost
                    heapq.heappush(pq, (f_cost, tentative_g_cost, neighbor, path + [neighbor]))  # Add neighbor to the queue

    return None

# Function to calculate the total distance of a path using the Haversine formula
def calculate_total_distance(graph, path):
    total_distance = 0
    for i in range(len(path) - 1):
        total_distance += haversine(graph.coordinates[path[i]], graph.coordinates[path[i + 1]])  # Calculate distance between consecutive cities
    return total_distance

# Function to run a specific search method and measure its execution time
def run_search(graph, method, start, goal):
    print(f"\nRunning {method.__name__} from {start} to {goal}...")  # Print the method being run
    start_time = time.perf_counter()  # Record the start time
    path = method(graph, start, goal)  # Run the search method
    end_time = time.perf_counter()  # Record the end time
    if path:
        print(f"Path found: {' -> '.join(path)}")  # Print the path if found
        total_distance = calculate_total_distance(graph, path)  # Calculate the total distance
        print(f"Total distance: {total_distance:.2f} km")  # Print the total distance
    else:
        print("No path found")  # Print if no path is found
    print(f"Time taken: {end_time - start_time:.6f} seconds")  # Print the time taken
    return path

# Example Usage
if __name__ == "__main__":
    graph = Graph()  # Create a graph instance
    
    # Load data from files (coordinates and adjacencies)
    coordinates_file = "coordinates.csv"
    adjacency_file = "adjacencies.txt"
    
    load_coordinates(graph, coordinates_file)  # Load city coordinates
    load_adjacency_list(graph, adjacency_file)  # Load adjacency list (connections)
    import random
    start_city = random.choice(list(graph.coordinates.keys()))  # Starting city
    end_city = random.choice(list(graph.coordinates.keys()))  # Destination city

    # Run each search method and compare results
    run_search(graph, bfs, start_city, end_city)
    run_search(graph, dfs, start_city, end_city)
    run_search(graph, iddfs, start_city, end_city)
    run_search(graph, best_first_search, start_city, end_city)
    run_search(graph, a_star_search, start_city, end_city)