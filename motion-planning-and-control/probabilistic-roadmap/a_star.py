import modern_robotics as mr
import numpy as np
import csv

# Get nodes and edges from csv files
nodes = np.loadtxt('./results/nodes.csv',delimiter=',')
edges = np.loadtxt('./results/edges.csv',delimiter=',')

# Function to find adjacent nodes and edge costs
def adj_nodes(edges, current_node):
    edges_id = edges[:,0:2]
    edges_cost = edges[:,2]
    
    adj_nodes = []
    costs = []

    # Find where current node has an edge
    row, col = np.where(edges_id == current_node)

    # Find adjacent nodes and associated costs
    for i in range(len(row)):
        if col[i] == 0:
            adj_nodes.append(int(edges_id[row[i]][1]))
            costs.append(edges_cost[row][i])
        if col[i] == 1:
            adj_nodes.append(int(edges_id[row[i]][0]))
            costs.append(edges_cost[row[i]])

    return adj_nodes, costs

# Function for A* Search
def a_star_search(nodes,edges):
    open_list = []
    closed_list = []

    # Start and goal nodes
    start_idx = 1
    current_node = start_idx
    goal_idx = 2
    # Add start node to open list
    open_list.append(start_idx)

    # Set up parent nodes, past cost, and heuristic cost
    parent_nodes = np.inf*np.ones((nodes.shape[0]))
    parent_nodes[start_idx-1] = 0
    past_cost = np.inf*np.ones((nodes.shape[0]))
    past_cost[start_idx-1] = 0
    h_cost = nodes[:,3]

    # Expand open nodes
    while open_list != []:
        
        if current_node == goal_idx:
            break

        # Find connected nodes and edges
        adj, costs = adj_nodes(edges,current_node)

        # Add new nodes to open list
        for i in range(len(adj)):
            if (adj[i] in open_list) == False:
                if (adj[i] in closed_list) == False:
                    open_list.append(adj[i])

            # Add current edge to cost from parent node
            parent_cost = past_cost[current_node-1]
            current_cost = parent_cost + costs[i]
            
            # If lower than previous cost, update
            if current_cost < past_cost[adj[i]-1]:
                parent_nodes[adj[i]-1] = current_node
                past_cost[adj[i]-1] = current_cost

        # Move current node to closed list
        open_list.remove(current_node)
        closed_list.append(current_node)
        
        tot_cost = past_cost + h_cost

        # Find lowest cost from open nodes
        open_costs = []
        for j in range(len(open_list)):
            open_costs.append(tot_cost[open_list[j]-1])
        # Choose new current node
        if len(open_list) < 1:
            break
        else:
            current_node = open_list[np.argmin(open_costs)]

    if current_node != goal_idx:
        print("No route found")
        return

    # Work backwards, get each parent node from goal to start
    route = []
    route_current = current_node
    route.append(route_current)
    while route_current != start_idx:
        route.insert(0, int(parent_nodes[route_current-1]))
        route_current = int(parent_nodes[route_current-1])
    
    # Create csv file in write mode
    f = open('./results/path.csv', 'w', newline='')
    writer = csv.writer(f)
    # Add route to csv
    writer.writerow(route)
    # Close the csv file
    f.close()

    return route

# Find shortest path
route = a_star_search(nodes,edges)