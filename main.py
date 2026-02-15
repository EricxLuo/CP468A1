import heapq

# Weights for the path cost calculation
W_DRIVE_TIME = 1.0   
W_PARKING_COST = 2.0 
W_WALK_TIME = 1.5    

class DynamicParkingAgent:
    # 1. Added max_walk_dist with a default of infinity so it won't break if omitted
    def __init__(self, road_graph, parking_lots, destination, node_coords, max_walk_dist=float('inf')):
        self.road_graph = road_graph       
        self.parking_lots = parking_lots   
        self.destination = destination     
        self.node_coords = node_coords     
        self.max_walk_dist = max_walk_dist # Store the distance constraint

    def heuristic(self, current_node, mode):
        # Distance estimate
        # Returns 0 if already parked or walking since the drive is over
        if mode in ['WALKING', 'PARKED']:
            return 0 
            
        curr_x, curr_y = self.node_coords[current_node]
        dest_x, dest_y = self.node_coords[self.destination]
        dist = ((curr_x - dest_x)**2 + (curr_y - dest_y)**2)**0.5
        
        return dist * W_DRIVE_TIME

    def get_walk_time(self, parking_node):
        p_x, p_y = self.node_coords[parking_node]
        d_x, d_y = self.node_coords[self.destination]
        dist = ((p_x - d_x)**2 + (p_y - d_y)**2)**0.5
        return dist * 2 

    def solve(self, start_node):
        # A* Priority Queue Setup
        # Stores: (f_cost, g_cost, current_node, mode, chosen_lot, path_history)
        pq = [(0, 0, start_node, 'DRIVING', None, [f"Start at {start_node}"])]
        visited = set()
        
        print(f"Starting Search from {start_node} to {self.destination}...\n")
        print(f"Max Walking Distance Enforced: {self.max_walk_dist} units\n")

        while pq:
            # Pop the current best state
            state = heapq.heappop(pq)
            f_score, g_score, current_v, mode, p_star, path = state

            state_key = (current_v, mode, p_star)
            if state_key in visited:
                continue
            visited.add(state_key)

            # Goal Test
            # Checks if we are at the destination and actually picked a parking spot
            if current_v == self.destination and p_star is not None:
                print("Goal Reached!")
                print("Path taken:")
                for step in path:
                    print(f" -> {step}")
                print(f"Total Penalty: {g_score:.1f}\n")
                return path, g_score

            # State Transitions
            if mode == 'DRIVING':
                # Action 1: Keep driving to neighbors
                for neighbor, t_to_drive in self.road_graph.get(current_v, {}).items():
                    new_g = g_score + (t_to_drive * W_DRIVE_TIME)
                    f_new = new_g + self.heuristic(neighbor, 'DRIVING')
                    
                    heapq.heappush(pq, (f_new, new_g, neighbor, 'DRIVING', None, 
                                        path + [f"Drive to {neighbor}"]))
                
                # Action 2: Park the car
                if current_v in self.parking_lots:
                    lot_info = self.parking_lots[current_v]
                    
                    # 2. Calculate actual walking distance to destination
                    p_x, p_y = self.node_coords[current_v]
                    d_x, d_y = self.node_coords[self.destination]
                    walk_dist = ((p_x - d_x)**2 + (p_y - d_y)**2)**0.5
                    
                    # 3. Only allow parking if it has space AND is within walking limits
                    if lot_info['capacity'] > 0 and walk_dist <= self.max_walk_dist: 
                        new_g = g_score + (lot_info['price'] * W_PARKING_COST)
                        f_new = new_g + self.heuristic(current_v, 'PARKED')
                        
                        heapq.heappush(pq, (f_new, new_g, current_v, 'PARKED', current_v, 
                                            path + [f"Park at {current_v} (${lot_info['price']})"]))

            elif mode == 'PARKED':
                # Action 3: Walk to destination
                t_to_walk = self.get_walk_time(current_v)
                new_g = g_score + (t_to_walk * W_WALK_TIME)
                
                heapq.heappush(pq, (new_g, new_g, self.destination, 'WALKING', p_star, 
                                    path + [f"Walk to {self.destination} ({t_to_walk:.1f} mins)"]))

        print("No valid route found.")
        return None

# Test Inputs
urban_map = {
    'Start': {'A': 5, 'B': 8},
    'A': {'Start': 5, 'C': 4, 'D': 7},
    'B': {'Start': 8, 'D': 3},
    'C': {'A': 4, 'Park1': 2},      
    'D': {'A': 7, 'B': 3, 'Park2': 2}, 
    'Park1': {'C': 2},
    'Park2': {'D': 2}
}

coordinates = {
    'Start': (0, 0),
    'A': (2, 2), 'B': (2, -1),
    'C': (4, 4), 'D': (4, 0),
    'Park1': (5, 5),    
    'Park2': (5, 1),    
    'Destination': (6, 6) 
}

parking_data = {
    'Park1': {'price': 20, 'capacity': 10},
    'Park2': {'price': 5, 'capacity': 5}
}

# Execution with a 3.0 distance limit
# Park2 is ~5.1 units away from destination, so it will be rejected despite being cheaper!
# Park1 is ~1.4 units away, so it will be accepted.
agent = DynamicParkingAgent(urban_map, parking_data, 'Destination', coordinates, max_walk_dist=3.0)
agent.solve('Start')