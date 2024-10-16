import heapq

def best_first_search(starting_state):
    # TODO(III): You should copy your code from MP3 here
    visited_states = {starting_state: (None, 0)}

    # The frontier is a priority queue
    # You can pop from the queue using "heapq.heappop(frontier)"
    # You can push onto the queue using "heapq.heappush(frontier, state)"
    # NOTE: states are ordered because the __lt__ method of AbstractState is implemented
    frontier = []
    heapq.heappush(frontier, starting_state)
    
    #   - add new states to the frontier by calling state.get_neighbors()
    #   - check whether you've finished the search by calling state.is_goal()
    #       - then call backtrack(visited_states, state)...
    # Your code here ---------------
    while frontier:
        frontier_state = heapq.heappop(frontier)
        # print(frontier_state.state[0])
        # print(frontier_state.state[1])
        # print(frontier_state.state[2])
        if (frontier_state.is_goal()):
            return backtrack(visited_states, frontier_state)

        neighbors = frontier_state.get_neighbors()
        for neighbor in neighbors:
            new_dist = neighbor.dist_from_start 
            #+ neighbor.compute_heuristic() #visited_states[frontier_state][1]
            if (neighbor not in visited_states):
                visited_states[neighbor] = (frontier_state, new_dist)
                heapq.heappush(frontier, neighbor)
            elif (new_dist < visited_states[neighbor][1]): #if not visited or visited but new cost is less
                visited_states[neighbor] = (frontier_state, new_dist)
    # ------------------------------
    # if you do not find the goal return an empty list
    return []

def backtrack(visited_states, goal_state):
    # TODO(III): You should copy your code from MP3 here
    path = []
    curr_state = goal_state
    # Your code here ---------------
    while curr_state is not None:
        path.append(curr_state)
        curr_state = visited_states[curr_state][0] 
    # ------------------------------
    path = path[::-1]
    return path