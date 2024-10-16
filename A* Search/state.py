from utils import is_english_word, levenshteinDistance
from abc import ABC, abstractmethod
import copy # you may want to use copy when creating neighbors for EightPuzzle...

# NOTE: using this global index means that if we solve multiple 
#       searches consecutively the index doesn't reset to 0...
from itertools import count
global_index = count()

# TODO(III): You should read through this abstract class
#           your search implementation must work with this API,
#           namely your search will need to call is_goal() and get_neighbors()
class AbstractState(ABC):
    def __init__(self, state, goal, dist_from_start=0, use_heuristic=True):
        self.state = state
        self.goal = goal
        # we tiebreak based on the order that the state was created/found
        self.tiebreak_idx = next(global_index)
        # dist_from_start is classically called "g" when describing A*, i.e., f(state) = g(start, state) + h(state, goal)
        self.dist_from_start = dist_from_start
        self.use_heuristic = use_heuristic
        if use_heuristic:
            self.h = self.compute_heuristic()
        else:
            self.h = 0

    # To search a space we will iteratively call self.get_neighbors()
    # Return a list of AbstractState objects
    @abstractmethod
    def get_neighbors(self):
        pass
    
    # Return True if the state is the goal
    @abstractmethod
    def is_goal(self):
        pass
    
    # A* requires we compute a heuristic from each state
    # compute_heuristic should depend on self.state and self.goal
    # Return a float
    @abstractmethod
    def compute_heuristic(self):
        pass
    
    # The "less than" method ensures that states are comparable, meaning we can place them in a priority queue
    # You should compare states based on f = g + h = self.dist_from_start + self.h
    # Return True if self is less than other
    @abstractmethod
    def __lt__(self, other):
        # NOTE: if the two states (self and other) have the same f value, tiebreak using tiebreak_idx as below
        if self.tiebreak_idx < other.tiebreak_idx:
            return True

    # The "hash" method allow us to keep track of which states have been visited before in a dictionary
    # You should hash states based on self.state (and sometimes self.goal, if it can change)
    # Return a float
    @abstractmethod
    def __hash__(self):
        pass
    # __eq__ gets called during hashing collisions, without it Python checks object equality
    @abstractmethod
    def __eq__(self, other):
        pass
    
# WordLadder ------------------------------------------------------------------------------------------------

# TODO(III): we've provided you most of WordLadderState, read through our comments and code below.
#           The only thing you must do is fill in the WordLadderState.__lt__(self, other) method
class WordLadderState(AbstractState):
    def __init__(self, state, goal, dist_from_start, use_heuristic, cost_per_letter):
        '''
        state: string of length n
        goal: string of length n
        dist_from_start: integer
        use_heuristic: boolean
        '''
        # NOTE: AbstractState constructor does not take cost_per_letter
        super().__init__(state, goal, dist_from_start, use_heuristic)
        self.cost_per_letter = cost_per_letter
        
    # Each word can have many neighbors:
    #   Every letter in the word (self.state) can be replaced by every letter in the alphabet
    #   The resulting word must be a valid English word (i.e., in our dictionary)
    def get_neighbors(self):
        '''
        Return: a list of WordLadderState
        '''
        nbr_states = []
        for word_idx in range(len(self.state)):
            prefix = self.state[:word_idx]
            suffix = self.state[word_idx+1:]
            # 'a' = 97, 'z' = 97 + 25 = 122
            for c_idx in range(97, 97+26):
                c = chr(c_idx) # convert index to character
                # Replace the character at word_idx with c
                potential_nbr = prefix + c + suffix
                edge_cost = self.cost_per_letter[c]
                # If the resulting word is a valid english word, add it as a neighbor
                # NOTE: dist_from_start increases by edge_cost (this may not be 1!)
                if is_english_word(potential_nbr):
                    new_state = WordLadderState(
                        state=potential_nbr,
                        goal=self.goal, # stays the same!
                        dist_from_start=self.dist_from_start + edge_cost,
                        use_heuristic=self.use_heuristic, # stays the same!
                        cost_per_letter=self.cost_per_letter # stays the same!
                    )
                    nbr_states.append(new_state)
        return nbr_states

    # Checks if we reached the goal word with a simple string equality check
    def is_goal(self):
        return self.state == self.goal
    
    # Strings are hashable, directly hash self.state
    def __hash__(self):
        return hash(self.state)
    def __eq__(self, other):
        return self.state == other.state
    
    # The heuristic we use is the edit distance (Levenshtein) between our current word and the goal word
    def compute_heuristic(self):
        return levenshteinDistance(self.state, self.goal)
    
    # TODO(III): implement this method
    def __lt__(self, other):    
        # You should return True if the current state has a lower g + h value than "other"
        # If they have the same value then you should use tiebreak_idx to decide which is smaller
        curr_value = self.dist_from_start + self.h
        other_value = other.dist_from_start + other.h

        if (curr_value < other_value):
            return True
        elif (curr_value == other_value):
            return self.tiebreak_idx < other.tiebreak_idx
        else:
            return False
    
    # str and repr just make output more readable when you print out states
    def __str__(self):
        return self.state
    def __repr__(self):
        return self.state

# EightPuzzle ------------------------------------------------------------------------------------------------

# TODO(IV): implement this method (also need it for the next homework)
# Manhattan distance between two points (a=(a1,a2), b=(b1,b2))
# difference in their x coordinates plus the difference in their y coordinates
def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

class EightPuzzleState(AbstractState):
    def __init__(self, state, goal, dist_from_start, use_heuristic, zero_loc):
        '''
        state: 3x3 array of integers 0-8
        goal: 3x3 goal array, default is np.arange(9).reshape(3,3).tolist()
        zero_loc: an additional helper argument indicating the 2d index of 0 in state, you do not have to use it
        '''
        # NOTE: AbstractState constructor does not take zero_loc
        super().__init__(state, goal, dist_from_start, use_heuristic)
        self.zero_loc = zero_loc
    
    # TODO(IV): implement this method
    def get_neighbors(self):
        '''
        Return: a list of EightPuzzleState
        '''
        nbr_states = []
        # NOTE: There are *up to 4* possible neighbors and the order you add them matters for tiebreaking
        #   Please add them in the following order: [below, left, above, right], where for example "below" 
        #   corresponds to moving the empty tile down (moving the tile below the empty tile up)
        x = self.zero_loc[0]
        y = self.zero_loc[1]

        if (self.zero_loc[0] != 2): #not on bottom edge
            new_state = EightPuzzleState(
                state = copy.deepcopy(self.state),
                goal=self.goal, # stays the same!
                dist_from_start=self.dist_from_start + 1,
                use_heuristic=self.use_heuristic, # stays the same!
                zero_loc = [x+1,y] # 0 moves to bottom
            )
            #swap elements
            new_state.state[x][y], new_state.state[x+1][y] = new_state.state[x+1][y], new_state.state[x][y]
            nbr_states.append(new_state)
            # print(f"bottom: {new_state.zero_loc}")
            # print(new_state.state[0])
            # print(new_state.state[1])
            # print(new_state.state[2])

        if (self.zero_loc[1] != 0): #not on left edge
            new_state = EightPuzzleState(
                state = copy.deepcopy(self.state),
                goal=self.goal, # stays the same!
                dist_from_start=self.dist_from_start + 1,
                use_heuristic=self.use_heuristic, # stays the same!
                zero_loc = [x,y-1] # 0 moves to right
            )
            #swap elements
            new_state.state[x][y], new_state.state[x][y-1] = new_state.state[x][y-1], new_state.state[x][y]
            nbr_states.append(new_state)
            # print(f"left: {new_state.zero_loc}")
            # print(new_state.state[0])
            # print(new_state.state[1])
            # print(new_state.state[2])

        if (self.zero_loc[0] != 0): #not on top edge
            new_state = EightPuzzleState(
                state = copy.deepcopy(self.state),
                goal=self.goal, # stays the same!
                dist_from_start=self.dist_from_start + 1,
                use_heuristic=self.use_heuristic, # stays the same!
                zero_loc = [x-1,y] # 0 moves to right
            )
            #swap elements
            new_state.state[x][y], new_state.state[x-1][y] = new_state.state[x-1][y], new_state.state[x][y]
            nbr_states.append(new_state)
            # print(f"top: {new_state.zero_loc}")
            # print(new_state.state[0])
            # print(new_state.state[1])
            # print(new_state.state[2])

        if (self.zero_loc[1] != 2): #not on right edge
            new_state = EightPuzzleState(
                state = copy.deepcopy(self.state),
                goal=self.goal, # stays the same!
                dist_from_start=self.dist_from_start + 1,
                use_heuristic=self.use_heuristic, # stays the same!
                zero_loc = [x,y + 1] # 0 moves to right
            )
            #swap elements
            new_state.state[x][y], new_state.state[x][y + 1] = new_state.state[x][y + 1], new_state.state[x][y]
            nbr_states.append(new_state)
            # print(f"right: {new_state.zero_loc}")
            # print(new_state.state[0])
            # print(new_state.state[1])
            # print(new_state.state[2])

        return nbr_states

    # Checks if goal has been reached
    def is_goal(self):
        # In python "==" performs deep list equality checking, so this works as desired
        return self.state == self.goal
    
    # Can't hash a list, so first flatten the 2d array and then turn into tuple
    def __hash__(self):
        return hash(tuple([item for sublist in self.state for item in sublist]))
    def __eq__(self, other):
        return self.state == other.state
    
    # TODO(IV): implement this method
    def compute_heuristic(self):
        total = 0
        # NOTE: There is more than one possible heuristic, 
        #       please implement the Manhattan heuristic, as described in the MP instructions
        goal_locs = {1: (0,1), 2: (0,2), 3: (1,0), 4: (1,1), 5: (1,2), 6: (2,0), 7: (2,1), 8: (2,2)}
        for i in range(3):
            for j in range(3):
                if self.state[i][j] != 0:
                    pos = goal_locs[self.state[i][j]]
                    total += manhattan((i,j), pos)

        return total
    
    # TODO(IV): implement this method
    # Hint: it should be identical to what you wrote in WordLadder.__lt__(self, other)
    def __lt__(self, other):
        curr_value = self.dist_from_start + self.h
        other_value = other.dist_from_start + other.h

        if (curr_value < other_value):
            return True
        elif (curr_value == other_value):
            return self.tiebreak_idx < other.tiebreak_idx
        else:
            return False
    
    # str and repr just make output more readable when you print out states
    def __str__(self):
        return self.state
    def __repr__(self):
        return "\n---\n"+"\n".join([" ".join([str(r) for r in c]) for c in self.state])
    