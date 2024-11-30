import heapq
import random

class Node:
    def __init__(self, position=None, parent=None, g=0, h=0):
        self.position = position
        self.parent = parent
        self.g = g
        self.h = h
        self.constraints = []

    def f(self):
        return self.g + self.h
    
    def __lt__(self, other):
        return self.f() < other.f()

    def __eq__(self, other):
        return self.f() == other.f()
    

def astar(start, goal, grid):
    
    def heuristic(node, goal):
        return abs(node.position[0] - goal[0]) + abs(node.position[1] - goal[1])

    open_set = []
    heapq.heappush(open_set, (0, Node(start)))
    visited = set()

    while open_set:
        _, current_node = heapq.heappop(open_set)

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return list(reversed(path))


        visited.add(current_node.position)

        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_x, new_y = current_node.position[0] + dx, current_node.position[1] + dy
            if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and grid[new_x][new_y] == 0 and (new_x, new_y) not in visited:
                new_position = (new_x, new_y)
                new_node = Node(new_position, current_node, current_node.g + 1, heuristic(Node(new_position), goal))
                heapq.heappush(open_set, (new_node.f(), new_node))

    return None

def SIC(solution):
    return sum(len(path) for path in solution.values())

def conflicts(paths):
    conflicts_list = []
    for agent1, path1 in paths.items():
        for agent2, path2 in paths.items():
            if agent1 != agent2:
                for timestep, (pos1, pos2) in enumerate(zip(path1, path2)):
                    if pos1 == pos2:
                        conflicts_list.append((agent1, agent2, pos1, timestep))
    return conflicts_list

def find_individual_paths(mapf_instance):
    paths = {}
    for agent, (start, goal) in mapf_instance.items():
        grid = []
        for _ in range(10):
            row = []
            for _ in range(10):
                row.append(0)
            grid.append(row)

        grid[2][4] = 1
        grid[3][4] = 1  
        path = astar(start, goal, grid)
        if path:
            paths[agent] = path
        else:
            print("No possible path found for agent:", agent)
    return paths

def validate_solution(solution):
    for agent1, path1 in solution.items():
        for agent2, path2 in solution.items():
            if agent1 != agent2:
                for timestep, (pos1, pos2) in enumerate(zip(path1, path2)):
                    if pos1 == pos2:
                        return False
    return True

def resolve_conflict(node, conflict):
    agent1, agent2, vertex, timestep = conflict
    
    # Choosing which agent's constraint to be added
    chosen_agent = random.choice([agent1, agent2])
    
    # Adding constraint for chosen agent
    chosen_constraints = node.constraints + [(chosen_agent, vertex, timestep)]
    chosen_solution = resolve_constraints(node.solution, chosen_constraints)
    
    return chosen_constraints, chosen_solution

def low_level_with_constraints(start, goal, grid, constraints):
    def heuristic(node, goal):
        return abs(node.position[0] - goal[0]) + abs(node.position[1] - goal[1])

    open_set = []
    heapq.heappush(open_set, (0, Node(start)))
    visited = set()

    while open_set:
        _, current_node = heapq.heappop(open_set)

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return list(reversed(path))

        visited.add((current_node.position, current_node.g))

        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_x, new_y = current_node.position[0] + dx, current_node.position[1] + dy
            if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and grid[new_x][new_y] == 0:
                new_position = (new_x, new_y)
                new_g = current_node.g + 1
                new_node = Node(new_position, current_node, new_g, heuristic(Node(new_position), goal))
                if not any(new_position == (c_agent_position[1], c_agent_position[2]) and new_g == c_timestep
                           for c_agent, c_agent_position, c_timestep in constraints):
                    
                    if (new_position, new_g) not in visited:
                        heapq.heappush(open_set, (new_node.f(), new_node))

    return None

def resolve_constraints(mapf_instance, constraints):
    paths = {}
    for agent, (start, goal) in mapf_instance.items():
        grid = []
        for _ in range(10):
            row = []
            for _ in range(10):
                row.append(0)
            grid.append(row) 
        grid[2][4] = 1
        grid[3][4] = 1   
        for c_agent, position, timestep in constraints:
            if c_agent == agent:
                if timestep < timestep + 1:
                    paths[agent] = low_level_with_constraints(start, goal, grid, constraints)
                else:
                    path = astar(start, goal, grid)
                    paths[agent] = path

    return paths

class CTNode:
    def __init__(self, constraints=None, solution=None, cost=0):
        self.constraints = constraints if constraints is not None else []
        self.solution = solution if solution is not None else {}
        self.cost = cost

def cost_and_conflicts(node):
    return node.cost, len(conflicts(node.solution))   #cost and no. of conflicts

def CBS(mapf_instance):
    root = CTNode()
    root.solution = find_individual_paths(mapf_instance)
    root.cost = SIC(root.solution)
    open_set = [root]

    while open_set:

        p = min(open_set, key=cost_and_conflicts)  # element with minimum cost and conflict
        open_set.remove(p)

        if validate_solution(p.solution):
            return p.solution

        for conflict in conflicts(p.solution):
            chosen_constraints, chosen_solution = resolve_conflict(p, conflict)
            child_node = CTNode(chosen_constraints, chosen_solution, SIC(chosen_solution))
            
            open_set.append(child_node)

    return {}

mapf_instance = {}

# initial and goal positions for each agent
agent_positions = {
    'agent0': ((0, 0), (9, 9)),
    'agent1': ((1, 1), (8, 3)),
    'agent2': ((2, 2), (6, 7)),
    'agent3': ((5, 3), (4, 6)),
    'agent4': ((5, 4), (9, 7))
}

for agent, (initial_position, goal_position) in agent_positions.items():
    mapf_instance[agent] = (initial_position, goal_position)
 
solution = CBS(mapf_instance)

for agent, path in solution.items():
    print("{} : {}".format(agent, path))