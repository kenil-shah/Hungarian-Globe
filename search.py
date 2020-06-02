import sys
import time
import heapq
from operator import itemgetter

state_expanded = 0


method_used = sys.argv[1]
file = sys.argv[2]
loop_count = 0
q_size = 0
max_qsize=0
"""
This array is used to check the axis on which the tile is currently located. It is used to calculate the distance of 
tile from its current state and goal state.
"""
three_axis = [[0, 2, 6, 10, 14, 18, 1, 19, 15, 11, 7, 3],
              [0, 5, 9, 13, 17, 21, 1, 20, 16, 12, 8, 4],
              [12, 23, 22, 10, 29, 28, 13, 27, 26, 11, 25, 24]]
"""
eqa = Rotating Equator Anticlockwise(Decrement)
eqc = Rotating Equator Anticlockwise(Increment)
a2a = Rotating 90-270 Longitude Anticlockwise(Decrement)
a2c = Rotating 90-270 Longitude Clockwise(Increment)
a1c = Rotating 0-180 Longitude Clockwise(Decrement)
a1a = Rotating 0-180 Longitude Anticlockwise(Increment)
"""
# Total possible actions would be as mentioned below
possible_actions = ['a1c', 'a1a', 'a2c', 'a2a', 'eqc', 'eqa']
goal_axis = [[1, 2], [1, 2], [1], [1], [2], [2], [1], [1], [2], [2], [1, 3], [1, 3], [2, 3], [2, 3], [1], [1], [2], [2],
             [1], [1], [2], [2], [3], [3], [3], [3], [3], [3], [3], [3]]


action_output = {
    '0a1c': 2,
    '0a1a': 3,
    '0a2c': 5,
    '0a2a': 4,
    '1a1c': 19,
    '1a1a': 18,
    '1a2c': 20,
    '1a2a': 21,
    '2a1c': 6,
    '2a1a': 0,
    '3a1c': 0,
    '3a1a': 7,
    '4a2c': 0,
    '4a2a': 8,
    '5a2c': 9,
    '5a2a': 0,
    '6a1c': 10,
    '6a1a': 2,
    '7a1c': 3,
    '7a1a': 11,
    '8a2c': 4,
    '8a2a': 12,
    '9a2c': 13,
    '9a2a': 5,
    '10a1c': 14,
    '10a1a': 6,
    '10eqa': 29,
    '10eqc': 22,
    '11a1c': 7,
    '11a1a': 15,
    '11eqa': 25,
    '11eqc': 26,
    '12a2c': 8,
    '12a2a': 16,
    '12eqa': 23,
    '12eqc': 24,
    '13a2c': 17,
    '13a2a': 9,
    '13eqa': 27,
    '13eqc': 28,
    '14a1c': 18,
    '14a1a': 10,
    '15a1c': 11,
    '15a1a': 19,
    '16a2c': 12,
    '16a2a': 20,
    '17a2c': 21,
    '17a2a': 13,
    '18a1c': 1,
    '18a1a': 14,
    '19a1c': 15,
    '19a1a': 1,
    '20a2c': 16,
    '20a2a': 1,
    '21a2c': 1,
    '21a2a': 17,
    '22eqa': 10,
    '22eqc': 23,
    '23eqa': 22,
    '23eqc': 12,
    '24eqa': 12,
    '24eqc': 25,
    '25eqa': 24,
    '25eqc': 11,
    '26eqa': 11,
    '26eqc': 27,
    '27eqa': 26,
    '27eqc': 13,
    '28eqa': 13,
    '28eqc': 29,
    '29eqa': 28,
    '29eqc': 10,
}

'''
Defining State Space for the initial state as defined in the files.
Eg
 _ _ _ _ _ _ _ _
|'0,0'     -- 0 |
|'180,180' -- 1 |
|'30,0'    -- 2 |
|'30,180'  -- 3 |
|'30,90'   -- 4 |
|.....          |
|'90,330'  -- 29|
'''


class StateSpace:

    def state_space(self, file_name):
        file_read = open(file_name, "r")
        contents = file_read.readlines()
        n = 0
        initial_state = []
        goal_state = []
        lookup = {}
        for content in contents:
            if (n != 0) and (n != 31):
                initial_state.append(content.split(', ')[1][1:-1])
                lookup[content.split(', ')[2][6:-3]] = n - 1
            n = n + 1
        count = 0
        for x in initial_state:
            initial_state[count] = lookup[x]
            goal_state.append(count)
            count = count + 1
        return initial_state, goal_state


class BFS:
    def apply_action(self, current_node):
        flag = 0
        node_outer = []
        possible_actions = ['a1c', 'a1a', 'a2c', 'a2a', 'eqc', 'eqa']
        try:
            if current_node[0].split("_")[0][-1] == 'a':
                possible_actions.remove(current_node[0].split("_")[0].split("-")[-1][:-1]+'c')
            else:
                possible_actions.remove(current_node[0].split("_")[0].split("-")[-1][:-1]+'a')
        except:
            pass
        path = current_node[0].split("_")[0]
        cost = str(int(current_node[0].split("_")[1]) + 1)
        current_node.pop(0)
        for x in possible_actions:
            node_inner = [path+"-"+str(x)]
            for state in current_node:
                try:
                    node_inner.append(action_output[str(state) + x])
                except:
                    node_inner.append(state)
            node_outer.append(node_inner)
            if node_inner[1:] == goal_state:
                flag = 1
                node_inner[0]=node_inner[0]+"_"+cost
                return node_inner, flag
        return node_outer, flag

    def bfs_node_expansion(self, initial_state):
        current_node = initial_state
        frontier_nodes = []
        count = 0

        while True:
            cost = str(int(current_node[0].split("_")[1])+1)
            count = count + 1
            node_outer, flag = self.apply_action(current_node)
            if flag == 1:
                print("Queue Length", len(frontier_nodes))
                print("Nodes Expanded", count)
                print("Path Length", node_outer[0].split("_")[1])
                print("Path taken", node_outer[0].split("_")[0])
                break
            for node in node_outer:
                node[0] = str(node[0]) + "_" + str(cost)
                frontier_nodes.append(node)
            #frontier_nodes += node_outer
            current_node = frontier_nodes.pop(0)


class AStar:
    def apply_action_astar(self, current_node):
        flag = 0
        if current_node[1:31] == goal_state:
            flag = 1
            return current_node, flag
        node_outer = []
        possible_actions = ['a1c', 'a1a', 'a2c', 'a2a', 'eqc', 'eqa']
        try:
            if current_node[0].split("_")[0][-1] == 'a':
                possible_actions.remove(current_node[0].split("_")[0].split("-")[-1][:-1]+'c')
            else:
                possible_actions.remove(current_node[0].split("_")[0].split("-")[-1][:-1]+'a')
        except:
            pass
        current_path = current_node[0]
        path = current_node[0].split("_")[0]
        current_node.pop(0)
        for x in possible_actions:
            node_inner = [path+"-"+str(x)]
            for state in current_node:
                try:
                    node_inner.append(action_output[str(state) + x])
                except:
                    node_inner.append(state)
            node_outer.append(node_inner)
        current_node.insert(0, current_path)
        return node_outer, flag

    def heuristic_cost_2(self, node):
        h_cost = 0
        node = node[1:]
        final = 0
        for x in range(0, 30):
            if node[x] == x:
                h_cost = h_cost+0
            else:
                if node[x] in three_axis[0]:
                    state_axis = 1
                elif node[x] in three_axis[1]:
                    state_axis = 2
                else:
                    state_axis = 3
                if state_axis in goal_axis[x]:
                    pos1 = three_axis[state_axis-1].index(node[x])
                    pos2 = three_axis[state_axis-1].index(x)
                    final = abs(pos1-pos2)
                    if final > 6:
                        final = 12 - final
                    h_cost = h_cost + final
                else:
                    int_pts = list(set(three_axis[goal_axis[x][0]-1]).intersection(three_axis[state_axis-1]))
                    # Distance for first intersection
                    pos1 = three_axis[state_axis-1].index(node[x])
                    pos2 = three_axis[goal_axis[int_pts[0]][0]-1].index(int_pts[0])
                    final1 = abs(pos1-pos2)
                    if final1 > 6:
                        final1 = 12 - final1

                    # Distance for second intersection
                    pos1 = three_axis[state_axis-1].index(node[x])
                    pos2 = three_axis[goal_axis[int_pts[1]][0]-1].index(int_pts[1])
                    final2 = abs(pos1-pos2)
                    if final2 > 6:
                        final2 = 12 - final2

                    final_axis = min(final1, final2)
                    get_index = (final1, final2).index(final_axis)
                    pos1 = three_axis[goal_axis[x][0]-1].index(int_pts[get_index])
                    pos2 = three_axis[goal_axis[x][0]-1].index(x)
                    final = abs(pos1-pos2)
                    if final > 6:
                        final = 12 - final
                    final = final + final_axis
                    h_cost = h_cost + final
        return h_cost

    def astar_node_expansion(self, initial_state):
        current_node = initial_state
        frontier_nodes = []
        count = 0
        while True:
            cost = str(int(current_node[0].split("_")[1])+1)
            node_outer, flag = self.apply_action_astar(current_node)
            if flag == 1:
                print("Queue Length", len(frontier_nodes))
                print("Nodes Expanded", count)
                final_path_length = current_node[0].split("_")[1]
                print("Final Path Length", current_node[0].split("_")[1])
                print("Goal Path", current_node[0].split("_")[0])
                break
            for node in node_outer:
                node[0] = str(node[0]) + "_" + str(cost)
            for node in node_outer:
                path_cost = int(node[0].split("_")[1])
                h_cost = self.heuristic_cost_2(node)/13
                total_cost = int(path_cost) + h_cost
                try:
                    node[31] = total_cost
                except:
                    node.append(total_cost)
                heapq.heappush(frontier_nodes, (node[-1], count, node))
            current_node = heapq.heappop(frontier_nodes)[-1]
            count = count + 1


class RBFS:

    def apply_action_astar(self, current_node):
        flag = 0
        node_outer = []
        possible_actions = ['a1c', 'a1a', 'a2c', 'a2a', 'eqc', 'eqa']
        try:
            if current_node[0].split("_")[0][-1] == 'a':
                possible_actions.remove(current_node[0].split("_")[0].split("-")[-1][:-1]+'c')
            else:
                possible_actions.remove(current_node[0].split("_")[0].split("-")[-1][:-1]+'a')
        except:
            pass
        current_path = current_node[0]
        path = current_node[0].split("_")[0]
        current_node.pop(0)
        for x in possible_actions:
            node_inner = [path+"-"+str(x)]
            for state in current_node:
                try:
                    node_inner.append(action_output[str(state) + x])
                except:
                    node_inner.append(state)
            node_outer.append(node_inner)
        current_node.insert(0, current_path)
        return node_outer, flag

    def node_check(self, outer_node):
        x =sorted(outer_node, key=itemgetter(31))
        return x[0], outer_node.index(x[0])

    def find_second(self, outer_node):
        return sorted(outer_node, key=itemgetter(31))[1]

    def recurssion_model(self, current, f_limit):
        global state_expanded
        global q_size
        global max_qsize
        if current[1:31] == goal_state:
            print("Nodes Expanded", state_expanded)
            print("Final Path Length", current[0].split("_")[1])
            print("Goal Path", current[0].split("_")[0])
            return current, f_limit
        rbfs_inherit = AStar()
        cost = str(int(current[0].split("_")[1]) + 1)
        state_expanded += 1
        node_outer, flag = self.apply_action_astar(current)
        q_size = q_size + 5
        for node in node_outer:
            node[0] = str(node[0]) + "_" + str(cost)
            path_cost = int(node[0].split("_")[1])
            h_cost = rbfs_inherit.heuristic_cost_2(node) / 13
            total_cost = int(path_cost) + (h_cost)
            try:
                node[31] = total_cost
            except:
                node.append(total_cost)
        if len(current) < 32:
            current.append(0)
        for node in node_outer:
            node[31] = max(node[31], current[31])

        while True:
            best_state, best_index = self.node_check(node_outer)
            if best_state[31] > f_limit:
                q_size = q_size - 1
                return None, best_state[31]
            alternative = self.find_second(node_outer)
            result, node_outer[best_index][31] = self.recurssion_model(best_state, min(alternative[31], f_limit))
            if result != None:
                return result, f_limit

    def rbfs_node_expansion(self, initial_state):
        current_node = initial_state
        result, f_limit = self.recurssion_model(current_node, 100000)


state_representation = StateSpace()
bfs_method = BFS()
astar_method = AStar()
rbfs_method = RBFS()
initial_state, goal_state = state_representation.state_space(file)
initial_state.insert(0, '_0')
start = time.time()
if method_used == "BFS":
    print("Running BFS")
    print("Processing...")
    bfs_method.bfs_node_expansion(initial_state)
elif method_used == "AStar":
    print("Running AStar")
    print("Processing...")
    astar_method.astar_node_expansion(initial_state)
elif method_used == "RBFS":
    print("RBFS")
    print("Processing...")
    rbfs_method.rbfs_node_expansion(initial_state)
else:
    print("Please Enter a valid input")

elapsed = time.time()
print("TOTAL TIME", elapsed-start)
print("Path Description")
print("a1c----> 0-180 Longitude Clockwise(Decrement)")
print("a1a----> 0-180 Longitude Anticlockwise(Increment)")
print("a2c----> 90-270 Longitude Clockwise(Increment)")
print("a2a----> 90-270 Longitude Anticlockwise(Decrement)")
print("eqa----> Equator Anticlockwise(Decrement)")
print("eqc----> Equator Anticlockwise(Increment)")
