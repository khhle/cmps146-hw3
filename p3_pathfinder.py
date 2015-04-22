import collections
import heapq
from math import sqrt
#It should define a function called "find_path". This function should take three arguments:
#source_point: (x,y) pair
#destination_point: (x,y) pair
#mesh: a mesh data structure (a fancy graph, described below)

#The function should return two values:
#path: a list of points like ((x1,y1),(x2,y2))
#visited_nodes: a list of boxes explored by your algorithm identified by their bounds (x1,x2,y1,y2) (this is the same as in the mesh format)


def find_path(src_point,dst_point,mesh):
    path = []
    visited_nodes = []
    flag = False
    box_list = mesh['boxes']
    adj_list = mesh['adj']
    direction = 0
    detail_points = {}


    #find box contain src
    for box in box_list:
        if(isInsideRect(box,src_point)):
            visited_nodes.append(box)
            flag = True
            break
        else:
            flag = False

    if flag is False:
        print("No path found")
        return [], []

    #find box contain dst
    for box in box_list:
        if(isInsideRect(box,dst_point)):
            visited_nodes.append(box)
            flag = True
            break
        else:
            flag = False

    if flag is False:
        print("No path found")
        return [], []
    #found_path,detail_points,visited_nodes = bfs(visited_nodes[0],visited_nodes[1],adj_list,src_point)
    found_path,detail_points,visited_nodes = dijkstra_search(visited_nodes[0],visited_nodes[1],adj_list,src_point)
    #found_path,detail_points,visited_nodes = a_star_search(visited_nodes[0],visited_nodes[1],adj_list,src_point,dst_point)
    #found_path,detail_points, direction, mid_point = bidirect_search(visited_nodes[0],visited_nodes[1],adj_list,src_point,dst_point)
    #Re-construct the path using line segment
    #start at dst_point since found_path in reverse order

    mid_point = (0,0)

    if direction != 0:
        pstart = mid_point
        for my_path in found_path:
            pend = detail_points[my_path][0]
            path.append((pstart,pend))
            pstart = pend
            if direction == 1 and my_path == visited_nodes[0]:
                pstart = mid_point
            if direction == -1 and my_path == visited_nodes[1]:
                pstart = mid_point
    else:
        pstart = dst_point
        for my_path in found_path:
            pend = detail_points[my_path][0]
            path.append((pstart,pend))
            pstart = pend
    #Add path to visited_nodes
    #for my_path in found_path:
    #    visited_nodes.append(my_path)


    return path,visited_nodes


#Calculate distance between 2 points
def euclidian(src,dst):
    startx,starty = src
    dstx, dsty = dst

    return sqrt((startx-dstx)*(startx-dstx) + (starty-dsty)*(starty-dsty))

#Find a point inside a given box
def find_point(src, box):
    x1,x2,y1,y2 = box
    srcx, srcy = src
    dstx = min(x2-1,max(x1,srcx))
    dsty = min(y2-1,max(y1,srcy))
    return dstx, dsty

#Find the cost from one box to another
def find_cost(src_box,dst_box,src_point):
    dst_point = find_point(src_point,dst_box)
    distance = euclidian(src_point, dst_point)
    return distance,dst_point



#check if a point is inside a rect
def isInsideRect(rect,point):
    x1,x2,y1,y2 = rect
    px,py = point

    if px < x1 and px < x2:
        return False
    if px > x1 and px > x2:
        return False
    if py < y1 and py < y2:
        return False
    if py > y1 and py > y2:
        return False

    return True







#BFS search
def bfs(source, target, adj,src_point):
    """Find a path from source to target on the graph defined by the adj function."""

    # initialize bookkeeping structures

    parent = {}
    discovered = {}
    queue = []
    visited = []
    detail_points = collections.defaultdict(list)
    # explore starting at the source node

    discovered[source] = True
    queue.append(source)

    iterations = 0
    detail_points[source].append(src_point)
    while queue:

        iterations += 1

        u = queue.pop()

        #if verbose:
        #    print "processing", u

        if u == target: # early termination
            break

        for v in adj[u]:
            if v not in discovered:
                visited.append(v)
                discovered[v] = True
                parent[v] = u
                queue.append(v)
                temp_point = find_point(detail_points[u][0],u)
                detail_points[v].append(temp_point)

    #if verbose:
    #    print "terminated after %d iterations" % iterations

    # reconstruct the path backwards, starting at the target node
    if u != target:
        print "no path"
        return [],([],[]),visited
    path = []
    node = target
    while node in parent:
        path.append(node)
        node = parent[node]
    path.append(source)
    #path.reverse()

    return path,detail_points,visited



#Dijkstra search
def dijkstra_search(start, goal,adj,src_point):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    detail_points = collections.defaultdict(list)

    detail_points[start].append(src_point)
    visited = []
    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break
        nextCells = adj[current] #find all the adj boxes
        for next in nextCells:
            cost, temp_point = find_cost(current,next,detail_points[current][0]) #find the cost and point inside
            new_cost = cost_so_far[current] + cost
            #new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                visited.append(next)
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
                #pstart = pend
                detail_points[next].append(temp_point)
    if current != goal:
        print "no path"
        return [],([],[]),visited
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    return path,detail_points,visited


#Helper function for A*
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)




#A* search
def a_star_search(start, goal,adj,src_point,dst_point):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    detail_points = collections.defaultdict(list)

    detail_points[start].append(src_point)
    visited = []
    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break
        nextCells = adj[current] #find all the adj boxes
        for next in nextCells:
            cost, temp_point = find_cost(current,next,detail_points[current][0]) #find the cost and point inside
            new_cost = cost_so_far[current] + cost
            #new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                visited.append(next)
                cost_so_far[next] = new_cost
                priority = new_cost + euclidian(dst_point, temp_point)
                #priority = new_cost + heuristic(dst_point, temp_point)
                frontier.put(next, priority)
                came_from[next] = current
                detail_points[next].append(temp_point)

    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    return path,detail_points,visited

#Bidirec search
def bidirect_search(start, goal,adj,src_point,dst_point):
    frontier = PriorityQueue2()
    frontier.put(start, 0,'dst')
    frontier.put(goal, 0,'src')

    came_from_forward = {}
    came_from_backward = {}
    cost_so_far_forward = {}
    cost_so_far_backward = {}

    visited_forward = {}
    visited_backward = {}

    came_from_forward[start] = None
    cost_so_far_forward[start] = 0
    came_from_backward[goal] = None
    cost_so_far_backward[goal] = 0

    detail_points = collections.defaultdict(list)

    detail_points[start].append(src_point)
    detail_points[goal].append(dst_point)

    visited_forward[start] = True
    visited_backward[goal] = True
    mid_point = (0,0)
    goal_mid = goal
    current_forward = start
    current_backward = goal
    direction = 0
    find = False
    while not frontier.empty():
        if find:
            break
        temp = frontier.get()
        if temp[2] == 'dst':
            #current_prev_forward = current_forward
            current_forward = temp[1]
            direction = 1

        elif temp[2] == 'src':
            #current_prev_backward = current_backward
            current_backward = temp[1]
            direction = -1
        #Visited[temp[1]] = True

        #if direction == -1 and current_backward == start:
        #    break

        #print Visited[current_forward]
        #if direction == 1 and current_forward in came_from_backward:
        #    foward_last = current_prev_forward
        #   came_from_forward[current_forward] = foward_last
        #    goal_mid = current_forward
        #    flag = True
        #    break
        #if direction == -1 and current_backward in came_from_forward:
        #    backward_last = current_prev_backward
        #    came_from_backward[current_backward] = backward_last
        #    goal_mid = current_backward
        #    flag = True
        #    break

        #if current_forward == goal and current_backward == start:
        #    break


        if direction == 1:
            nextCells_forward = adj[current_forward] #find all the adj boxes
        elif direction == -1:
            nextCells_backward = adj[current_backward] #find all the adj boxes

        if direction == 1:
            for next in nextCells_forward:
                cost, temp_point = find_cost(current_forward,next,detail_points[current_forward][0]) #find the cost and point inside
                new_cost = cost_so_far_forward[current_forward] + cost
                if next in came_from_backward:
                    came_from_forward[next] = current_forward
                    #detail_points[next].append(temp_point)
                    mid_point = temp_point
                    goal_mid = next
                    find = True
                    break
                #current_prev_forward = current_forward
                #new_cost = cost_so_far[current] + graph.cost(current, next)
                if next not in cost_so_far_forward or new_cost < cost_so_far_forward[next]:
                    cost_so_far_forward[next] = new_cost
                    priority = new_cost
                    frontier.put(next, priority,'dst')
                    came_from_forward[next] = current_forward
                    #pstart = pend
                    detail_points[next].append(temp_point)
                    visited_forward[next] = True

        if direction == -1:
            for next in nextCells_backward:
                cost, temp_point = find_cost(current_backward,next,detail_points[current_backward][0]) #find the cost and point inside
                new_cost = cost_so_far_backward[current_backward] + cost
                if next in came_from_forward:
                    came_from_backward[next] = current_backward
                    #detail_points[next].append(temp_point)
                    mid_point = temp_point
                    goal_mid = next
                    find = True
                    break
                #current_prev_backward = current_backward
                #new_cost = cost_so_far[current] + graph.cost(current, next)
                if next not in cost_so_far_backward or new_cost < cost_so_far_backward[next]:
                    cost_so_far_backward[next] = new_cost
                    priority = new_cost
                    frontier.put(next, priority,'src')
                    came_from_backward[next] = current_backward
                    #pstart = pend
                    detail_points[next].append(temp_point)
                    visited_backward[next] = True
    if find is False:
        print "No path found"
        return [],([],[]),0,(0,0)
    path = []
    if direction is 1:
        print("forward")
        current = current_forward
        path.append(current)
        while current != start:
            current = came_from_forward[current]
            path.append(current)

        current = goal_mid
        path.append(current)
        while current != goal:
            current = came_from_backward[current]
            path.append(current)

    if direction is -1:
        print("backward")
        current = current_backward
        path.append(current)
        while current != goal:
            current = came_from_backward[current]
            path.append(current)

        current = goal_mid
        path.append(current)
        while current != start:
            current = came_from_forward[current]
            path.append(current)


    #current = start
    #path = [current]
    #while current != goal:
    #    current = came_from_backward[current]
    #    path.append(current)

    return path,detail_points, direction, mid_point


#Helper class for search function
class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]
		
		
#Helper class for search function
class PriorityQueue2:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority,goal):
        heapq.heappush(self.elements, (priority, item,goal))

    def get(self):
        return heapq.heappop(self.elements)
