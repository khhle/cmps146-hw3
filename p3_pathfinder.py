
from heapq import heappush, heappop
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
    coordinates = []
    box_list = mesh['boxes']
    adj_list = mesh['adj']

    box_list_traverse = []
    detail_points = {'Box': box_list_traverse, 'xy': coordinates}

    #find box contain src
    for box in box_list:
        if(isInsideRect(box,src_point)):
            visited_nodes.append(box)

    #find box contain dst
    for box in box_list:
        if(isInsideRect(box,dst_point)):
            visited_nodes.append(box)


    #bfs_path = bfs(visited_nodes[0],visited_nodes[1],adj_list)
    bfs_path = dijkstras_shortest_path(visited_nodes[0], src_point, visited_nodes[1],adj_list)
    for my_path in bfs_path:
        visited_nodes.append(my_path)
    path = path_search(bfs_path,src_point, dst_point)
    #path.append((src_point, dst_point))
    #path = ((100,200),(300,400))
    #print str(test)
    return path,visited_nodes

def euclidian(src,dst):
    startx,starty = src
    dstx, dsty = dst

    return sqrt((startx-dstx)*(startx-dstx) + (starty-dsty)*(starty-dsty))

def find_point(src, box): #finds midpoint in box
    x1,x2,y1,y2 = box
    srcx, srcy = src
    dstx = min(x2-1,max(x1,srcx))
    dsty = min(y2-1,max(y1,srcy))
    return dstx, dsty

def path_search(boxes,src, end):
    path = []
    for box in boxes:
        if box == boxes[-1]:
            coordinate = end
            path.append((src,coordinate))
        else:
            coordinate = find_point(src, box)
            path.append((src,coordinate))
            src = coordinate
    return path

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

#finds distances from current box to adj boxes
def dfind_distance(adj, src_point):
    valid = []
    for box in adj:
        point = find_point(src_point, box)
        distance = euclidian(src_point, point)
        valid.append((distance, point))
    return valid


def dijkstras_shortest_path(src_box, src_point, dst, adj):
    Dist = {}
    Prev = {}
    #and another one to check if nodes have been fully processed
    Processed = {}
    Dist[src_box] = 0
    queue = [(0,src_box)]
    nextCells = [(Dist[src_box], src_point)]
    found = False

    #while queue is not empty. If it's empty we've searched all valid points
    while(queue):
        curr = heappop(queue)
        Processed[curr[1]] = True
        #print "Popping %s with Distance" %(curr[1],)

        nextCells = dfind_distance(adj, src_box)
        print (nextCells[0][1])
        for i in range(0, len(nextCells)):
            if not (nextCells[i][1] in Processed):
                #update the distance in the current tuple
                nextCells[i] = (euclidian, nextCells[i][1])
                #check if distance needs updating in the dict
                if not (nextCells[i][1] in Dist) or (Dist[nextCells[i][1]] > nextCells[i][0]):
                    Dist[nextCells[i][1]] = nextCells[i][0]
                    Prev[nextCells[i][1]] = curr[1]
                    #push onto the queue
                    heappush(queue, nextCells[i])
                    if nextCells[i][1] == dst:
                        found = True
                        break
        if found:
            break

    if found:
        path = []
        curr = Prev[dst]
        while curr != src_box:
            path.append(curr)
            curr = Prev[curr]
        return path


#BFS search
def bfs(source, target, adj, verbose=False):
    """Find a path from source to target on the graph defined by the adj function."""

    # initialize bookkeeping structures

    parent = {}
    discovered = {}
    queue = []

    # explore starting at the source node

    discovered[source] = True
    queue.append(source)

    iterations = 0

    while queue:

        iterations += 1

        u = queue.pop()

        if verbose:
            print "processing", u

        if u == target: # early termination
            break

        for v in adj[u]:
            if v not in discovered:
                discovered[v] = True
                parent[v] = u
                queue.append(v)

    if verbose:
        print "terminated after %d iterations" % iterations

    # reconstruct the path backwards, starting at the target node

    path = []
    node = target
    while node in parent:
        path.append(node)
        node = parent[node]
    path.append(source)
    path.reverse()

    return path



