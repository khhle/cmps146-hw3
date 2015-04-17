

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

    box_list = mesh['boxes']
    adj_list = mesh['adj']
    #find box contain src
    for box in box_list:
        if(isInsideRect(box,src_point)):
            visited_nodes.append(box)

    #find box contain src
    for box in box_list:
        if(isInsideRect(box,dst_point)):
            visited_nodes.append(box)


    bfs_path = bfs(visited_nodes[0],visited_nodes[1],adj_list)
    for my_path in bfs_path:
        visited_nodes.append(my_path)

    return path,visited_nodes



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