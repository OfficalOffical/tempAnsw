

def createGraph():

    # create a graph without library
    graph = {}
    graph['S'] = {'A': 3}
    graph['A'] = {'S': 3, 'B': 1, 'C': 2, 'D': 4, 'E': 2}
    graph['B'] = {'A': 1, 'E': 3}
    graph['C'] = {'A': 2, 'D': 2, 'G': 4, 'F': 3}
    graph['D'] = {'A': 4, 'C': 2, 'E': 1, 'G': 2}
    graph['E'] = {'A': 2, 'B': 3, 'D': 1, 'G': 3, 'H': 3}
    graph['F'] = {'C': 3, 'G': 2}
    graph['G'] = {'C': 4, 'D': 2, 'E': 3, 'F': 2, 'H': 1, 'T': 3, 'I': 1}
    graph['H'] = {'E': 3, 'G': 1, 'I': 2}
    graph['I'] = {'G': 1, 'H': 2, 'T': 1}
    graph['T'] = {'G': 3, 'I': 1}

    return graph

def findShortestPathWithDijkstra(graph):

    maxDistance = 1000000

    # create a dijkstra's algorithm
    unvisited = graph.keys()
    visited = set()
    # create a dictionary to store the distance from S to each node
    distanceBtwNodes = {node: [maxDistance, "S"] for node in graph}
    distanceBtwNodes['S'][0] = 0

    # find the shortest path
    for x in unvisited:
        for y in graph[x]:
            if distanceBtwNodes[y][0] > distanceBtwNodes[x][0] + graph[x][y] and y not in visited:
                distanceBtwNodes[y][0] = distanceBtwNodes[x][0] + graph[x][y]
                distanceBtwNodes[y][1] = x
        visited.add(x)

    # create the path with the shortest distance
    printPath(graph,distanceBtwNodes)

def printPath(graph,distanceBtwNodes):
    path = ['T']
    while path[-1] != 'S':
        path.append(distanceBtwNodes[path[-1]][1])
    path.reverse()

    # print the path with values
    for x in range(len(path) - 1):
        print(f"| {path[x]} | - {graph[path[x]][path[x + 1]]} >", end=' ')
    print(f"| {path[-1]} | Min distance from S to T is : {distanceBtwNodes[path[-1]][0]}")

    return distanceBtwNodes



def kruskal_mst(graph):
    # Create an empty list to store the edges of the MST
    mst = []
    # Create a dictionary to store the parent of each node in the tree
    parent = {node: node for node in graph}

    # Define a function to find the parent of a node
    def find(node):
        if parent[node] == node:
            return node
        else:
            return find(parent[node])
    # Define a function to merge two trees
    def merge(tree1, tree2):
        parent[tree1] = tree2
    # Create a list of edges sorted by weight
    edges = []
    for node in graph:
        for neighbor, weight in graph[node].items():
            edges.append((weight, node, neighbor))
    edges.sort()
    # Iterate through the edges and add them to the MST if they don't create a cycle
    for weight, node1, node2 in edges:
        if find(node1) != find(node2):
            mst.append((node1, node2, weight))
            merge(find(node1), find(node2))
    # Compute the total weight of the MST
    total_weight = sum(weight for node1, node2, weight in mst)

    print(mst)

    return total_weight


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    graph = createGraph()
    findShortestPathWithDijkstra(graph)
    kruskal_mst(graph)


# See PyCharm help at https://www.jetbrains.com/help/pycharm/
