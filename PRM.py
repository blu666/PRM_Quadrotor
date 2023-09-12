import numpy as np
from Environment import Environment
import bisect
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class State():
    def __init__(self, pos, yaw):
        self.pos = pos
        self.yaw = yaw

class PRM():
    def __init__(self, start, goal, environment, neighborhood_radius, iterations=500, path_resolution=0.5, quad_size=5):
        self.start = start
        self.goal = goal
        self.env = environment
        self.dims = self.env.env.shape
        self.neighborhood_radius = neighborhood_radius
        self.iterations = iterations
        self.path_resolution = path_resolution
        self.quad_size = quad_size



        self.nodes = []
        self.nodes.append(State(start, np.random.random_sample() * 2 * np.pi))
        self.nodes.append(State(goal, np.random.random_sample() * 2 * np.pi))

        self.edges = dict() # {node: [node1, node2, ...]}
        self.edges[self.nodes[0]] = []
        self.edges[self.nodes[1]] = []
        
        self.connectedComponents = dict() # {node: cc_parent}
        self.connectedComponents[self.nodes[0]] = self.nodes[0]
        self.connectedComponents[self.nodes[1]] = self.nodes[1]

        self.path = []


    def checkState(self, state):
        x, y, z = state.pos
        x, y, z = int(x), int(y), int(z)
        return self.env.check_collision(x, y, z, self.quad_size)
    
    
    def sampleFreeState(self):
        while True:
            pos = np.random.random_sample(3) * self.dims
            yaw = np.random.random_sample() * 2 * np.pi

            if self.checkState(State(pos, yaw)):
                return State(pos, yaw)
        

    def findNeighbors(self, node, radius):
        distance = []
        for n in self.nodes:
            d = np.linalg.norm(node.pos - n.pos)
            if d <= radius:
                bisect.insort(distance, (d, n))
        return distance


    def checkPathCollision(self, node1, node2):
        vec = node2.pos - node1.pos
        dist = np.linalg.norm(vec)
        unit_vec = vec / dist
        num_points = int(dist / self.path_resolution)

        for i in range(1, num_points):
            pos = node1.pos + unit_vec * i * self.path_resolution
            if not self.checkState(State(pos, 0)):
                return False
        return True


    def checkFeasibility(self, node1, node2):
        pass

    
    def findCCParent(self, node):
        while node != self.connectedComponents[node]:
            node = self.connectedComponents[node]
        return node


    def traversePath(self):
        
        self.dfs(self.nodes[0], set())
        self.path.append((self.nodes[0].pos, self.nodes[0].yaw))
        self.path.reverse()
        # print("path success")
        
    
    def dfs(self, node, visited):
        if node == self.nodes[1]:
            self.path.append((node.pos, node.yaw))
            print("path success")
            return True
        visited.add(node)
        for n in self.edges[node]:
            if n not in visited:
                if self.dfs(n, visited):
                    self.path.append((n.pos, n.yaw))
                    return True
        return False


    def plan(self):
        found = False
        for i in range(self.iterations):
            node = self.sampleFreeState()
            neighbors = self.findNeighbors(node, self.neighborhood_radius)
            self.nodes.append(node)
            self.connectedComponents[node] = node
            self.edges[node] = []
            for n in neighbors:
                cc1 = self.findCCParent(node)
                cc2 = self.findCCParent(n[1])
                if cc1 != cc2 and self.checkPathCollision(node, n[1]):
                    self.connectedComponents[cc1] = cc2
                    self.edges[node].append(n[1])
                    self.edges[n[1]].append(node)

            if self.findCCParent(self.nodes[0]) == self.findCCParent(self.nodes[1]):
                if not found:
                    print("Found path after {} iterations".format(i))
                    found = True
        
        self.traversePath()
        return self.path
    

    def plot_path(self):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        x, y, z = self.env.env.nonzero()
        ax.scatter(x, y, z, color="aqua", alpha=0.2)

        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_zlim(0, 100)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        print(len(self.path))
        node = self.path[0]
        ax.scatter(node[0][0], node[0][1], node[0][2], color="green", alpha=0.5)
        last = node
        # print(node)
        for i in range(1, len(self.path)-1):
            node = self.path[i]
            ax.scatter(node[0][0], node[0][1], node[0][2], color="red", alpha=0.5)
            ax.plot([last[0][0], node[0][0]], [last[0][1], node[0][1]], [last[0][2], node[0][2]], color="tomato", alpha=0.5)
            last = node
            # print(node)
        node = self.path[-1]
        ax.scatter(node[0][0], node[0][1], node[0][2], color="green", alpha=0.5)
        ax.plot([last[0][0], node[0][0]], [last[0][1], node[0][1]], [last[0][2], node[0][2]], color="tomato", alpha=0.5)
        

        plt.show()
        

if __name__ == "__main__":
    env = Environment("map1.npy")
    prm = PRM((10, 20, 50), (90, 100, 100), env, 10, iterations=3000, path_resolution=0.5, quad_size=3)
    prm.plan()
    print(prm.path)
    prm.plot_path()
