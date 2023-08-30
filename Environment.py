import numpy as np

class Environment:
    def __init__(self, config):
        self.env = np.load(config)
        self.dims = self.env.shape

    
    def check_collision(self, x, y, z, quad_size=5):
        size = quad_size//2
        x0 = max(0, x-size)
        x1 = min(self.dims[0], x+size+1)
        y0 = max(0, y-size)
        y1 = min(self.dims[1], y+size+1)
        z0 = max(0, z-size)
        z1 = min(self.dims[2], y+size+1)

        if np.sum(self.env[x0:x1, y0:y1, z0:z1]) > 0:
            return False
        else:
            return True




# class VoxelEnvironment(Environment):
#     def __init__(self, height, width, config_file):
#         super().__init__(height, width)
#         self.load_config(config_file)

    
#     def check_collision(self, x, y):
#         # TODO

    
#     def load_config(self, config_file):
#         # TODO
    