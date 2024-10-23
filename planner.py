# PLANNER
import numpy as np
# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        parabola = self.parabola()
        sigmoid = self.sigmoid()

        def parabola():
            x_values = np.linspace(0, 1.5, 100)
            for x in x_values:
                trajectory = [x, x**2]
            return trajectory
    
        def sigmoid():
            x_values = np.linspace(0, 1.5, 100)
            for x in x_values:
                trajectory = [x, 1/(1+np.exp(-x))-1]
            return trajectory
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return 