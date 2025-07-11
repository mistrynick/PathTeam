import subprocess
import matplotlib.pyplot as plt
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import re
import time
import random 
import numpy as np
import cv2

def dstarplan(start, end, obstacles):
    process = subprocess.Popen(
        ['./path'],           
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,      
        text=True                    
    )
    input_string = f"{start[0]} {start[1]} {end[0]} {end[1]} {len(obstacles)}"
   
    for ob in obstacles:
        input_string += f" {ob[0]} {ob[1]}"
    
    stdout, stderr = process.communicate(input=input_string)
    return stdout


def parseData(stdout):
    path = [tuple(map(int, line.split())) for line in stdout.strip().split('\n')]
    return path

    
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("dstar lite")

        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)

       
        self.line, = self.ax.plot([], [], 'b-')
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.start = [1,1]
        self.end = [78, 90]
        

        self.index = 0
        self.initial = [(3, 3), (70, 5), (50, 7), (8, 9), (5,3)]
        self.obstacles = [(3, 3), (70, 5), (50, 7), (8, 9), (5,3)]
        self.endPoints = [(90, 80), (70, 79), (50, 100), (60, 70), (30,80)] # final position for obstacles
        self.randomObstacles()
    
        self.ox = [pt[0] for pt in self.obstacles]
        self.oy = [pt[1] for pt in self.obstacles]
        self.obstacle_scatter = self.ax.scatter(self.ox, self.oy, c='red', s=10, marker='s')
        self.update_plot()
    
    def randomObstacles(self):
        n_obstacles = 500
        self.obstacles = []
        self.initial = []
        self.endPoints = []
        for _ in range(n_obstacles):
            x = random.randint(0, 100)
            y = random.randint(0, 100)
            ex = random.randint(0, 100)
            ey = random.randint(0, 100)
            self.obstacles.append((x,y))
            self.initial.append((x,y))
            self.endPoints.append((ex, ey))
        
        self.ox = [pt[0] for pt in self.obstacles]
        self.oy = [pt[1] for pt in self.obstacles]
            
    
    def moveObstacles(self):
        newList = []

        for i, (x,y) in enumerate(self.obstacles):
            target_x, target_y = self.endPoints[i]
            dx = target_x - x
            dy = target_y - y
            if dx != 0:
                new_x = x + (1 if dx > 0 else -1)
                new_y = y
            elif dy != 0:
                new_x = x
                new_y = y + (1 if dy > 0 else -1)
            else:
                new_x, new_y = x, y 
                self.endPoints[i] = self.initial[i]
            
            newList.append((new_x, new_y))
        self.obstacles = newList
        
        self.ox = [pt[0] for pt in self.obstacles]
        self.oy = [pt[1] for pt in self.obstacles]
        

    def update_plot(self):
        
        
        if self.start[0] != self.end[0] and self.start[1] != self.end[1]:
            startTime = time.time()
            stdout = dstarplan(self.start, self.end, self.obstacles)
            endTime = time.time()
            timeElasped = endTime - startTime
            print(f"iteration: {self.index}, replanning took {timeElasped} seconds!")
            path = parseData(stdout=stdout)

            xdata = []
            ydata = []
            for waypoint in path:
                xdata.append(waypoint[0])
                ydata.append(waypoint[1])
            
            self.line.set_data(xdata, ydata)
            self.obstacle_scatter.set_offsets(np.c_[self.ox, self.oy])
            
            self.canvas.draw()
            self.index += 1
            self.root.after(100, self.update_plot)  
            self.start[0] = path[1][0]
            self.start[1] = path[1][1]
            self.moveObstacles()


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
