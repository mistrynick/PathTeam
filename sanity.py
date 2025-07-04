import subprocess
import matplotlib.pyplot as plt
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

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
        input_string += f" {obstacles[0]} {obstacles[1]}"

    stdout, stderr = process.communicate(input=input_string)


def parseData(stdout):
    tokens = stdout.split(" ")
    #position = (tokens[0], tokens[1])
    #path_raw = tokens[2:]
    path = list(zip(tokens[::2], tokens[1::2]))
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
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, 10)


        self.index = 0
        self.update_plot()

    def update_plot(self):
        start = (0,0)
        end = (10, 10)
        obstacles = [(2,3), (5,5),  (6,7) (8, 9)]

        while start[0] != end[0] and start[1] != end[1]:
            stdout = dstarplan(start, end, obstacles)
            path = parseData(stdout=stdout)
            xdata = []
            ydata = []
            for waypoint in path:
                xdata.append(waypoint[0])
                ydata.append(waypoint[1])
            
            self.line.set_data(xdata, ydata)
            self.canvas.draw()
            self.index += 1
            self.root.after(50, self.update_plot)  # Delay in milliseconds

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
