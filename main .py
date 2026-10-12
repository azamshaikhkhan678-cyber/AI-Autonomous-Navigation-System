# main.py
"""
AI-Based Autonomous Navigation System
Built using Python's native Tkinter UI framework to bypass internet installation blockers.
Features: 8-way directional A* pathfinder matrix navigation and dynamic obstacle placement.
"""

import math
import heapq
import tkinter as tk
from tkinter import messagebox

# Grid Canvas Constants
WIDTH, HEIGHT = 900, 650
GRID_SIZE = 25
ROWS = HEIGHT // GRID_SIZE
COLS = WIDTH // GRID_SIZE

# Color Hex Codes
COLOR_BG = "#141821"        # Dark slate theme map workspace
COLOR_WALL = "#414e61"      # Grey-blue barriers
COLOR_CAR = "#00c873"       # Neon green vehicle indicator 
COLOR_GOAL = "#ff3c5a"      # Crimson red target destination position
COLOR_PATH = "#0096ff"      # Cyber cyan planned trajectory trace
COLOR_SENSOR = "#f0b414"    # Amber sensor field outline

class NativeNavigationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("AI Autonomous Navigation System - Native Portal Engine")
        self.root.configure(bg="#12161f")

        # System State Framework variables
        self.grid_matrix = [[0 for _ in range(COLS)] for _ in range(ROWS)]
        self.start_node = (2, 2)
        self.goal_node = (COLS - 3, ROWS - 3)
        self.robot_pos = [float(self.start_node[0] * GRID_SIZE + 12), float(self.start_node[1] * GRID_SIZE + 12)]
        self.robot_angle = 0.0
        self.planned_waypoint_path = []
        self.is_running = False

        # Generate outer perimeter boundary walls automatically
        for r in range(ROWS):
            self.grid_matrix[r][0] = 1
            self.grid_matrix[r][COLS-1] = 1
        for c in range(COLS):
            self.grid_matrix[0][c] = 1
            self.grid_matrix[ROWS-1][c] = 1

        # Add fixed default layout structures
        for r in range(4, 16): self.grid_matrix[r][13] = 1
        for c in range(13, 28): self.grid_matrix[16][c] = 1

        # Assemble UI Interfaces
        self.control_panel = tk.Frame(root, bg="#18202c", bd=2, relief=tk.GROOVE)
        self.control_panel.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)

        self.btn_start = tk.Button(self.control_panel, text="▶ START AI RUN", font=("Courier New", 11, "bold"), 
                                   fg="#ffffff", bg="#0088cc", activebackground="#005588", activeforeground="#ffffff",
                                   command=self.toggle_execution, width=18)
        self.btn_start.pack(side=tk.LEFT, padx=15, pady=5)

        self.lbl_info = tk.Label(self.control_panel, text="SANDBOX: Click and drag on map grid space to paint walls!", 
                                 font=("Courier New", 10, "bold"), fg="#f0b414", bg="#18202c")
        self.lbl_info.pack(side=tk.LEFT, padx=20)

        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg=COLOR_BG, highlightthickness=0)
        self.canvas.pack(padx=10, pady=10)

        # Bind mouse movement triggers to draw custom obstacle walls seamlessly
        self.canvas.bind("<B1-Motion>", self.paint_wall_drag)
        self.canvas.bind("<Button-1>", self.paint_wall_drag)

        self.compute_global_route()
        self.render_canvas_frame()
        self.update_simulation_loop()

    def compute_global_route(self):
        """Executes discrete 8-way A* graph search formulas to map trajectory trails."""
        start = (int(self.robot_pos[0] // GRID_SIZE), int(self.robot_pos[1] // GRID_SIZE))
        goal = self.goal_node

        open_list = []
        heapq.heappush(open_list, (0, start))
        origin_tracker = {}
        g_scores = {start: 0.0}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                path = []
                while current in origin_tracker:
                    path.append(current)
                    current = origin_tracker[current]
                path.reverse()
                self.planned_waypoint_path = path
                return

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if not (0 <= neighbor[0] < COLS and 0 <= neighbor[1] < ROWS): continue
                if self.grid_matrix[neighbor[1]][neighbor[0]] == 1: continue

                weight = 1.414 if (dx != 0 and dy != 0) else 1.0
                test_g = g_scores[current] + weight

                if neighbor not in g_scores or test_g < g_scores[neighbor]:
                    origin_tracker[neighbor] = current
                    g_scores[neighbor] = test_g
                    f = test_g + math.hypot(goal[0]-neighbor[0], goal[1]-neighbor[1])
                    heapq.heappush(open_list, (f, neighbor))
        self.planned_waypoint_path = []

    def paint_wall_drag(self, event):
        mc, mr = event.x // GRID_SIZE, event.y // GRID_SIZE
        if 0 <= mc < COLS and 0 <= mr < ROWS:
            if (mc, mr) != self.goal_node and (int(self.robot_pos[0]//GRID_SIZE), int(self.robot_pos[1]//GRID_SIZE)) != (mc, mr):
                self.grid_matrix[mr][mc] = 1
                self.compute_global_route()
                self.render_canvas_frame()

    def toggle_execution(self):
        self.is_running = not self.is_running
        if self.is_running:
            self.btn_start.config(text="⏸ PAUSE SIM", bg="#ff3c5a")
        else:
            self.btn_start.config(text="▶ START AI RUN", bg="#0088cc")

    def update_simulation_loop(self):
        """Drives kinematic vehicle position adjustments and local hazard avoidance rules."""
        if self.is_running and self.planned_waypoint_path:
            target_node = self.planned_waypoint_path[0]
            tx = target_node[0] * GRID_SIZE + GRID_SIZE // 2
            ty = target_node[1] * GRID_SIZE + GRID_SIZE // 2

            # Advance waypoint lists upon clearing arrival boundary thresholds
            if math.hypot(tx - self.robot_pos[0], ty - self.robot_pos[1]) < 12:
                if len(self.planned_waypoint_path) > 1:
                    self.planned_waypoint_path.pop(0)

            # Steering Guidance Angular Computations
            desired_angle = math.atan2(ty - self.robot_pos[1], tx - self.robot_pos[0])
            angle_diff = (desired_angle - self.robot_angle + math.pi) % (2 * math.pi) - math.pi
            self.robot_angle += max(min(angle_diff, 0.08), -0.08)

            # Drive forward configuration mechanics
            next_x = self.robot_pos[0] + math.cos(self.robot_angle) * 2.5
            next_y = self.robot_pos[1] + math.sin(self.robot_angle) * 2.5

            # Stop movement safely if path maps intersect new block obstacles
            chk_c, chk_r = int(next_x // GRID_SIZE), int(next_y // GRID_SIZE)
            if 0 <= chk_c < COLS and 0 <= chk_r < ROWS and self.grid_matrix[chk_r][chk_c] == 0:
                self.robot_pos[0] = next_x
                self.robot_pos[1] = next_y
            else:
                self.compute_global_route()

            if math.hypot(gx - self.robot_pos[0], gy - self.robot_pos[1]) < 15:
                self.is_running = False
                self.btn_start.config(text="▶ START AI RUN", bg="#0088cc")
                messagebox.showinfo("System Telemetry Alert", "Destination Target successfully reached!")
                self.robot_pos = [float(self.start_node[0]*GRID_SIZE+12), float(self.start_node[1]*GRID_SIZE+12)]
                self.compute_global_route()

        self.render_canvas_frame()
        self.root.after(16, self.update_simulation_loop)

    def render_canvas_frame(self):
        """Clears and renders all tracking parameters onto the display area canvas."""
        self.canvas.delete("all")

        # Render barrier grid coordinates
        for r in range(ROWS):
            for c in range(COLS):
                if self.grid_matrix[r][c] == 1:
                    self.canvas.create_rectangle(c*GRID_SIZE, r*GRID_SIZE, (c+1)*GRID_SIZE, (r+1)*GRID_SIZE, 
                                                 fill=COLOR_WALL, outline="#2c3545")

        # Draw goal destination target circle graphic
        global gx, gy
        gx = self.goal_node[0] * GRID_SIZE + GRID_SIZE // 2
        gy = self.goal_node[1] * GRID_SIZE + GRID_SIZE // 2
        self.canvas.create_oval(gx-9, gy-9, gx+9, gy+9, fill=COLOR_GOAL, outline="")

        # Draw trajectory tracking cyan line arrays
        if len(self.planned_waypoint_path) > 1:
            pts = []
            for node in self.planned_waypoint_path:
                pts.append(node[0]*GRID_SIZE + GRID_SIZE//2)
                pts.append(node[1]*GRID_SIZE + GRID_SIZE//2)
            self.canvas.create_line(*pts, fill=COLOR_PATH, width=2, capstyle=tk.ROUND, joinstyle=tk.ROUND)

        # Render robot chassis layout icons
        rx, ry = self.robot_pos[0], self.robot_pos[1]
        self.canvas.create_oval(rx-11, ry-11, rx+11, ry+11, fill=COLOR_CAR, outline="")
        
        # Add heading nose vector projection line segment
        nx = rx + math.cos(self.robot_angle) * 11
        ny = ry + math.sin(self.robot_angle) * 11
        self.canvas.create_line(rx, ry, nx, ny, fill="#ffffff", width=3)

        # Draw active telemetry tracking indicators directly onto display canvases
        self.canvas.create_text(160, HEIGHT - 20, text=f"ROBOT STATE POS:({int(rx)},{int(ry)}) | PATH ENVELOPE COMPLIANT", 
                                font=("Courier New", 9, "bold"), fill="#ffffff")

if __name__ == "__main__":
    window = tk.Tk()
    application = NativeNavigationApp(window)
    window.mainloop()
