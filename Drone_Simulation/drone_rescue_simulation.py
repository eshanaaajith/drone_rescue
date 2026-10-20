import pygame
import random
import sys
import heapq
import time
import numpy as np

# --- Constants ---
CELL_SIZE = 30
GRID_WIDTH = 25
GRID_HEIGHT = 15
STATS_PANEL_WIDTH = 250
WINDOW_WIDTH = CELL_SIZE * GRID_WIDTH + STATS_PANEL_WIDTH
WINDOW_HEIGHT = CELL_SIZE * GRID_HEIGHT + 150  # Extra space for event log
FPS = 10

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
RED = (255, 50, 50)
GREEN = (50, 255, 50)
BLUE = (50, 50, 255)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
ORANGE = (255, 165, 0)
DARK_GRAY = (50, 50, 50)

# Zones for Search Drones
ZONE_1_BOUNDS = (0, 0, GRID_WIDTH // 3, GRID_HEIGHT)
ZONE_2_BOUNDS = (GRID_WIDTH // 3, 0, 2 * GRID_WIDTH // 3, GRID_HEIGHT)
ZONE_3_BOUNDS = (2 * GRID_WIDTH // 3, 0, GRID_WIDTH, GRID_HEIGHT)

class Node:
    def __init__(self, x, y, cost=0, heuristic=0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent = parent
        
    @property
    def f(self):
        return self.cost + self.heuristic

    def __lt__(self, other):
        return self.f < other.f

def get_neighbors(x, y, grid):
    neighbors = []
    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT and grid[ny][nx] != 1:
            neighbors.append((nx, ny))
    return neighbors

def heuristic(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

def astar(start_x, start_y, goal_x, goal_y, grid):
    start_node = Node(start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y))
    open_list = []
    heapq.heappush(open_list, start_node)
    closed_set = set()
    
    while open_list:
        current = heapq.heappop(open_list)
        
        if current.x == goal_x and current.y == goal_y:
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]
            
        closed_set.add((current.x, current.y))
        
        for nx, ny in get_neighbors(current.x, current.y, grid):
            if (nx, ny) in closed_set:
                continue
                
            new_cost = current.cost + 1
            new_node = Node(nx, ny, new_cost, heuristic(nx, ny, goal_x, goal_y), current)
            
            better_found = False
            for item in open_list:
                if item.x == nx and item.y == ny and item.cost <= new_cost:
                    better_found = True
                    break
            
            if not better_found:
                heapq.heappush(open_list, new_node)
                
    return None

class SearchDrone:
    def __init__(self, id, bounds, color, grid):
        self.id = id
        self.bounds = bounds
        # FIX: spawn only on a free cell (not hazard / not on top of a victim)
        while True:
            self.x = random.randint(bounds[0], bounds[2] - 1)
            self.y = random.randint(bounds[1], bounds[3] - 1)
            if grid[self.y][self.x] == 0:
                break
        self.done = False
        self.color = color
        self.visited = set()
        self.visited.add((self.x, self.y))
        self.path_to_frontier = []
        
    def move(self, grid):
        frontiers = set()
        for vx, vy in self.visited:
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = vx + dx, vy + dy
                if self.bounds[0] <= nx < self.bounds[2] and self.bounds[1] <= ny < self.bounds[3]:
                    if grid[ny][nx] != 1 and (nx, ny) not in self.visited:
                        frontiers.add((nx, ny))
        
        self.done = not frontiers
        if not frontiers:
            return
            
        if self.path_to_frontier:
            next_pos = self.path_to_frontier[0]
            # FIX: a hazard shifted onto the cached path -> drop it and replan next tick
            if grid[next_pos[1]][next_pos[0]] == 1:
                self.path_to_frontier = []
                return
            self.path_to_frontier.pop(0)
            self.x, self.y = next_pos
            self.visited.add(next_pos)
            return
            
        closest_frontier = None
        min_dist = float('inf')
        for fx, fy in frontiers:
            dist = heuristic(self.x, self.y, fx, fy)
            if dist < min_dist:
                min_dist = dist
                closest_frontier = (fx, fy)
                
        if closest_frontier:
            path = astar(self.x, self.y, closest_frontier[0], closest_frontier[1], grid)
            if path and len(path) > 1:
                self.path_to_frontier = path[1:]
                next_pos = self.path_to_frontier.pop(0)
                self.x, self.y = next_pos
                self.visited.add(next_pos)
            else:
                self.visited.add(closest_frontier)

class ResponseDrone:
    def __init__(self, start_x, start_y):
        self.x = start_x
        self.y = start_y
        self.path = []
        self.target_victim = None
        self.state = "IDLE"
        
    def move(self, grid, goal):
        """Advance one step. Returns 'REPLANNED', 'WAITING' or None."""
        if not self.path:
            return None
        nx, ny = self.path[0]
        status = None
        # FIX: a hazard shifted onto the next cell -> replan with A* (or wait)
        if grid[ny][nx] == 1:
            new_path = astar(self.x, self.y, goal[0], goal[1], grid)
            if new_path and len(new_path) > 1:
                self.path = new_path[1:]
                nx, ny = self.path[0]
                status = "REPLANNED"
            else:
                return "WAITING"
        self.path.pop(0)
        self.x, self.y = nx, ny
        return status

class Simulation:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Disaster Rescue Drone Swarm")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 18)
        self.header_font = pygame.font.SysFont("consolas", 22, bold=True)
        
        self.grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]
        self.base_station = (GRID_WIDTH // 2, GRID_HEIGHT - 1)
        
        # Generate Hazards
        self.num_hazards = 45
        for _ in range(self.num_hazards):
            hx = random.randint(0, GRID_WIDTH - 1)
            hy = random.randint(0, GRID_HEIGHT - 1)
            self.grid[hy][hx] = 1
            
        self.grid[self.base_station[1]][self.base_station[0]] = 0  # base is always free
        
        # Generate Victims
        self.victims = []
        self.num_victims = 6
        for _ in range(self.num_victims):
            while True:
                vx = random.randint(0, GRID_WIDTH - 1)
                vy = random.randint(0, GRID_HEIGHT - 1)
                if self.grid[vy][vx] == 0 and (vx, vy) != self.base_station:  # FIX: never on base
                    self.grid[vy][vx] = 2
                    self.victims.append((vx, vy))
                    break
                    
        self.victims_found = set()
        self.victims_rescued = set()
        
        self.search_drones = [
            SearchDrone("Alpha", ZONE_1_BOUNDS, CYAN, self.grid),
            SearchDrone("Beta", ZONE_2_BOUNDS, ORANGE, self.grid),
            SearchDrone("Gamma", ZONE_3_BOUNDS, (255, 100, 255), self.grid)
        ]
        self.response_drones = [
            ResponseDrone(self.base_station[0], self.base_station[1]),
            ResponseDrone(self.base_station[0], self.base_station[1])
        ]
        
        self.logs = ["Simulation Started.", "Search drones deployed."]
        self.step_count = 0
        self.collisions = 0
        self.replans = 0
        self.mission_complete = False
        self.unreachable_warned = set()
        
    def log_event(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.logs.append(f"[{timestamp}] {message}")
        if len(self.logs) > 6:
            self.logs.pop(0)
            
    def update(self):
        self.step_count += 1
        occupied = {(d.x, d.y) for d in self.search_drones + self.response_drones}
        if random.random() < 0.1:
            hazard_cells = [(x, y) for y in range(GRID_HEIGHT) for x in range(GRID_WIDTH) if self.grid[y][x] == 1]
            if hazard_cells:
                hx, hy = random.choice(hazard_cells)
                neighbors = []
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    nx, ny = hx + dx, hy + dy
                    if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT and self.grid[ny][nx] == 0:
                        if (nx, ny) != self.base_station and (nx, ny) not in occupied:  # FIX
                            neighbors.append((nx, ny))
                if neighbors:
                    nx, ny = random.choice(neighbors)
                    self.grid[hy][hx] = 0
                    self.grid[ny][nx] = 1

        for drone in self.search_drones:
            drone.move(self.grid)
            
            if self.grid[drone.y][drone.x] == 2 and (drone.x, drone.y) not in self.victims_found:
                self.victims_found.add((drone.x, drone.y))
                self.log_event(f"Search Drone {drone.id} found victim at ({drone.x}, {drone.y})")
                
        active_targets = [d.target_victim for d in self.response_drones if d.state == "RESPONDING"]
        pending_victims = [v for v in self.victims_found if v not in self.victims_rescued and v not in active_targets]
        
        for drone in self.response_drones:
            if drone.state == "IDLE":
                # nearest-victim-first; skip unreachable ones instead of blocking the queue
                for target in sorted(pending_victims, key=lambda v: heuristic(drone.x, drone.y, v[0], v[1])):
                    path = astar(drone.x, drone.y, target[0], target[1], self.grid)
                    if path:
                        drone.target_victim = target
                        drone.path = path[1:]
                        drone.state = "RESPONDING"
                        pending_victims.remove(target)
                        self.log_event(f"Base assigned Response Drone to ({target[0]}, {target[1]})")
                        break
                    elif target not in self.unreachable_warned:
                        self.unreachable_warned.add(target)
                        self.log_event(f"No safe route to victim at ({target[0]}, {target[1]})!")
                    
            elif drone.state == "RESPONDING":
                if drone.path:
                    if drone.move(self.grid, drone.target_victim) == "REPLANNED":
                        self.replans += 1
                        self.log_event("Hazard on path - route replanned (A*).")
                else:
                    target = drone.target_victim
                    if target and (drone.x, drone.y) == target:
                        self.victims_rescued.add(target)
                        self.grid[target[1]][target[0]] = 0
                        self.log_event(f"Medicine delivered to ({target[0]}, {target[1]}). Ambulance called.")
                        drone.state = "RETURNING"
                        path = astar(drone.x, drone.y, self.base_station[0], self.base_station[1], self.grid)
                        drone.path = path[1:] if path else []
                        
            elif drone.state == "RETURNING":
                if drone.path:
                    if drone.move(self.grid, self.base_station) == "REPLANNED":
                        self.replans += 1
                        self.log_event("Hazard on path - route replanned (A*).")
                elif (drone.x, drone.y) == self.base_station:
                    drone.state = "IDLE"
                    drone.target_victim = None
                    self.log_event("Response Drone returned to base.")
                else:  # FIX: no route yet -> retry instead of falsely reporting 'returned'
                    path = astar(drone.x, drone.y, self.base_station[0], self.base_station[1], self.grid)
                    if path:
                        drone.path = path[1:]
        
        # Safety audit: count any drone that ends a step on a hazard cell (should stay 0)
        for d in self.search_drones + self.response_drones:
            if self.grid[d.y][d.x] == 1:
                self.collisions += 1
        if (not self.mission_complete and len(self.victims_rescued) == self.num_victims
                and all(d.state == "IDLE" for d in self.response_drones)):
            self.mission_complete = True
            self.log_event(f"MISSION COMPLETE in {self.step_count} steps.")
        
    def draw(self):
        self.screen.fill(WHITE)
        
        # Zones
        pygame.draw.rect(self.screen, (240, 255, 255), (ZONE_1_BOUNDS[0]*CELL_SIZE, ZONE_1_BOUNDS[1]*CELL_SIZE, (ZONE_1_BOUNDS[2]-ZONE_1_BOUNDS[0])*CELL_SIZE, (ZONE_1_BOUNDS[3]-ZONE_1_BOUNDS[1])*CELL_SIZE))
        pygame.draw.rect(self.screen, (255, 245, 230), (ZONE_2_BOUNDS[0]*CELL_SIZE, ZONE_2_BOUNDS[1]*CELL_SIZE, (ZONE_2_BOUNDS[2]-ZONE_2_BOUNDS[0])*CELL_SIZE, (ZONE_2_BOUNDS[3]-ZONE_2_BOUNDS[1])*CELL_SIZE))
        pygame.draw.rect(self.screen, (255, 240, 255), (ZONE_3_BOUNDS[0]*CELL_SIZE, ZONE_3_BOUNDS[1]*CELL_SIZE, (ZONE_3_BOUNDS[2]-ZONE_3_BOUNDS[0])*CELL_SIZE, (ZONE_3_BOUNDS[3]-ZONE_3_BOUNDS[1])*CELL_SIZE))
        
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(self.screen, GRAY, rect, 1)
                
                if self.grid[y][x] == 1:
                    pygame.draw.rect(self.screen, RED, rect)
                elif self.grid[y][x] == 2:
                    if int(time.time() * 2) % 2 == 0:
                        pygame.draw.rect(self.screen, RED, rect)
                        pygame.draw.rect(self.screen, WHITE, (x*CELL_SIZE + 10, y*CELL_SIZE + 4, 10, 22))
                        pygame.draw.rect(self.screen, WHITE, (x*CELL_SIZE + 4, y*CELL_SIZE + 10, 22, 10))
                        
        base_rect = pygame.Rect(self.base_station[0] * CELL_SIZE, self.base_station[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE)
        pygame.draw.rect(self.screen, DARK_GRAY, base_rect)
        pygame.draw.rect(self.screen, GREEN, base_rect, 2)
        base_text = self.font.render("B", True, GREEN)
        self.screen.blit(base_text, (self.base_station[0] * CELL_SIZE + 10, self.base_station[1] * CELL_SIZE + 5))
        
        for drone in self.search_drones:
            pygame.draw.circle(self.screen, drone.color, (drone.x * CELL_SIZE + CELL_SIZE//2, drone.y * CELL_SIZE + CELL_SIZE//2), CELL_SIZE//2 - 2)
            pygame.draw.circle(self.screen, BLACK, (drone.x * CELL_SIZE + CELL_SIZE//2, drone.y * CELL_SIZE + CELL_SIZE//2), CELL_SIZE//2 - 2, 1)
            text = self.font.render("S", True, BLACK)
            self.screen.blit(text, (drone.x * CELL_SIZE + 10, drone.y * CELL_SIZE + 5))
            
        for drone in self.response_drones:
            if drone.path:
                path_points = [(drone.x * CELL_SIZE + CELL_SIZE//2, drone.y * CELL_SIZE + CELL_SIZE//2)]
                path_points += [(px * CELL_SIZE + CELL_SIZE//2, py * CELL_SIZE + CELL_SIZE//2) for px, py in drone.path]
                if len(path_points) > 1:
                    pygame.draw.lines(self.screen, BLUE, False, path_points, 3)
                
            pygame.draw.circle(self.screen, BLUE, (drone.x * CELL_SIZE + CELL_SIZE//2, drone.y * CELL_SIZE + CELL_SIZE//2), CELL_SIZE//2)
            pygame.draw.circle(self.screen, WHITE, (drone.x * CELL_SIZE + CELL_SIZE//2, drone.y * CELL_SIZE + CELL_SIZE//2), CELL_SIZE//2, 2)
            text = self.font.render("R", True, WHITE)
            self.screen.blit(text, (drone.x * CELL_SIZE + 10, drone.y * CELL_SIZE + 5))
        
        # Log Panel
        pygame.draw.rect(self.screen, (245, 245, 245), (0, GRID_HEIGHT * CELL_SIZE, GRID_WIDTH * CELL_SIZE, 150))
        pygame.draw.line(self.screen, BLACK, (0, GRID_HEIGHT * CELL_SIZE), (GRID_WIDTH * CELL_SIZE, GRID_HEIGHT * CELL_SIZE), 2)
        
        header_text = self.header_font.render("Event Log:", True, BLACK)
        self.screen.blit(header_text, (10, GRID_HEIGHT * CELL_SIZE + 10))
        
        log_y = GRID_HEIGHT * CELL_SIZE + 35
        for log in self.logs:
            text = self.font.render(log, True, (40, 40, 40))
            self.screen.blit(text, (10, log_y))
            log_y += 18
            
        # Stats Panel
        stats_x = GRID_WIDTH * CELL_SIZE
        pygame.draw.rect(self.screen, (235, 235, 235), (stats_x, 0, STATS_PANEL_WIDTH, WINDOW_HEIGHT))
        pygame.draw.line(self.screen, BLACK, (stats_x, 0), (stats_x, WINDOW_HEIGHT), 2)
        
        stats_header = self.header_font.render("Statistics", True, BLACK)
        self.screen.blit(stats_header, (stats_x + 10, 10))
        
        stats_texts = [
            f"Victims:       {self.num_victims}",
            f"Found:         {len(self.victims_found)}",
            f"Rescued:       {len(self.victims_rescued)}",
            "",
            f"Search Drones: {len(self.search_drones)}",
            f"Response:      {len(self.response_drones)}",
            "",
            "Status:"
        ]
        
        for drone in self.search_drones:
            stats_texts.append(f"{drone.id:8} {'SECTOR DONE' if drone.done else 'SEARCHING'}")
            
        for i, drone in enumerate(self.response_drones):
            stats_texts.append(f"Resp {i+1}:   {drone.state}")
        stats_texts.append("")
        stats_texts.append(f"Steps:      {self.step_count}")
        stats_texts.append(f"Collisions: {self.collisions}")
        stats_texts.append(f"Replans:    {self.replans}")
        stats_texts.append("MISSION COMPLETE" if self.mission_complete else "Mission: IN PROGRESS")
            
        sy = 45
        for st in stats_texts:
            surf = self.font.render(st, True, (40, 40, 40))
            self.screen.blit(surf, (stats_x + 10, sy))
            sy += 22
            
        pygame.display.flip()
        
    def run(self):
        running = True
        paused = False   # press SPACE to pause / resume (handy for explaining the demo)
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                    paused = not paused
                    
            if not paused:
                self.update()
            self.draw()
            self.clock.tick(FPS)
            
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    # Optional: python drone_rescue_simulation_fixed.py 42  -> repeatable demo
    if len(sys.argv) > 1:
        random.seed(int(sys.argv[1]))
    sim = Simulation()
    sim.run()
