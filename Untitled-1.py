"""
mini_gps_simple.py
Beginner-friendly Mini-GPS with A* pathfinding and interactive editing.

Features:
 - Visual map with nodes and edges
 - Click nodes to set START (left) and GOAL (right)
 - SHIFT+left-click a node to toggle it blocked (unusable)
 - Click near an edge to increase traffic weight (+50)
 - ALT+click near an edge to decrease traffic weight (-50, minimum 1)
 - SPACE: compute path (no animation)
 - R: reset start, goal, and path
 - S: save screenshot as mini_gps_snapshot.png
 - ESC or window close: quit
"""

import pygame
import math
import heapq
import sys

# =====================
# Configuration
# =====================
WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 700
FPS = 60

NODE_RADIUS = 40         # visual radius of node circles
EDGE_BASE_WIDTH = 4       # base line width for edges
TRAFFIC_STEP = 50         # amount to change weight when clicking an edge
WEIGHT_LABEL_OFFSET_X = 12
WEIGHT_LABEL_OFFSET_Y = -20

# =====================
# Initialize Pygame
# =====================
pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Mini GPS - Simple A*")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)

# =====================
# Graph definition
# =====================
nodes = {
    "Bank":     (100, 100),
    "Cafe":     (350, 120),
    "Club":     (600, 100),
    "Park":     (900, 130),
    "Mall":     (1200, 100),

    "School":   (150, 300),
    "Gym":      (450, 280),
    "Market":   (700, 300),
    "Hospital": (950, 280),
    "Cinema":   (1200, 300),

    "Library":  (400, 500),
    "Office":   (1000, 500),

    "Home":     (700, 600)   # new Home node
}

# Adjacency list (undirected)
adjacency = {
    "Bank": ["Cafe", "School"],
    "Cafe": ["Bank", "Club", "Gym"],
    "Club": ["Cafe", "Park", "Market"],
    "Park": ["Club", "Mall", "Hospital"],
    "Mall": ["Park", "Cinema"],

    "School": ["Bank", "Gym"],
    "Gym": ["School", "Cafe", "Market", "Library"],
    "Market": ["Gym", "Club", "Hospital", "Office", "Home"],
    "Hospital": ["Market", "Park", "Cinema"],
    "Cinema": ["Hospital", "Mall", "Office"],

    "Library": ["Gym", "Office"],
    "Office": ["Library", "Cinema", "Market"],

    "Home": ["Market"]  # connect Home logically
}

# =====================
# Compute initial weights (Euclidean distance) and store each undirected edge once
# =====================
def euclid_distance(node1, node2):
    x1, y1 = nodes[node1]
    x2, y2 = nodes[node2]
    return math.hypot(x2 - x1, y2 - y1)

weights = {}
for n1 in adjacency:
    for n2 in adjacency[n1]:
        edge = tuple(sorted([n1, n2]))
        if edge not in weights:
            weights[edge] = int(euclid_distance(n1, n2))

# =====================
# Blocked sets & UI state
# =====================
blocked_nodes = set()
blocked_edges = set()

start_node = None
goal_node = None
current_path = None
path_cost = None

# =====================
# Helper functions (drawing, lookup)
# =====================
def draw_text(surface, text, x, y, color=(220,220,220)):
    img = font.render(text, True, color)
    surface.blit(img, (x, y))

# =====================
# Node hit detection updated
# =====================
def find_node_at_position(position):
    mx, my = position
    for name, (x, y) in nodes.items():
        text_surface = font.render(name, True, (10,10,10))
        text_width = text_surface.get_width()
        text_height = text_surface.get_height()
        padding_x = 10
        padding_y = 6
        rect_width = text_width + 2*padding_x
        rect_height = text_height + 2*padding_y
        rect_x = x - rect_width//2
        rect_y = y - rect_height//2
        rect = pygame.Rect(rect_x, rect_y, rect_width, rect_height)
        if rect.collidepoint(mx, my):
            return name
    return None


def find_edge_near_position(position, max_distance=12):
    mx, my = position
    best_edge = None
    best_dist = max_distance
    for edge, w in weights.items():
        u, v = edge
        x1, y1 = nodes[u]
        x2, y2 = nodes[v]
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            continue
        t = ((mx - x1) * dx + (my - y1) * dy) / (dx*dx + dy*dy)
        t = max(0.0, min(1.0, t))
        px = x1 + t * dx
        py = y1 + t * dy
        dist = math.hypot(mx - px, my - py)
        if dist < best_dist:
            best_dist = dist
            best_edge = edge
    return best_edge

# =====================
# A* pathfinding
# =====================
def heuristic(node1, node2):
    return euclid_distance(node1, node2)

def reconstruct_path(came_from, current):
    path = []
    while current is not None:
        path.append(current)
        current = came_from.get(current, None)
    path.reverse()
    return path

def astar(start, goal):
    if start is None or goal is None or start in blocked_nodes or goal in blocked_nodes:
        return None, None
    open_list = []
    heapq.heappush(open_list, (heuristic(start, goal), 0.0, start, None))
    came_from = {}
    g_score = {start: 0.0}
    closed_set = set()

    while open_list:
        f_current, g_current, current_node, parent_node = heapq.heappop(open_list)
        if current_node in g_score and g_current > g_score[current_node]:
            continue
        came_from[current_node] = parent_node
        if current_node == goal:
            return reconstruct_path(came_from, current_node), g_score.get(goal)
        closed_set.add(current_node)
        for neighbor in adjacency.get(current_node, []):
            edge = tuple(sorted([current_node, neighbor]))
            if neighbor in closed_set or neighbor in blocked_nodes or current_node in blocked_nodes or edge in blocked_edges:
                continue
            tentative_g = g_score[current_node] + weights[edge]
            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                heapq.heappush(open_list, (tentative_g + heuristic(neighbor, goal), tentative_g, neighbor, current_node))
    return None, None

# =====================
# Draw graph
# =====================
# =====================
# Draw graph
# =====================
def draw_graph(path=None, path_cost_value=None):
    screen.fill((22,24,30))

    # Draw edges
    for edge, w in weights.items():
        u, v = edge
        x1, y1 = nodes[u]
        x2, y2 = nodes[v]
        ratio = w / (euclid_distance(u, v) + 1e-9)
        if ratio < 1.6:
            color = (160,160,160)
        elif ratio < 2.6:
            color = (230,200,0)
        else:
            color = (200,40,40)
        width = int(EDGE_BASE_WIDTH * min(4.0, max(1.0, ratio)))
        if edge in blocked_edges:
            pygame.draw.line(screen, (32,32,32), (x1,y1), (x2,y2), width+4)
        else:
            pygame.draw.line(screen, color, (x1,y1), (x2,y2), width)
        # Draw weight
        mid_x = int((x1+x2)/2 + WEIGHT_LABEL_OFFSET_X)
        mid_y = int((y1+y2)/2 + WEIGHT_LABEL_OFFSET_Y)
        draw_text(screen, str(int(w)), mid_x, mid_y, (200,200,200))

    # Highlight path
    if path is not None and len(path) >= 2:
        for i in range(len(path)-1):
            a, b = path[i], path[i+1]
            pygame.draw.line(screen, (10,200,90), nodes[a], nodes[b], 10)

    # Draw nodes as rectangles sized to text
    for name, (x, y) in nodes.items():
        if name in blocked_nodes:
            color = (20,20,20)
        elif name == start_node:
            color = (40,200,60)
        elif name == goal_node:
            color = (50,140,255)
        else:
            color = (200,200,200)

        # Text surface
        text_surface = font.render(name, True, (10,10,10))
        text_width = text_surface.get_width()
        text_height = text_surface.get_height()

        # Rectangle with padding
        padding_x = 10
        padding_y = 6
        rect_width = text_width + 2*padding_x
        rect_height = text_height + 2*padding_y
        rect_x = x - rect_width//2
        rect_y = y - rect_height//2
        rect = pygame.Rect(rect_x, rect_y, rect_width, rect_height)

        # Draw rectangle and border
        pygame.draw.rect(screen, color, rect)
        pygame.draw.rect(screen, (10,10,10), rect, 2)

        # Draw centered text
        screen.blit(text_surface, (x - text_width//2, y - text_height//2))

    # UI info text
    info_y = WINDOW_HEIGHT-110
    cost_text = '-' if path_cost_value is None else str(int(path_cost_value))
    draw_text(screen, f"Start: {start_node}   Goal: {goal_node}   Cost: {cost_text}", 10, info_y)
    draw_text(screen, "LEFT click node = set START    RIGHT click node = set GOAL", 10, info_y+20)
    draw_text(screen, "SHIFT + LEFT click node = toggle BLOCKED    Click near edge = change traffic", 10, info_y+40)
    draw_text(screen, "ALT + click edge = decrease traffic    SPACE = compute path    R = reset    S = save", 10, info_y+60)
    if path is None and start_node is not None and goal_node is not None:
        draw_text(screen, "No path exists between start and goal!", 10, info_y+80, (255,100,100))
    pygame.display.flip()
    
def save_snapshot():
    pygame.image.save(screen, "mini_gps_snapshot.png")

# =====================
# Main loop
# =====================
running = True
while running:
    clock.tick(FPS)
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running=False
        elif event.type==pygame.KEYDOWN:
            if event.key==pygame.K_ESCAPE:
                running=False
            elif event.key==pygame.K_SPACE:
                result_path, result_cost = astar(start_node, goal_node)
                current_path = result_path
                path_cost = result_cost
            elif event.key==pygame.K_r:
                start_node = None
                goal_node = None
                current_path = None
                path_cost = None
            elif event.key==pygame.K_s:
                save_snapshot()
        elif event.type==pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            mods = pygame.key.get_mods()
            clicked_node = find_node_at_position(pos)
            clicked_edge = find_edge_near_position(pos)
            if clicked_node is not None and (mods & pygame.KMOD_SHIFT):
                if clicked_node in blocked_nodes:
                    blocked_nodes.remove(clicked_node)
                else:
                    blocked_nodes.add(clicked_node)
                current_path = None
                path_cost = None
            elif event.button==1 and clicked_node is not None:
                start_node=clicked_node
                current_path=None
                path_cost=None
            elif event.button==3 and clicked_node is not None:
                goal_node=clicked_node
                current_path=None
                path_cost=None
            elif clicked_edge is not None and clicked_node is None:
                key=tuple(sorted(clicked_edge))
                if mods & pygame.KMOD_ALT:
                    weights[key]=max(1, weights[key]-TRAFFIC_STEP)
                else:
                    weights[key]+=TRAFFIC_STEP
                current_path=None
                path_cost=None
    draw_graph(current_path, path_cost)

pygame.quit()
sys.exit()
