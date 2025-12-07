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
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 700
FPS = 60

NODE_RADIUS = 18          # visual radius of node circles
EDGE_BASE_WIDTH = 4       # base line width for edges
TRAFFIC_STEP = 50         # amount to change weight when clicking an edge
WEIGHT_LABEL_OFFSET_X = 6
WEIGHT_LABEL_OFFSET_Y = -10

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
# Positions (x, y) for nodes (edit to change layout)
nodes = {
    'A': (100, 500),
    'B': (260, 420),
    'C': (260, 580),
    'D': (420, 500),
    'E': (600, 420),
    'F': (600, 580),
    'G': (760, 500),
    'H': (880, 380),
    'I': (880, 620),
}

# Adjacency list (undirected)
adjacency = {
    'A': ['B', 'C'],
    'B': ['A', 'D', 'E'],
    'C': ['A', 'D', 'F'],
    'D': ['B', 'C', 'E', 'F'],
    'E': ['B', 'D', 'G', 'H'],
    'F': ['C', 'D', 'G', 'I'],
    'G': ['D', 'E', 'F', 'H', 'I'],
    'H': ['E', 'G'],
    'I': ['F', 'G']
}

# Function to compute Euclidean distance between two nodes
def euclid_distance(node1, node2):
    x1, y1 = nodes[node1]
    x2, y2 = nodes[node2]
    return math.hypot(x2 - x1, y2 - y1)

# Compute initial weights (Euclidean distance) and store each undirected edge once
weights = {}
for n1 in adjacency:
    for n2 in adjacency[n1]:
        edge = tuple(sorted([n1, n2]))
        if edge not in weights:
            weights[edge] = int(euclid_distance(n1, n2))


# Blocked sets
blocked_nodes = set()
blocked_edges = set()

# UI state
start_node = None
goal_node = None
current_path = None   # list of nodes from start to goal, or None
path_cost = None      # numeric cost or None

# =====================
# Helper functions (math, drawing, lookup)
# =====================


def draw_text(surface, text, x, y, color=(220,220,220)):
    img = font.render(text, True, color)
    surface.blit(img, (x, y))

def find_node_at_position(position):
    """Return node name if mouse is inside a node circle, else None."""
    mx, my = position
    for name, (x, y) in nodes.items():
        dx = mx - x
        dy = my - y
        if dx*dx + dy*dy <= NODE_RADIUS * NODE_RADIUS:
            return name
    return None

def find_edge_near_position(position, max_distance=12):
    """
    Return an edge tuple (u, v) if mouse is within max_distance of that edge segment.
    Edge keys match the 'weights' dictionary (sorted tuple).
    """
    mx, my = position
    best_edge = None
    best_dist = max_distance
    for edge, w in weights.items():
        u, v = edge
        x1, y1 = nodes[u]
        x2, y2 = nodes[v]
        dx = x2 - x1
        dy = y2 - y1
        # ignore degenerate edges
        if dx == 0 and dy == 0:
            continue
        # projection factor t of mouse point onto segment (0..1)
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
# A* implementation (clear & explicit, uses heapq for efficiency)
# Returns (path_list, cost) or (None, None) if no path
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
    # Basic checks
    if start is None or goal is None:
        return None, None
    if start in blocked_nodes or goal in blocked_nodes:
        return None, None

    # open_list is a min-heap of tuples: (f_score, g_score, node, parent)
    open_list = []
    initial_f = heuristic(start, goal) + 0  # g=0
    heapq.heappush(open_list, (initial_f, 0.0, start, None))

    # Bookkeeping
    came_from = {}            # came_from[node] = parent_node
    g_score = {start: 0.0}    # best known cost to reach node
    closed_set = set()        # nodes fully processed

    # Main loop
    while len(open_list) > 0:
        # Pop item with smallest f_score
        current_item = heapq.heappop(open_list)
        f_current = current_item[0]
        g_current = current_item[1]
        current_node = current_item[2]
        parent_node = current_item[3]

        # If we already found a better g_score for this node, skip this stale item
        if current_node in g_score and g_current > g_score[current_node]:
            continue

        # Record the parent (used later for path reconstruction)
        came_from[current_node] = parent_node

        # If we reached the goal, reconstruct the path
        if current_node == goal:
            return reconstruct_path(came_from, current_node), g_score.get(goal, None)

        # Mark current node as processed
        closed_set.add(current_node)

        # Examine neighbors
        for neighbor in adjacency.get(current_node, []):
            edge = tuple(sorted([current_node, neighbor]))

            # Skip neighbor if it's blocked or the edge is blocked
            if neighbor in closed_set:
                continue
            if neighbor in blocked_nodes or current_node in blocked_nodes:
                continue
            if edge in blocked_edges:
                continue

            # Tentative g cost if we go current -> neighbor
            tentative_g = g_score[current_node] + weights[edge]

            # If neighbor has no g_score yet, treat it as infinite
            previous_g = g_score.get(neighbor, float('inf'))

            # If this path to neighbor is better, record it
            if tentative_g < previous_g:
                g_score[neighbor] = tentative_g
                f_for_neighbor = tentative_g + heuristic(neighbor, goal)
                # push to heap: (f, g, node, parent)
                heapq.heappush(open_list, (f_for_neighbor, tentative_g, neighbor, current_node))

    # If open_list empties and we never returned, there's no path
    return None, None

# =====================
# Drawing function
# =====================
def draw_graph(path=None, path_cost_value=None):
    screen.fill((22, 24, 30))  # dark background

    # Draw edges
    for edge, w in weights.items():
        u, v = edge
        x1, y1 = nodes[u]
        x2, y2 = nodes[v]

        base_distance = euclid_distance(u, v)
        ratio = w / (base_distance + 1e-9)

        # choose color by traffic ratio
        if ratio < 1.6:
            color = (160, 160, 160)   # normal gray
        elif ratio < 2.6:
            color = (230, 200, 0)     # moderate (yellow)
        else:
            color = (200, 40, 40)     # heavy (red)

        # wider line for heavy ratio
        width = int(EDGE_BASE_WIDTH * min(4.0, max(1.0, ratio)))

        # if blocked edge, draw as darker thick line
        if edge in blocked_edges:
            pygame.draw.line(screen, (32, 32, 32), (x1, y1), (x2, y2), width + 4)
        else:
            pygame.draw.line(screen, color, (x1, y1), (x2, y2), width)

        # draw weight near middle
        mid_x = int((x1 + x2) / 2 + WEIGHT_LABEL_OFFSET_X)
        mid_y = int((y1 + y2) / 2 + WEIGHT_LABEL_OFFSET_Y)
        draw_text(screen, str(int(w)), mid_x, mid_y, (200, 200, 200))

    # Draw path highlight if exists (thick green line behind nodes)
    if path is not None and len(path) >= 2:
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            pygame.draw.line(screen, (10, 200, 90), nodes[a], nodes[b], 10)

    # Draw nodes
    for name, (x, y) in nodes.items():
        if name in blocked_nodes:
            color = (20, 20, 20)            # black-like for blocked
        elif name == start_node:
            color = (40, 200, 60)           # green for start
        elif name == goal_node:
            color = (50, 140, 255)          # blue for goal
        else:
            color = (200, 200, 200)         # light gray for normal nodes

        pygame.draw.circle(screen, color, (x, y), NODE_RADIUS)
        pygame.draw.circle(screen, (10, 10, 10), (x, y), NODE_RADIUS, 2)
        draw_text(screen, name, x - 6, y - 9, (10, 10, 10))

    # UI text area
    info_y = WINDOW_HEIGHT - 110
    cost_text = '-' if path_cost_value is None else str(int(path_cost_value))
    draw_text(screen, f"Start: {start_node}   Goal: {goal_node}   Cost: {cost_text}", 10, info_y)
    draw_text(screen, "LEFT click node = set START    RIGHT click node = set GOAL", 10, info_y + 20)
    draw_text(screen, "SHIFT + LEFT click node = toggle BLOCKED    Click near edge = change traffic", 10, info_y + 40)
    draw_text(screen, "ALT + click edge = decrease traffic    SPACE = compute path    R = reset    S = save", 10, info_y + 60)

    # If there is no path but both start and goal are set, show message
    if path is None and start_node is not None and goal_node is not None:
        draw_text(screen, "No path exists between start and goal!", 10, info_y + 80, (255, 100, 100))

    pygame.display.flip()

# =====================
# Save snapshot
# =====================
def save_snapshot():
    pygame.image.save(screen, "mini_gps_snapshot.png")

# =====================
# Main loop
# =====================
running = True
while running:
    clock.tick(FPS)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

            elif event.key == pygame.K_SPACE:
                # compute path safely
                result_path, result_cost = astar(start_node, goal_node)
                if result_path is None:
                    current_path = None
                    path_cost = None
                else:
                    current_path = result_path
                    path_cost = result_cost

            elif event.key == pygame.K_r:
                # reset everything
                start_node = None
                goal_node = None
                current_path = None
                path_cost = None

            elif event.key == pygame.K_s:
                save_snapshot()

        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            mods = pygame.key.get_mods()
            clicked_node = find_node_at_position(pos)
            clicked_edge = find_edge_near_position(pos)

            # SHIFT + left click: toggle blocked node
            if clicked_node is not None and (mods & pygame.KMOD_SHIFT):
                if clicked_node in blocked_nodes:
                    blocked_nodes.remove(clicked_node)
                else:
                    blocked_nodes.add(clicked_node)
                current_path = None
                path_cost = None

            # left click on node: set start
            elif event.button == 1 and clicked_node is not None and not (mods & pygame.KMOD_SHIFT):
                start_node = clicked_node
                current_path = None
                path_cost = None

            # right click on node: set goal
            elif event.button == 3 and clicked_node is not None:
                goal_node = clicked_node
                current_path = None
                path_cost = None

            # click near edge to change weight (ALT decreases)
            elif clicked_edge is not None and clicked_node is None:
                key = tuple(sorted(clicked_edge))
                if mods & pygame.KMOD_ALT:
                    weights[key] = max(1, weights[key] - TRAFFIC_STEP)
                else:
                    weights[key] = weights[key] + TRAFFIC_STEP
                current_path = None
                path_cost = None

    # draw everything
    draw_graph(current_path, path_cost)

# clean exit
pygame.quit()
sys.exit()
