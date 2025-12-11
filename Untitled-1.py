"""
Mini Egypt Map GPS (with A*)
Each node = Egyptian Governorate
Left click: set START
Right click: set GOAL
SHIFT + left click: toggle blocked node
Click near edge = increase traffic
ALT + click edge = reduce traffic
SPACE = compute path
R = reset
S = save snapshot
"""

import pygame
import math
import heapq
import sys

# =====================
# Configuration
# =====================
WINDOW_WIDTH = 1600
WINDOW_HEIGHT = 1050
FPS = 60

NODE_RADIUS = 25
EDGE_BASE_WIDTH = 4
TRAFFIC_STEP = 50
WEIGHT_LABEL_OFFSET_X = 6
WEIGHT_LABEL_OFFSET_Y = -10

# =====================
# Initialize Pygame
# =====================
pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Mini Egypt GPS - A* Pathfinding")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)

# =====================
# Egyptian Map Nodes (Scaled + Spread)
# =====================

def S(x): return int(x * 1.4 + 80)
def T(y): return int(y * 1.4 + 40)

nodes = {
    # Delta (Top)
    "Alex": (S(200), T(120)),
    "Beheira": (S(260), T(160)),
    "KafrElSheikh": (S(300), T(120)),
    "Gharbia": (S(330), T(160)),
    "Monufia": (S(300), T(200)),
    "Dakahliya": (S(380), T(140)),
    "Damietta": (S(430), T(110)),
    "Sharqia": (S(420), T(200)),
    "PortSaid": (S(480), T(120)),

    # Cairo region
    "Qalyubia": (S(340), T(240)),
    "Cairo": (S(360), T(280)),
    "Giza": (S(300), T(300)),
    "Suez": (S(460), T(260)),
    "Ismailia": (S(480), T(220)),

    # Upper Egypt
    "BeniSuef": (S(330), T(360)),
    "Fayoum": (S(280), T(360)),
    "Minya": (S(340), T(420)),
    "Asyut": (S(340), T(480)),
    "Sohag": (S(340), T(540)),
    "Qena": (S(340), T(600)),
    "Luxor": (S(340), T(650)),
    "Aswan": (S(340), T(700)),

    # Sinai
    "NorthSinai": (S(560), T(180)),
    "SouthSinai": (S(560), T(300)),

    # Red Sea
    "RedSea": (S(480), T(400)),
    "Hurghada": (S(500), T(500)),
}

# =====================
# Egyptian Roads (Adjacency)
# =====================

adjacency = {
    "Alex": ["Beheira"],
    "Beheira": ["Alex", "KafrElSheikh", "Gharbia", "Monufia"],
    "KafrElSheikh": ["Beheira", "Gharbia", "Dakahliya"],
    "Gharbia": ["Beheira", "KafrElSheikh", "Dakahliya", "Monufia", "Qalyubia"],
    "Monufia": ["Beheira", "Gharbia", "Qalyubia", "Cairo"],
    "Dakahliya": ["KafrElSheikh", "Gharbia", "Damietta", "Sharqia", "PortSaid"],
    "Damietta": ["Dakahliya", "PortSaid"],
    "Sharqia": ["Dakahliya", "Qalyubia", "Ismailia"],
    "PortSaid": ["Damietta", "Dakahliya", "Ismailia"],

    "Qalyubia": ["Gharbia", "Monufia", "Cairo", "Sharqia"],
    "Cairo": ["Qalyubia", "Giza", "Suez", "Ismailia"],
    "Giza": ["Cairo", "Fayoum", "BeniSuef"],
    "Suez": ["Cairo", "Ismailia", "SouthSinai"],
    "Ismailia": ["Sharqia", "Cairo", "Suez", "PortSaid", "NorthSinai"],

    "BeniSuef": ["Giza", "Fayoum", "Minya"],
    "Fayoum": ["Giza", "BeniSuef"],
    "Minya": ["BeniSuef", "Asyut"],
    "Asyut": ["Minya", "Sohag"],
    "Sohag": ["Asyut", "Qena"],
    "Qena": ["Sohag", "Luxor", "Hurghada"],
    "Luxor": ["Qena", "Aswan"],
    "Aswan": ["Luxor"],

    "NorthSinai": ["Ismailia", "SouthSinai"],
    "SouthSinai": ["NorthSinai", "Suez", "Hurghada"],

    "RedSea": ["Hurghada"],
    "Hurghada": ["RedSea", "Qena", "SouthSinai"]
}

# =====================
# Weight calculation
# =====================

def euclid_distance(n1, n2):
    x1, y1 = nodes[n1]
    x2, y2 = nodes[n2]
    return math.hypot(x2 - x1, y2 - y1)

weights = {}
for a in adjacency:
    for b in adjacency[a]:
        edge = tuple(sorted([a, b]))
        if edge not in weights:
            weights[edge] = int(euclid_distance(a, b))

# =====================
# States
# =====================

blocked_nodes = set()
blocked_edges = set()
start_node = None
goal_node = None
current_path = None
path_cost = None

# =====================
# Helpers
# =====================

def draw_text(surface, text, x, y, color=(230, 230, 230)):
    img = font.render(text, True, color)
    surface.blit(img, (x, y))

def find_node_at_pos(pos):
    mx, my = pos
    for name, (x, y) in nodes.items():
        if (mx - x) ** 2 + (my - y) ** 2 <= NODE_RADIUS ** 2:
            return name
    return None

def find_edge_near_pos(pos, max_dist=12):
    mx, my = pos
    best = None
    best_d = max_dist
    for (u, v), w in weights.items():
        x1, y1 = nodes[u]
        x2, y2 = nodes[v]
        dx, dy = x2 - x1, y2 - y1
        if dx == 0 and dy == 0:
            continue
        t = ((mx-x1)*dx + (my-y1)*dy) / (dx*dx + dy*dy)
        t = max(0, min(1, t))
        px = x1 + t*dx
        py = y1 + t*dy
        dist = math.hypot(mx - px, my - py)
        if dist < best_d:
            best_d = dist
            best = (u, v)
    if best:
        return tuple(sorted(best))
    return None

# =====================
# A* pathfinding
# =====================

def heuristic(a, b):
    return euclid_distance(a, b)

def astar(start, goal):
    if start is None or goal is None:
        return None, None
    if start in blocked_nodes or goal in blocked_nodes:
        return None, None

    pq = []
    heapq.heappush(pq, (heuristic(start, goal), 0, start, None))
    came = {}
    g = {start: 0}

    while pq:
        f, gc, node, parent = heapq.heappop(pq)

        if node in g and gc > g[node]:
            continue

        came[node] = parent

        if node == goal:
            path = []
            n = node
            while n is not None:
                path.append(n)
                n = came[n]
            return list(reversed(path)), g[goal]

        for nei in adjacency[node]:
            edge = tuple(sorted([node, nei]))
            if nei in blocked_nodes or edge in blocked_edges:
                continue

            cost = g[node] + weights[edge]
            if cost < g.get(nei, float("inf")):
                g[nei] = cost
                heapq.heappush(pq, (cost + heuristic(nei, goal), cost, nei, node))

    return None, None

# =====================
# Drawing
# =====================

def draw_graph(path=None, cost=None):
    screen.fill((20, 22, 30))

    # edges
    for (u, v), w in weights.items():
        x1, y1 = nodes[u]
        x2, y2 = nodes[v]

        base = euclid_distance(u, v)
        ratio = w / (base + 0.0001)

        if ratio < 1.6:
            color = (150, 150, 150)
        elif ratio < 2.6:
            color = (230, 200, 0)
        else:
            color = (200, 40, 40)

        width = int(EDGE_BASE_WIDTH * min(4, max(1, ratio)))

        if (u, v) in blocked_edges:
            pygame.draw.line(screen, (30, 30, 30), (x1, y1), (x2, y2), width + 4)
        else:
            pygame.draw.line(screen, color, (x1, y1), (x2, y2), width)

        # weight label
        mx = (x1 + x2) // 2 + WEIGHT_LABEL_OFFSET_X
        my = (y1 + y2) // 2 + WEIGHT_LABEL_OFFSET_Y
        draw_text(screen, str(w), mx, my)

    # path
    if path and len(path) > 1:
        for i in range(len(path)-1):
            pygame.draw.line(screen, (10, 200, 90), nodes[path[i]], nodes[path[i+1]], 10)

    # nodes
    for name, (x, y) in nodes.items():
        if name in blocked_nodes:
            color = (30, 30, 30)
        elif name == start_node:
            color = (40, 200, 60)
        elif name == goal_node:
            color = (60, 140, 255)
        else:
            color = (220, 220, 220)

        pygame.draw.circle(screen, color, (x, y), NODE_RADIUS)
        pygame.draw.circle(screen, (10, 10, 10), (x, y), NODE_RADIUS, 2)
        draw_text(screen, name, x - 20, y - 8, (10, 10, 10))

    info_y = WINDOW_HEIGHT - 120
    draw_text(screen, f"Start: {start_node}   Goal: {goal_node}   Cost: {cost if cost else '-'}", 10, info_y)
    draw_text(screen, "Left click: START | Right click: GOAL | SHIFT+click: block node", 10, info_y + 20)
    draw_text(screen, "Click edge: +traffic | ALT+click: -traffic | SPACE: path | R: reset | S: save", 10, info_y + 40)

    pygame.display.flip()

# =====================
# Save Snapshot
# =====================

def save_snapshot():
    pygame.image.save(screen, "egypt_gps_snapshot.png")

# =====================
# Main Loop
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
                current_path, path_cost = astar(start_node, goal_node)
            elif event.key == pygame.K_r:
                start_node = goal_node = None
                current_path = None
                path_cost = None
            elif event.key == pygame.K_s:
                save_snapshot()

        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            mods = pygame.key.get_mods()

            node = find_node_at_pos(pos)
            edge = find_edge_near_pos(pos)

            # Shift = block/unblock node
            if node and (mods & pygame.KMOD_SHIFT):
                if node in blocked_nodes:
                    blocked_nodes.remove(node)
                else:
                    blocked_nodes.add(node)
                current_path = None
                path_cost = None

            # Left click = start
            elif event.button == 1 and node:
                start_node = node
                current_path = None
                path_cost = None

            # Right click = goal
            elif event.button == 3 and node:
                goal_node = node
                current_path = None
                path_cost = None

            # Traffic change on edges
            elif edge:
                if mods & pygame.KMOD_ALT:
                    weights[edge] = max(1, weights[edge] - TRAFFIC_STEP)
                else:
                    weights[edge] += TRAFFIC_STEP
                current_path = None
                path_cost = None

    draw_graph(current_path, path_cost)

pygame.quit()
sys.exit()
