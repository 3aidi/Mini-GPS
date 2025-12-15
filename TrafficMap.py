import pygame
import math
import heapq
import sys


WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 700
FPS = 60
EDGE_BASE_WIDTH = 4       
TRAFFIC_STEP = 50         
WEIGHT_LABEL_OFFSET_X = 12
WEIGHT_LABEL_OFFSET_Y = -20
TRAFFIC_NORMAL_THRESHOLD = 1.6
TRAFFIC_MODERATE_THRESHOLD = 2.6
TRAFFIC_MAX_VISUAL_RATIO = 4.0
NODE_PADDING_X = 10
NODE_PADDING_Y = 6
EDGE_CLICK_MAX_DISTANCE = 12
EPSILON = 1e-9  


pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Mini GPS - Simple A*")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)


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

    "Home":     (700, 600)
}


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
    "Home": ["Market"] 
}


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


blocked_nodes = set()
blocked_edges = set()

start_node = None
goal_node = None
current_path = None
path_cost = None
path_attempted = False


# (drawing, lookup)
def draw_text(surface, text, x, y, color=(220,220,220)):
    img = font.render(text, True, color)
    surface.blit(img, (x, y))

def reset_path():
    global current_path, path_cost, path_attempted
    current_path = None
    path_cost = None
    path_attempted = False

def get_node_rect(name, x, y):
    """Calculate rectangle and text surface for a node label."""
    text_surface = font.render(name, True, (10,10,10))
    text_width = text_surface.get_width()
    text_height = text_surface.get_height()
    rect_width = text_width + 2 * NODE_PADDING_X
    rect_height = text_height + 2 * NODE_PADDING_Y
    rect = pygame.Rect(x - rect_width//2, y - rect_height//2, rect_width, rect_height)
    return rect, text_surface

# Node hit detection
def find_node_at_position(position):
    mx, my = position
    for name, (x, y) in nodes.items():
        rect, _ = get_node_rect(name, x, y)
        if rect.collidepoint(mx, my):
            return name
    return None


def find_edge_near_position(position, max_distance=EDGE_CLICK_MAX_DISTANCE):
    mx, my = position
    best_edge = None
    best_dist = max_distance
    for edge, weight in weights.items():
        node1, node2 = edge
        x1, y1 = nodes[node1]
        x2, y2 = nodes[node2]
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

# A* pathfinding
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
    
    if start == goal:
        return [start], 0
    
    open_list = []
    heapq.heappush(open_list, (heuristic(start, goal), 0.0, start, None))
    came_from = {}
    g_score = {start: 0.0}
    closed_set = set()

    while open_list:
        f_current, g_current, current_node, parent_node = heapq.heappop(open_list)
        
        # Skip if we've found a better path to this node already
        if current_node in g_score and g_current > g_score[current_node]:
            continue
            
        came_from[current_node] = parent_node
        
        if current_node == goal:
            return reconstruct_path(came_from, current_node), g_score.get(goal)
        
        closed_set.add(current_node)
        
        for neighbor in adjacency.get(current_node, []):
            edge = tuple(sorted([current_node, neighbor]))
            
            if neighbor in closed_set or neighbor in blocked_nodes or edge in blocked_edges:
                continue
            
            # Calculate tentative g-score: cost from start to neighbor through current
            tentative_g = g_score[current_node] + weights[edge]
            
            # If this path is better than any previous path to neighbor
            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score, tentative_g, neighbor, current_node))
    
    return None, None


def draw_edges(path=None):
    for edge, weight in weights.items():
        node1, node2 = edge
        x1, y1 = nodes[node1]
        x2, y2 = nodes[node2]
        
        # Calculate traffic ratio (weight vs original distance)
        ratio = weight / (euclid_distance(node1, node2) + EPSILON)
        
        if ratio < TRAFFIC_NORMAL_THRESHOLD:
            color = (160, 160, 160)  
        elif ratio < TRAFFIC_MODERATE_THRESHOLD:
            color = (230, 200, 0)     
        else:
            color = (200, 40, 40)    
        
        width = int(EDGE_BASE_WIDTH * min(TRAFFIC_MAX_VISUAL_RATIO, max(1.0, ratio)))
        
        # Draw blocked edges darker
        if edge in blocked_edges:
            pygame.draw.line(screen, (32, 32, 32), (x1, y1), (x2, y2), width + 4)
        else:
            pygame.draw.line(screen, color, (x1, y1), (x2, y2), width)
        
        # Draw weight label
        mid_x = int((x1 + x2) / 2 + WEIGHT_LABEL_OFFSET_X)
        mid_y = int((y1 + y2) / 2 + WEIGHT_LABEL_OFFSET_Y)
        draw_text(screen, str(int(weight)), mid_x, mid_y, (200, 200, 200))
    
    # Highlight the found path in green
    if path and len(path) >= 2:
        for i in range(len(path) - 1):
            from_node, to_node = path[i], path[i + 1]
            pygame.draw.line(screen, (10, 200, 90), nodes[from_node], nodes[to_node], 10)

def draw_nodes():
    for name, (x, y) in nodes.items():
        if name in blocked_nodes:
            color = (20, 20, 20)      
        elif name == start_node:
            color = (40, 200, 60)     
        elif name == goal_node:
            color = (50, 140, 255)    
        else:
            color = (200, 200, 200)   
        
        # Get rectangle and text surface
        rect, text_surface = get_node_rect(name, x, y)
        
        # Draw filled rectangle with border
        pygame.draw.rect(screen, color, rect)
        pygame.draw.rect(screen, (10, 10, 10), rect, 2)
        
        # Draw centered text
        text_width = text_surface.get_width()
        text_height = text_surface.get_height()
        screen.blit(text_surface, (x - text_width // 2, y - text_height // 2))

def draw_ui_info(path_cost_value=None):
    info_y = WINDOW_HEIGHT - 110
    cost_text = '-' if path_cost_value is None else str(int(path_cost_value))
    
    draw_text(screen, f"Start: {start_node}   Goal: {goal_node}   Cost: {cost_text}", 10, info_y)
    draw_text(screen, "LEFT click node = set START    RIGHT click node = set GOAL", 10, info_y + 20)
    draw_text(screen, "SHIFT + LEFT click node = toggle BLOCKED    CTRL + click edge = toggle BLOCKED edge", 10, info_y + 40)
    draw_text(screen, "Click edge = increase traffic    ALT + click edge = decrease traffic    SPACE = compute    R = reset", 10, info_y + 60)
    
    if path_attempted and current_path is None and start_node is not None and goal_node is not None:
        draw_text(screen, "No path exists between start and goal!", 10, info_y + 80, (255, 100, 100))

def draw_graph():
    screen.fill((22, 24, 30))
    draw_edges(current_path)
    draw_nodes()
    draw_ui_info(path_cost)
    pygame.display.flip()
    
def save_snapshot():
    pygame.image.save(screen, "Traffic_Map_snapshot.png")

# Main loop
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
                # Compute path using A*
                path_attempted = True
                current_path, path_cost = astar(start_node, goal_node)
            elif event.key == pygame.K_r:
                # Reset all selections
                start_node = None
                goal_node = None
                reset_path()
            elif event.key == pygame.K_s:
                save_snapshot()
                
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            modifiers = pygame.key.get_mods()
            clicked_node = find_node_at_position(pos)
            clicked_edge = find_edge_near_position(pos)
            
            # Toggle node blocking (SHIFT + click)
            if clicked_node and (modifiers & pygame.KMOD_SHIFT):
                blocked_nodes ^= {clicked_node}  
                reset_path()
                
            # Set start or goal node
            elif clicked_node:
                if event.button == 1:  # Left click = start
                    start_node = clicked_node
                elif event.button == 3:  # Right click = goal
                    goal_node = clicked_node
                reset_path()
                
            # Modify edge weight or block edge
            elif clicked_edge and not clicked_node:
                edge_key = tuple(sorted(clicked_edge))
                
                if modifiers & pygame.KMOD_CTRL:
                    blocked_edges ^= {edge_key}  # Toggle using XOR

                elif modifiers & pygame.KMOD_ALT:
                    weights[edge_key] = max(1, weights[edge_key] - TRAFFIC_STEP)

                else:
                    weights[edge_key] += TRAFFIC_STEP
                
                reset_path()
    
    draw_graph()

pygame.quit()
sys.exit()
