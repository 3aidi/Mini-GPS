# Mini GPS Traffic Map using A* Algorithm

## Authors

- **Mahmoud Saad Elaidi** (ID: 224101559)
- **Ismail Ahmed Ismail** (ID: 224101657)
- **Ahmed Hatem Ali** (ID: 224101514)

---

## 1. Project Title

**Mini GPS Traffic Map using A* Algorithm**

---

## 2. Project Description

This project is a beginner-friendly graphical simulation of a GPS system that finds the shortest path between two locations using the **A* pathfinding algorithm**.

The program visualizes a city map as a graph where:

- **Nodes** represent locations (e.g., Bank, School, Hospital)
- **Edges** represent roads with adjustable traffic weights
- The user can interactively modify traffic and block locations to see how routes change

The goal is to demonstrate graph traversal, heuristics, and pathfinding logic in a clear and visual way.

---

## 3. Objectives

- Apply graph concepts (nodes, edges, weights)
- Implement the A* search algorithm
- Visualize pathfinding results using Pygame
- Handle user interaction and real-time updates
- Perform logical and arithmetic validation of paths

---

## 4. Technologies Used

- **Python 3**
- **Pygame library**
- **Heap-based priority queue** (heapq)
- **Euclidean distance calculations**

---

## 5. Features

- Visual map with labeled locations
- Left-click to select start node
- Right-click to select goal node
- Block/unblock nodes using SHIFT + click
- Increase/decrease traffic on roads
- A* pathfinding with heuristic
- Real-time cost calculation
- Screenshot saving

---

## 6. Data Structures Used

- **Dictionary** for node coordinates
- **Adjacency list** for graph connections
- **Dictionary** for edge weights
- **Set** for blocked nodes and edges
- **Priority queue** for A* open list

---

## 7. Program Flow

1. Program initializes the map and graph
2. User selects a start and goal node
3. User optionally blocks nodes or modifies traffic
4. A* algorithm calculates the shortest path
5. Path cost is displayed
6. User can reset or modify the map and recalculate

---

## 8. Input and Output

### Input:
- Mouse clicks (nodes and edges)
- Keyboard keys (SPACE, R, S, ESC)

### Output:
- Visual highlighted path
- Total path cost
- Traffic color indicators
- Error message if no path exists

---

## 9. Example Scenario

**Start:** Bank  
**Goal:** Cinema

**Calculated path:**
```
Bank → Cafe → Club → Market → Hospital → Cinema
```

**Total cost:** 1340

---

## 10. Arithmetic & Logical Testing

### Normal Case
- All nodes open
- Traffic weights unchanged  
→ Shortest Euclidean-based path is found

### Edge Case
- Start or goal node blocked  
→ No path returned

### Traffic Modification
- Increase traffic on a main road  
→ Algorithm selects a longer but cheaper alternative route

### Invalid Case
- All connecting paths blocked  
→ Program correctly reports no available path

---

## 11. Error Handling

- Prevents pathfinding if start or goal is blocked
- Handles unreachable destinations safely
- Avoids negative or zero edge weights
- Skips invalid clicks

---

## 12. Limitations

- No file-based data persistence
- Static predefined map
- No animation for path traversal
- Designed for educational use only

---

## 13. Controls Summary

| Control | Action |
|---------|--------|
| **Left Click** | Set start |
| **Right Click** | Set goal |
| **SHIFT + Click** | Block/unblock node |
| **Click edge** | Increase traffic |
| **ALT + Click edge** | Decrease traffic |
| **SPACE** | Compute path |
| **R** | Reset |
| **S** | Save screenshot |
| **ESC** | Exit |

---

## 14. Conclusion

This project demonstrates how **A* pathfinding** works in a real-world-like GPS scenario.

It combines:
- Graph theory
- Heuristics
- Interactive visualization

The system clearly shows how traffic and obstacles affect route selection, making it an excellent educational tool for understanding pathfinding algorithms.
