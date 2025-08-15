
// maze.c - Maze Solving Implementation
#include "maze.h"
#include <string.h>
#include <stdlib.h>

// Static maze data
static MazeCell_t maze[MAZE_SIZE][MAZE_SIZE];
static MouseState_t mouse_state;
static MazeState_t maze_state;
static MazeConfig_t config;
static OptimalPath_t current_path;
static uint16_t path_index;

// Exploration state
static uint8_t exploration_run_count = 0;
static bool returning_to_start = false;

// Direction vectors for navigation
static const int8_t dx[] = {0, 1, 0, -1};  // N, E, S, W
static const int8_t dy[] = {1, 0, -1, 0};  // N, E, S, W

// Private function prototypes
static void Maze_InitializeBoundaryWalls(void);
static bool Maze_FloodFillStep(Coordinate_t goal);
static Direction_t Maze_ChooseExplorationDirection(void);
static bool Maze_IsDeadEnd(Coordinate_t pos);
static uint8_t Maze_CountUnknownWalls(Coordinate_t pos);
static void Maze_MarkPath(Coordinate_t start, Coordinate_t goal);

void Maze_Init(MazeConfig_t *maze_config) {
    // Copy configuration
    if (maze_config) {
        config = *maze_config;
    } else {
        // Default configuration
        config.exploration_speed_mms = 200;
        config.speed_run_speed_mms = 1000;
        config.turn_speed_mms = 300;
        config.use_diagonal_moves = false;
        config.conservative_exploration = true;
        config.max_exploration_runs = 3;
    }
    
    Maze_Reset();
}

void Maze_Reset(void) {
    // Clear maze data
    memset(maze, 0, sizeof(maze));
    
    // Initialize boundary walls
    Maze_InitializeBoundaryWalls();
    
    // Initialize mouse state
    mouse_state.position.x = 0;
    mouse_state.position.y = 0;
    mouse_state.heading = DIR_NORTH;
    mouse_state.in_center = false;
    mouse_state.total_distance_mm = 0;
    mouse_state.exploration_time_ms = 0;
    
    // Reset exploration state
    maze_state = MAZE_STATE_EXPLORING;
    exploration_run_count = 0;
    returning_to_start = false;
    path_index = 0;
    
    // Initial flood fill from center
    Coordinate_t center = {MAZE_CENTER_X, MAZE_CENTER_Y};
    Maze_FloodFill(center);
}

static void Maze_InitializeBoundaryWalls(void) {
    // Set outer walls
    for (int8_t i = 0; i < MAZE_SIZE; i++) {
        // North boundary
        maze[i][MAZE_SIZE-1].walls |= WALL_NORTH;
        // South boundary  
        maze[i][0].walls |= WALL_SOUTH;
        // East boundary
        maze[MAZE_SIZE-1][i].walls |= WALL_EAST;
        // West boundary
        maze[0][i].walls |= WALL_WEST;
    }
    
    // Set starting position walls (known configuration)
    maze[0][0].walls |= WALL_WEST | WALL_SOUTH;
}

void Maze_SetWall(int8_t x, int8_t y, Direction_t direction, bool exists) {
    if (!Maze_IsValidPosition(x, y)) return;
    
    if (exists) {
        maze[x][y].walls |= (1 << direction);
        
        // Set corresponding wall on adjacent cell
        Coordinate_t adj = Maze_GetAdjacentCell((Coordinate_t){x, y}, direction);
        if (Maze_IsValidPosition(adj.x, adj.y)) {
            Direction_t opposite = Maze_GetOppositeDirection(direction);
            maze[adj.x][adj.y].walls |= (1 << opposite);
        }
    } else {
        maze[x][y].walls &= ~(1 << direction);
        
        // Clear corresponding wall on adjacent cell
        Coordinate_t adj = Maze_GetAdjacentCell((Coordinate_t){x, y}, direction);
        if (Maze_IsValidPosition(adj.x, adj.y)) {
            Direction_t opposite = Maze_GetOppositeDirection(direction);
            maze[adj.x][adj.y].walls &= ~(1 << opposite);
        }
    }
}

bool Maze_GetWall(int8_t x, int8_t y, Direction_t direction) {
    if (!Maze_IsValidPosition(x, y)) return true;
    return (maze[x][y].walls & (1 << direction)) != 0;
}

void Maze_UpdatePosition(int8_t x, int8_t y, Direction_t heading) {
    mouse_state.position.x = x;
    mouse_state.position.y = y;
    mouse_state.heading = heading;
    mouse_state.in_center = Maze_IsCenter(x, y);
    
    // Mark cell as visited
    if (Maze_IsValidPosition(x, y)) {
        maze[x][y].visited = true;
    }
}

MoveCommand_t Maze_GetNextExplorationMove(void) {
    Coordinate_t current = mouse_state.position;
    
    // Check if we've reached the center
    if (Maze_IsCenter(current.x, current.y) && !returning_to_start) {
        returning_to_start = true;
        exploration_run_count++;
        
        // Start return journey - flood fill from start
        Coordinate_t start = {0, 0};
        Maze_FloodFill(start);
    }
    
    // Check if we've returned to start
    if (returning_to_start && current.x == 0 && current.y == 0) {
        returning_to_start = false;
        
        if (exploration_run_count >= config.max_exploration_runs || 
            Maze_IsExplorationComplete()) {
            maze_state = MAZE_STATE_SPEED_RUN;
            return MOVE_STOP;
        } else {
            // Start another exploration run
            Coordinate_t center = {MAZE_CENTER_X, MAZE_CENTER_Y};
            Maze_FloodFill(center);
        }
    }
    
    // Choose next direction based on flood fill
    Direction_t target_dir = Maze_ChooseExplorationDirection();
    
    if (target_dir == mouse_state.heading) {
        return MOVE_FORWARD;
    } else {
        Direction_t relative = Maze_GetRelativeDirection(mouse_state.heading, target_dir);
        switch (relative) {
            case DIR_EAST:  return MOVE_TURN_RIGHT;
            case DIR_WEST:  return MOVE_TURN_LEFT;
            case DIR_SOUTH: return MOVE_TURN_AROUND;
            default:        return MOVE_FORWARD;
        }
    }
}

static Direction_t Maze_ChooseExplorationDirection(void) {
    Coordinate_t current = mouse_state.position;
    uint16_t min_distance = UINT16_MAX;
    Direction_t best_direction = mouse_state.heading;
    
    // Check all four directions
    for (Direction_t dir = DIR_NORTH; dir <= DIR_WEST; dir++) {
        // Skip if wall exists
        if (Maze_GetWall(current.x, current.y, dir)) continue;
        
        Coordinate_t next = Maze_GetAdjacentCell(current, dir);
        if (!Maze_IsValidPosition(next.x, next.y)) continue;
        
        uint16_t distance = maze[next.x][next.y].distance;
        
        // Prefer unexplored cells in conservative mode
        if (config.conservative_exploration) {
            uint8_t unknown_walls = Maze_CountUnknownWalls(next);
            if (unknown_walls > 0) {
                distance -= 100;  // Bias towards exploration
            }
        }
        
        if (distance < min_distance) {
            min_distance = distance;
            best_direction = dir;
        }
    }
    
    return best_direction;
}

void Maze_FloodFill(Coordinate_t goal) {
    // Initialize distances
    for (int8_t x = 0; x < MAZE_SIZE; x++) {
        for (int8_t y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].distance = UINT16_MAX;
        }
    }
    
    // Set goal distance(s)
    if (Maze_IsCenter(goal.x, goal.y)) {
        // Set all center cells to distance 0
        for (int8_t x = MAZE_CENTER_X; x < MAZE_CENTER_X + MAZE_CENTER_SIZE; x++) {
            for (int8_t y = MAZE_CENTER_Y; y < MAZE_CENTER_Y + MAZE_CENTER_SIZE; y++) {
                maze[x][y].distance = 0;
            }
        }
    } else {
        maze[goal.x][goal.y].distance = 0;
    }
    
    // Flood fill algorithm
    bool changed;
    do {
        changed = Maze_FloodFillStep(goal);
    } while (changed);
}

static bool Maze_FloodFillStep(Coordinate_t goal) {
    bool changed = false;
    
    for (int8_t x = 0; x < MAZE_SIZE; x++) {
        for (int8_t y = 0; y < MAZE_SIZE; y++) {
            uint16_t current_distance = maze[x][y].distance;
            if (current_distance == UINT16_MAX) continue;
            
            // Check all neighbors
            for (Direction_t dir = DIR_NORTH; dir <= DIR_WEST; dir++) {
                if (Maze_GetWall(x, y, dir)) continue;
                
                Coordinate_t neighbor = Maze_GetAdjacentCell((Coordinate_t){x, y}, dir);
                if (!Maze_IsValidPosition(neighbor.x, neighbor.y)) continue;
                
                uint16_t new_distance = current_distance + 1;
                if (new_distance < maze[neighbor.x][neighbor.y].distance) {
                    maze[neighbor.x][neighbor.y].distance = new_distance;
                    changed = true;
                }
            }
        }
    }
    
    return changed;
}

bool Maze_FindOptimalPath(Coordinate_t start, Coordinate_t goal, OptimalPath_t *path) {
    if (!path) return false;
    
    // Flood fill from goal
    Maze_FloodFill(goal);
    
    // Clear previous path
    memset(path, 0, sizeof(OptimalPath_t));
    
    // Trace path from start to goal
    Coordinate_t current = start;
    path->node_count = 0;
    
    while (!Maze_IsCenter(current.x, current.y) && 
           path->node_count < MAZE_SIZE * MAZE_SIZE) {
        
        // Find best next move
        uint16_t min_distance = UINT16_MAX;
        Direction_t best_dir = DIR_NORTH;
        
        for (Direction_t dir = DIR_NORTH; dir <= DIR_WEST; dir++) {
            if (Maze_GetWall(current.x, current.y, dir)) continue;
            
            Coordinate_t next = Maze_GetAdjacentCell(current, dir);
            if (!Maze_IsValidPosition(next.x, next.y)) continue;
            
            if (maze[next.x][next.y].distance < min_distance) {
                min_distance = maze[next.x][next.y].distance;
                best_dir = dir;
            }
        }
        
        // Add node to path
        PathNode_t *node = &path->nodes[path->node_count];
        node->pos = current;
        node->direction = best_dir;
        node->distance_mm = MAZE_CELL_SIZE_MM;
        node->speed = config.speed_run_speed_mms;
        
        path->node_count++;
        path->total_distance_mm += MAZE_CELL_SIZE_MM;
        
        // Move to next cell
        current = Maze_GetAdjacentCell(current, best_dir);
        
        // Mark path for visualization
        maze[current.x][current.y].on_path = true;
    }
    
    return path->node_count > 0;
}

bool Maze_IsExplorationComplete(void) {
    uint16_t explored_cells = 0;
    
    for (int8_t x = 0; x < MAZE_SIZE; x++) {
        for (int8_t y = 0; y < MAZE_SIZE; y++) {
            if (maze[x][y].visited) {
                explored_cells++;
            }
        }
    }
    
    // Consider exploration complete if we've visited enough cells
    return explored_cells >= (MAZE_SIZE * MAZE_SIZE * 0.7f);
}

// Utility functions
bool Maze_IsValidPosition(int8_t x, int8_t y) {
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

bool Maze_IsCenter(int8_t x, int8_t y) {
    return (x >= MAZE_CENTER_X && x < MAZE_CENTER_X + MAZE_CENTER_SIZE &&
            y >= MAZE_CENTER_Y && y < MAZE_CENTER_Y + MAZE_CENTER_SIZE);
}

Direction_t Maze_GetOppositeDirection(Direction_t dir) {
    return (Direction_t)((dir + 2) % 4);
}

Direction_t Maze_GetRelativeDirection(Direction_t current, Direction_t target) {
    return (Direction_t)((target - current + 4) % 4);
}

Coordinate_t Maze_GetAdjacentCell(Coordinate_t pos, Direction_t dir) {
    Coordinate_t result = {
        .x = pos.x + dx[dir],
        .y = pos.y + dy[dir]
    };
    return result;
}

static uint8_t Maze_CountUnknownWalls(Coordinate_t pos) {
    uint8_t count = 0;
    
    for (Direction_t dir = DIR_NORTH; dir <= DIR_WEST; dir++) {
        Coordinate_t adj = Maze_GetAdjacentCell(pos, dir);
        if (Maze_IsValidPosition(adj.x, adj.y) && !maze[adj.x][adj.y].visited) {
            count++;
        }
    }
    
    return count;
}

MouseState_t Maze_GetMouseState(void) {
    return mouse_state;
}

MazeState_t Maze_GetState(void) {
    return maze_state;
}

void Maze_SetState(MazeState_t state) {
    maze_state = state;
}

uint16_t Maze_GetExplorationPercentage(void) {
    uint16_t visited_count = 0;
    
    for (int8_t x = 0; x < MAZE_SIZE; x++) {
        for (int8_t y = 0; y < MAZE_SIZE; y++) {
            if (maze[x][y].visited) {
                visited_count++;
            }
        }
    }
    
    return (visited_count * 100) / (MAZE_SIZE * MAZE_SIZE);
}