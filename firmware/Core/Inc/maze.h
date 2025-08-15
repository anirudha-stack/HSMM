// maze.h - Maze Solving and Path Planning System
#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>
#include <stdbool.h>

// Maze constants
#define MAZE_SIZE           16
#define MAZE_CENTER_SIZE    2
#define MAZE_CENTER_X       (MAZE_SIZE/2)
#define MAZE_CENTER_Y       (MAZE_SIZE/2)
#define MAZE_CELL_SIZE_MM   180    // Standard micromouse cell size

// Wall definitions
#define WALL_NORTH  0x01
#define WALL_EAST   0x02
#define WALL_SOUTH  0x04
#define WALL_WEST   0x08
#define WALL_UNKNOWN 0x00
#define WALL_ALL    0x0F

// Directions
typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
} Direction_t;

// Coordinates
typedef struct {
    int8_t x;
    int8_t y;
} Coordinate_t;

// Maze cell
typedef struct {
    uint8_t walls;          // Bit field for walls
    uint16_t distance;      // Distance from goal (flood fill)
    bool visited;           // Exploration flag
    bool on_path;           // Part of optimal path
} MazeCell_t;

// Path node for path planning
typedef struct {
    Coordinate_t pos;
    Direction_t direction;
    uint8_t speed;          // Speed for this segment
    uint16_t distance_mm;   // Distance to travel
} PathNode_t;

// Path planning result
typedef struct {
    PathNode_t nodes[MAZE_SIZE * MAZE_SIZE];
    uint16_t node_count;
    uint32_t total_distance_mm;
    uint32_t estimated_time_ms;
} OptimalPath_t;

// Mouse state
typedef struct {
    Coordinate_t position;
    Direction_t heading;
    bool in_center;
    uint32_t total_distance_mm;
    uint32_t exploration_time_ms;
} MouseState_t;

// Maze exploration state
typedef enum {
    MAZE_STATE_EXPLORING = 0,
    MAZE_STATE_RETURNING,
    MAZE_STATE_SPEED_RUN,
    MAZE_STATE_COMPLETE,
    MAZE_STATE_ERROR
} MazeState_t;

// Movement commands
typedef enum {
    MOVE_FORWARD = 0,
    MOVE_TURN_LEFT,
    MOVE_TURN_RIGHT,
    MOVE_TURN_AROUND,
    MOVE_STOP
} MoveCommand_t;

// Algorithm configuration
typedef struct {
    uint16_t exploration_speed_mms;
    uint16_t speed_run_speed_mms;
    uint16_t turn_speed_mms;
    bool use_diagonal_moves;
    bool conservative_exploration;
    uint8_t max_exploration_runs;
} MazeConfig_t;

// Function prototypes

// Maze initialization and management
void Maze_Init(MazeConfig_t *config);
void Maze_Reset(void);
void Maze_SetWall(int8_t x, int8_t y, Direction_t direction, bool exists);
bool Maze_GetWall(int8_t x, int8_t y, Direction_t direction);
void Maze_UpdatePosition(int8_t x, int8_t y, Direction_t heading);

// Exploration algorithms
MoveCommand_t Maze_GetNextExplorationMove(void);
bool Maze_IsExplorationComplete(void);
void Maze_FloodFill(Coordinate_t goal);
uint16_t Maze_GetCellDistance(int8_t x, int8_t y);

// Path planning and optimization
bool Maze_FindOptimalPath(Coordinate_t start, Coordinate_t goal, OptimalPath_t *path);
bool Maze_PlanSpeedProfile(OptimalPath_t *path);
MoveCommand_t Maze_GetNextSpeedRunMove(void);

// Navigation utilities
bool Maze_IsValidPosition(int8_t x, int8_t y);
bool Maze_IsCenter(int8_t x, int8_t y);
Direction_t Maze_GetOppositeDirection(Direction_t dir);
Direction_t Maze_GetRelativeDirection(Direction_t current, Direction_t target);
Coordinate_t Maze_GetAdjacentCell(Coordinate_t pos, Direction_t dir);

// State management
MouseState_t Maze_GetMouseState(void);
MazeState_t Maze_GetState(void);
void Maze_SetState(MazeState_t state);

// Statistics and analysis
uint16_t Maze_GetExplorationPercentage(void);
uint32_t Maze_GetPathDistance(Coordinate_t start, Coordinate_t goal);
void Maze_PrintMaze(void);  // For debugging

#endif // MAZE_H
