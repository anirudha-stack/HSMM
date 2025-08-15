// menu.h - Enhanced Menu System
#ifndef MENU_H
#define MENU_H

#include <stdint.h>
#include <stdbool.h>

// Menu constants
#define MENU_MAX_ITEMS          10
#define MENU_MAX_LABEL_LENGTH   20
#define MENU_MAX_DEPTH          5
#define MENU_SCROLL_THRESHOLD   4
#define MENU_SELECT_THRESHOLD   4
#define MENU_IDLE_TIMEOUT_MS    30000

// Menu action types
typedef enum {
    ACTION_NONE = 0,
    ACTION_SEARCH_MAZE,
    ACTION_RUN_MAZE,
    ACTION_CALIB_SENSOR,
    ACTION_SET_PID,
    ACTION_UTIL_TEST,
    ACTION_VIEW_STATS,
    ACTION_FACTORY_RESET,
    ACTION_SAVE_CONFIG,
    ACTION_LOAD_CONFIG
} MenuActionID_t;

// Menu item types
typedef enum {
    MENU_TYPE_ACTION = 0,
    MENU_TYPE_SUBMENU,
    MENU_TYPE_VALUE_EDIT,
    MENU_TYPE_TOGGLE,
    MENU_TYPE_BACK
} MenuItemType_t;

// Menu states
typedef enum {
    MENU_STATE_IDLE = 0,
    MENU_STATE_BROWSING,
    MENU_STATE_EXECUTING,
    MENU_STATE_EDITING,
    MENU_STATE_CONFIRM
} MenuState_t;

// Forward declaration
struct MenuItem;

// Value editing structure
typedef struct {
    float *value_ptr;
    float min_value;
    float max_value;
    float step_size;
    uint8_t decimal_places;
    char unit[8];
} ValueEdit_t;

// Menu item structure
typedef struct MenuItem {
    char label[MENU_MAX_LABEL_LENGTH];
    MenuItemType_t type;
    MenuActionID_t action_id;
    
    union {
        struct {
            struct MenuItem* items;
            uint8_t count;
        } submenu;
        
        ValueEdit_t value_edit;
        
        bool *toggle_ptr;
    } data;
    
    bool visible;
    bool enabled;
} MenuItem_t;

// Menu context with navigation history
typedef struct {
    MenuItem_t* current_menu;
    uint8_t item_count;
    uint8_t selected_index;
    uint8_t scroll_offset;
    
    // Navigation stack for back functionality
    struct {
        MenuItem_t* menu;
        uint8_t selected_index;
        uint8_t scroll_offset;
    } nav_stack[MENU_MAX_DEPTH];
    uint8_t nav_depth;
    
    // Input handling
    int16_t last_scroll_pos;
    int16_t last_select_pos;
    int8_t scroll_accumulator;
    int8_t select_accumulator;
    
    // State management
    MenuState_t state;
    uint32_t last_activity_ms;
    uint32_t action_start_ms;
    
    // Editing state
    float edit_value;
    bool edit_confirmed;
    
    // Display properties
    uint8_t visible_lines;
    bool needs_refresh;
} MenuContext_t;

// Menu configuration
typedef struct {
    uint8_t visible_lines;
    uint32_t idle_timeout_ms;
    uint32_t action_timeout_ms;
    bool auto_return_enabled;
    bool confirm_actions;
} MenuConfig_t;

// Function prototypes
bool Menu_Init(MenuContext_t *ctx, MenuItem_t *root_menu, uint8_t item_count, MenuConfig_t *config);
void Menu_Update(MenuContext_t *ctx, int16_t scroll_pos, int16_t select_pos);
void Menu_Render(MenuContext_t *ctx);
bool Menu_ExecuteAction(MenuContext_t *ctx, MenuActionID_t action_id);

// Navigation functions
bool Menu_EnterSubmenu(MenuContext_t *ctx);
bool Menu_GoBack(MenuContext_t *ctx);
void Menu_GoToRoot(MenuContext_t *ctx);
bool Menu_SelectItem(MenuContext_t *ctx, uint8_t index);

// Value editing functions
bool Menu_StartValueEdit(MenuContext_t *ctx);
void Menu_UpdateValueEdit(MenuContext_t *ctx, int8_t direction);
bool Menu_ConfirmValueEdit(MenuContext_t *ctx);
void Menu_CancelValueEdit(MenuContext_t *ctx);

// State management
MenuState_t Menu_GetState(MenuContext_t *ctx);
bool Menu_IsIdle(MenuContext_t *ctx);
void Menu_ResetIdleTimer(MenuContext_t *ctx);
void Menu_ForceRefresh(MenuContext_t *ctx);

// Utility functions
void Menu_SetItemVisibility(MenuItem_t *item, bool visible);
void Menu_SetItemEnabled(MenuItem_t *item, bool enabled);
uint8_t Menu_GetVisibleItemCount(MenuItem_t *menu, uint8_t total_count);

#endif // MENU_H
