
// menu.c - Enhanced Menu Implementation
#include "menu.h"
#include "ssd1306.h"
#include "fonts.h"
#include <string.h>
#include <stdio.h>

// Static configuration
static MenuConfig_t menu_config = {
    .visible_lines = 3,
    .idle_timeout_ms = 30000,
    .action_timeout_ms = 5000,
    .auto_return_enabled = true,
    .confirm_actions = false
};

// External time function (should be provided by system)
extern uint32_t HAL_GetTick(void);

// Private function prototypes
static void Menu_ProcessScrollInput(MenuContext_t *ctx, int16_t scroll_delta);
static void Menu_ProcessSelectInput(MenuContext_t *ctx, int16_t select_delta);
static void Menu_UpdateScrollOffset(MenuContext_t *ctx);
static void Menu_RenderBrowsingMode(MenuContext_t *ctx);
static void Menu_RenderEditingMode(MenuContext_t *ctx);
static void Menu_RenderConfirmMode(MenuContext_t *ctx);
static bool Menu_PushNavStack(MenuContext_t *ctx);
static bool Menu_PopNavStack(MenuContext_t *ctx);

bool Menu_Init(MenuContext_t *ctx, MenuItem_t *root_menu, uint8_t item_count, MenuConfig_t *config) {
    if (!ctx || !root_menu) return false;
    
    // Clear context
    memset(ctx, 0, sizeof(MenuContext_t));
    
    // Set configuration
    if (config) {
        menu_config = *config;
    }
    
    // Initialize context
    ctx->current_menu = root_menu;
    ctx->item_count = item_count;
    ctx->selected_index = 0;
    ctx->scroll_offset = 0;
    ctx->nav_depth = 0;
    ctx->state = MENU_STATE_BROWSING;
    ctx->visible_lines = menu_config.visible_lines;
    ctx->needs_refresh = true;
    ctx->last_activity_ms = HAL_GetTick();
    
    return true;
}

void Menu_Update(MenuContext_t *ctx, int16_t scroll_pos, int16_t select_pos) {
    if (!ctx) return;
    
    uint32_t current_time = HAL_GetTick();
    
    // Calculate deltas
    int16_t scroll_delta = scroll_pos - ctx->last_scroll_pos;
    int16_t select_delta = select_pos - ctx->last_select_pos;
    
    ctx->last_scroll_pos = scroll_pos;
    ctx->last_select_pos = select_pos;
    
    // Check for input activity
    if (scroll_delta != 0 || select_delta != 0) {
        ctx->last_activity_ms = current_time;
        Menu_ResetIdleTimer(ctx);
    }
    
    // Handle idle timeout
    if (menu_config.auto_return_enabled && 
        (current_time - ctx->last_activity_ms) > menu_config.idle_timeout_ms) {
        if (ctx->state != MENU_STATE_IDLE) {
            Menu_GoToRoot(ctx);
            ctx->state = MENU_STATE_IDLE;
            ctx->needs_refresh = true;
        }
        return;
    }
    
    // Process inputs based on current state
    switch (ctx->state) {
        case MENU_STATE_BROWSING:
            Menu_ProcessScrollInput(ctx, scroll_delta);
            Menu_ProcessSelectInput(ctx, select_delta);
            break;
            
        case MENU_STATE_EDITING:
            Menu_UpdateValueEdit(ctx, scroll_delta);
            if (select_delta > MENU_SELECT_THRESHOLD) {
                Menu_ConfirmValueEdit(ctx);
            } else if (select_delta < -MENU_SELECT_THRESHOLD) {
                Menu_CancelValueEdit(ctx);
            }
            break;
            
        case MENU_STATE_CONFIRM:
            if (select_delta > MENU_SELECT_THRESHOLD) {
                // Confirm action
                MenuItem_t *item = &ctx->current_menu[ctx->selected_index];
                Menu_ExecuteAction(ctx, item->action_id);
                ctx->state = MENU_STATE_BROWSING;
                ctx->needs_refresh = true;
            } else if (select_delta < -MENU_SELECT_THRESHOLD) {
                // Cancel action
                ctx->state = MENU_STATE_BROWSING;
                ctx->needs_refresh = true;
            }
            break;
            
        case MENU_STATE_EXECUTING:
            // Check for action timeout
            if ((current_time - ctx->action_start_ms) > menu_config.action_timeout_ms) {
                ctx->state = MENU_STATE_BROWSING;
                ctx->needs_refresh = true;
            }
            break;
            
        default:
            break;
    }
}

static void Menu_ProcessScrollInput(MenuContext_t *ctx, int16_t scroll_delta) {
    ctx->scroll_accumulator += scroll_delta;
    
    if (ctx->scroll_accumulator >= MENU_SCROLL_THRESHOLD) {
        // Scroll down
        uint8_t visible_items = Menu_GetVisibleItemCount(ctx->current_menu, ctx->item_count);
        if (ctx->selected_index < visible_items - 1) {
            ctx->selected_index++;
            Menu_UpdateScrollOffset(ctx);
            ctx->needs_refresh = true;
        }
        ctx->scroll_accumulator = 0;
    } else if (ctx->scroll_accumulator <= -MENU_SCROLL_THRESHOLD) {
        // Scroll up
        if (ctx->selected_index > 0) {
            ctx->selected_index--;
            Menu_UpdateScrollOffset(ctx);
            ctx->needs_refresh = true;
        }
        ctx->scroll_accumulator = 0;
    }
}

static void Menu_ProcessSelectInput(MenuContext_t *ctx, int16_t select_delta) {
    ctx->select_accumulator += select_delta;
    
    if (ctx->select_accumulator >= MENU_SELECT_THRESHOLD) {
        // Select/Enter
        MenuItem_t *item = &ctx->current_menu[ctx->selected_index];
        
        if (!item->enabled) {
            ctx->select_accumulator = 0;
            return;
        }
        
        switch (item->type) {
            case MENU_TYPE_SUBMENU:
                Menu_EnterSubmenu(ctx);
                break;
                
            case MENU_TYPE_ACTION:
                if (menu_config.confirm_actions) {
                    ctx->state = MENU_STATE_CONFIRM;
                } else {
                    Menu_ExecuteAction(ctx, item->action_id);
                }
                ctx->needs_refresh = true;
                break;
                
            case MENU_TYPE_VALUE_EDIT:
                Menu_StartValueEdit(ctx);
                break;
                
            case MENU_TYPE_TOGGLE:
                if (item->data.toggle_ptr) {
                    *item->data.toggle_ptr = !(*item->data.toggle_ptr);
                    ctx->needs_refresh = true;
                }
                break;
                
            case MENU_TYPE_BACK:
                Menu_GoBack(ctx);
                break;
        }
        
        ctx->select_accumulator = 0;
    } else if (ctx->select_accumulator <= -MENU_SELECT_THRESHOLD) {
        // Back/Cancel
        Menu_GoBack(ctx);
        ctx->select_accumulator = 0;
    }
}

static void Menu_UpdateScrollOffset(MenuContext_t *ctx) {
    // Keep selected item visible
    if (ctx->selected_index < ctx->scroll_offset) {
        ctx->scroll_offset = ctx->selected_index;
    } else if (ctx->selected_index >= ctx->scroll_offset + ctx->visible_lines) {
        ctx->scroll_offset = ctx->selected_index - ctx->visible_lines + 1;
    }
}

bool Menu_EnterSubmenu(MenuContext_t *ctx) {
    MenuItem_t *item = &ctx->current_menu[ctx->selected_index];
    
    if (item->type != MENU_TYPE_SUBMENU) return false;
    
    // Push current state to navigation stack
    if (!Menu_PushNavStack(ctx)) return false;
    
    // Enter submenu
    ctx->current_menu = item->data.submenu.items;
    ctx->item_count = item->data.submenu.count;
    ctx->selected_index = 0;
    ctx->scroll_offset = 0;
    ctx->needs_refresh = true;
    
    return true;
}

bool Menu_GoBack(MenuContext_t *ctx) {
    if (ctx->nav_depth == 0) return false;
    
    return Menu_PopNavStack(ctx);
}

void Menu_GoToRoot(MenuContext_t *ctx) {
    while (ctx->nav_depth > 0) {
        Menu_PopNavStack(ctx);
    }
}

static bool Menu_PushNavStack(MenuContext_t *ctx) {
    if (ctx->nav_depth >= MENU_MAX_DEPTH) return false;
    
    ctx->nav_stack[ctx->nav_depth].menu = ctx->current_menu;
    ctx->nav_stack[ctx->nav_depth].selected_index = ctx->selected_index;
    ctx->nav_stack[ctx->nav_depth].scroll_offset = ctx->scroll_offset;
    ctx->nav_depth++;
    
    return true;
}

static bool Menu_PopNavStack(MenuContext_t *ctx) {
    if (ctx->nav_depth == 0) return false;
    
    ctx->nav_depth--;
    ctx->current_menu = ctx->nav_stack[ctx->nav_depth].menu;
    ctx->selected_index = ctx->nav_stack[ctx->nav_depth].selected_index;
    ctx->scroll_offset = ctx->nav_stack[ctx->nav_depth].scroll_offset;
    ctx->needs_refresh = true;
    
    return true;
}

void Menu_Render(MenuContext_t *ctx) {
    if (!ctx || !ctx->needs_refresh) return;
    
    switch (ctx->state) {
        case MENU_STATE_BROWSING:
            Menu_RenderBrowsingMode(ctx);
            break;
            
        case MENU_STATE_EDITING:
            Menu_RenderEditingMode(ctx);
            break;
            
        case MENU_STATE_CONFIRM:
            Menu_RenderConfirmMode(ctx);
            break;
            
        case MENU_STATE_EXECUTING:
            SSD1306_Clear();
            SSD1306_GotoXY(0, 10);
            SSD1306_Puts("Executing...", &Font_7x10, SSD1306_COLOR_WHITE);
            SSD1306_UpdateScreen();
            break;
            
        case MENU_STATE_IDLE:
            SSD1306_Clear();
            SSD1306_GotoXY(20, 10);
            SSD1306_Puts("Ready", &Font_11x18, SSD1306_COLOR_WHITE);
            SSD1306_UpdateScreen();
            break;
    }
    
    ctx->needs_refresh = false;
}

static void Menu_RenderBrowsingMode(MenuContext_t *ctx) {
    SSD1306_Clear();
    
    uint8_t visible_items = Menu_GetVisibleItemCount(ctx->current_menu, ctx->item_count);
    
    for (uint8_t i = 0; i < ctx->visible_lines && (ctx->scroll_offset + i) < visible_items; i++) {
        uint8_t item_index = ctx->scroll_offset + i;
        MenuItem_t *item = &ctx->current_menu[item_index];
        
        if (!item->visible) continue;
        
        uint8_t y_pos = i * 10;
        
        // Draw selection indicator
        SSD1306_GotoXY(0, y_pos);
        if (item_index == ctx->selected_index) {
            SSD1306_Puts(">", &Font_7x10, SSD1306_COLOR_WHITE);
        } else {
            SSD1306_Puts(" ", &Font_7x10, SSD1306_COLOR_WHITE);
        }
        
        // Draw item label
        SSD1306_GotoXY(10, y_pos);
        SSD1306_Puts(item->label, &Font_7x10, 
                     item->enabled ? SSD1306_COLOR_WHITE : SSD1306_COLOR_BLACK);
        
        // Draw additional indicators
        if (item->type == MENU_TYPE_SUBMENU) {
            SSD1306_GotoXY(120, y_pos);
            SSD1306_Puts(">", &Font_7x10, SSD1306_COLOR_WHITE);
        } else if (item->type == MENU_TYPE_TOGGLE && item->data.toggle_ptr) {
            SSD1306_GotoXY(100, y_pos);
            SSD1306_Puts(*item->data.toggle_ptr ? "ON" : "OFF", &Font_7x10, SSD1306_COLOR_WHITE);
        }
    }
    
    SSD1306_UpdateScreen();
}

static void Menu_RenderEditingMode(MenuContext_t *ctx) {
    SSD1306_Clear();
    
    MenuItem_t *item = &ctx->current_menu[ctx->selected_index];
    ValueEdit_t *edit = &item->data.value_edit;
    
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Edit:", &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY(0, 10);
    SSD1306_Puts(item->label, &Font_7x10, SSD1306_COLOR_WHITE);
    
    // Display current value
    char value_str[20];
    snprintf(value_str, sizeof(value_str), "%.*f %s", 
             edit->decimal_places, ctx->edit_value, edit->unit);
    
    SSD1306_GotoXY(0, 20);
    SSD1306_Puts(value_str, &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_UpdateScreen();
}

static void Menu_RenderConfirmMode(MenuContext_t *ctx) {
    SSD1306_Clear();
    
    MenuItem_t *item = &ctx->current_menu[ctx->selected_index];
    
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Confirm:", &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY(0, 10);
    SSD1306_Puts(item->label, &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY(0, 20);
    SSD1306_Puts("Press to confirm", &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_UpdateScreen();
}

uint8_t Menu_GetVisibleItemCount(MenuItem_t *menu, uint8_t total_count) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < total_count; i++) {
        if (menu[i].visible) count++;
    }
    return count;
}

bool Menu_StartValueEdit(MenuContext_t *ctx) {
    MenuItem_t *item = &ctx->current_menu[ctx->selected_index];
    if (item->type != MENU_TYPE_VALUE_EDIT) return false;
    
    ctx->edit_value = *item->data.value_edit.value_ptr;
    ctx->state = MENU_STATE_EDITING;
    ctx->needs_refresh = true;
    
    return true;
}

void Menu_UpdateValueEdit(MenuContext_t *ctx, int8_t direction) {
    MenuItem_t *item = &ctx->current_menu[ctx->selected_index];
    ValueEdit_t *edit = &item->data.value_edit;
    
    if (direction > 0) {
        ctx->edit_value += edit->step_size;
        if (ctx->edit_value > edit->max_value) {
            ctx->edit_value = edit->max_value;
        }
    } else if (direction < 0) {
        ctx->edit_value -= edit->step_size;
        if (ctx->edit_value < edit->min_value) {
            ctx->edit_value = edit->min_value;
        }
    }
    
    ctx->needs_refresh = true;
}

bool Menu_ConfirmValueEdit(MenuContext_t *ctx) {
    MenuItem_t *item = &ctx->current_menu[ctx->selected_index];
    if (item->type != MENU_TYPE_VALUE_EDIT) return false;
    
    *item->data.value_edit.value_ptr = ctx->edit_value;
    ctx->state = MENU_STATE_BROWSING;
    ctx->needs_refresh = true;
    
    return true;
}

void Menu_CancelValueEdit(MenuContext_t *ctx) {
    ctx->state = MENU_STATE_BROWSING;
    ctx->needs_refresh = true;
}

MenuState_t Menu_GetState(MenuContext_t *ctx) {
    return ctx->state;
}

void Menu_ResetIdleTimer(MenuContext_t *ctx) {
    ctx->last_activity_ms = HAL_GetTick();
    if (ctx->state == MENU_STATE_IDLE) {
        ctx->state = MENU_STATE_BROWSING;
        ctx->needs_refresh = true;
    }
}

void Menu_ForceRefresh(MenuContext_t *ctx) {
    ctx->needs_refresh = true;
}