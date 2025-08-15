#ifndef MENU_H
#define MENU_H

#include <stdint.h>

typedef enum {
    ACTION_NONE = 0,
    ACTION_SEARCH_MAZE,
   	ACTION_RUN_MAZE,
    ACTION_CALIB_SENSOR,
    ACTION_SET_PID,
    ACTION_UTIL_TEST,
} MenuActionID;

typedef struct MenuItem {
    char* label;
    MenuActionID actionId;
    struct MenuItem* subItems;
    uint8_t subItemCount;
} MenuItem;

typedef struct {
    MenuItem* currentItems;
    uint8_t itemCount;
    uint8_t selectedIndex;
    MenuItem* parent;
    uint8_t parentCount;
} MenuContext;

void Menu_Init(MenuContext* ctx, MenuItem* rootItems, uint8_t rootCount);
void Menu_Render(MenuContext* ctx);
void Menu_UpdateEncoderInputs(int16_t absScroll, int16_t absSelect);

#endif