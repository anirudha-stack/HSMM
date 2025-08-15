#include "menu.h"
#include "menu_router.h"
#include "ssd1306.h"
#include "fonts.h"

#define SCROLL_THRESHOLD 5
static int16_t lastScrollPos = 0;
static int16_t lastSelectPos = 0;
static int8_t scrollDelta = 0;
static int8_t selectDelta = 0;

void Menu_Init(MenuContext* ctx, MenuItem* rootItems, uint8_t rootCount) {
    ctx->currentItems = rootItems;
    ctx->itemCount = rootCount;
    ctx->selectedIndex = 0;
    ctx->parent = NULL;
    ctx->parentCount = 0;
}

void Menu_UpdateEncoderInputs(int16_t absScroll, int16_t absSelect) {
    scrollDelta = absScroll - lastScrollPos;
    selectDelta = absSelect - lastSelectPos;
    lastScrollPos = absScroll;
    lastSelectPos = absSelect;
}

void Menu_ProcessScroll(MenuContext* ctx, int8_t delta) {
    static int8_t scrollTicks = 0;
    scrollTicks += delta;
    if (scrollTicks >= SCROLL_THRESHOLD) {
        ctx->selectedIndex = (ctx->selectedIndex + 1) % ctx->itemCount;
        scrollTicks = 0;
    } else if (scrollTicks <= -SCROLL_THRESHOLD) {
        ctx->selectedIndex = (ctx->selectedIndex + ctx->itemCount - 1) % ctx->itemCount;
        scrollTicks = 0;
    }
}
void Menu_ProcessSelect(MenuContext* ctx, int8_t delta) {
    static int8_t selectTicks = 0;
    selectTicks += delta;

    if (selectTicks >= SCROLL_THRESHOLD) {
        MenuItem* current = &ctx->currentItems[ctx->selectedIndex];

        if (current->subItemCount > 0 && current->subItems != NULL) {
            // ?? This is where submenu is entered
            ctx->parent = ctx->currentItems;
            ctx->parentCount = ctx->itemCount;

            ctx->currentItems = current->subItems;
            ctx->itemCount = current->subItemCount;
            ctx->selectedIndex = 0;
        } else {
            Menu_ExecuteAction(current->actionId);
        }

        selectTicks = 0;
    } else if (selectTicks <= -SCROLL_THRESHOLD) {
        // ?? This is where you should go back to parent
        if (ctx->parent != NULL) {
            ctx->currentItems = ctx->parent;
            ctx->itemCount = ctx->parentCount;
            ctx->selectedIndex = 0;
            ctx->parent = NULL;
            ctx->parentCount = 0;
        }
        selectTicks = 0;
    }
}

void Menu_Render(MenuContext* ctx) {
    if (scrollDelta) {
        Menu_ProcessScroll(ctx, scrollDelta);
        scrollDelta = 0;
    }
    if (selectDelta) {
        Menu_ProcessSelect(ctx, selectDelta);
        selectDelta = 0;
    }

    SSD1306_Clear();

    // Calculate the starting index for visible window
    uint8_t visibleLines = 3;
    uint8_t start = 0;
    if (ctx->selectedIndex >= 1 && ctx->itemCount > visibleLines) {
        if (ctx->selectedIndex == ctx->itemCount - 1)
            start = ctx->itemCount - visibleLines;
        else
            start = ctx->selectedIndex - 1;
    }

    // Render up to 3 visible items
    for (uint8_t i = 0; i < visibleLines; ++i) {
        uint8_t idx = start + i;
        if (idx >= ctx->itemCount) break;

        SSD1306_GotoXY(0, i * 10);
        if (idx == ctx->selectedIndex)
            SSD1306_Puts(">", &Font_7x10, SSD1306_COLOR_WHITE);
        else
            SSD1306_Puts(" ", &Font_7x10, SSD1306_COLOR_WHITE);

        SSD1306_Puts(ctx->currentItems[idx].label, &Font_7x10, SSD1306_COLOR_WHITE);
    }

    SSD1306_UpdateScreen();
}
