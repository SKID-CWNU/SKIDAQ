SET(SOURCES screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    fonts/ui_font_Font16.c
    fonts/ui_font_Font25.c
    fonts/ui_font_Font30.c
    fonts/ui_font_Font50.c
    fonts/ui_font_Font70.c
    fonts/ui_font_Font1bol.c
    fonts/ui_font_Font20bol.c)

add_library(ui ${SOURCES})
