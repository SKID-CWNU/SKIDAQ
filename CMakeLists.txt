SET(SOURCES screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    fonts/ui_font_Font16.c
    fonts/ui_font_Font1.c
    fonts/ui_font_Font50.c
    fonts/ui_font_Font30.c)

add_library(ui ${SOURCES})
