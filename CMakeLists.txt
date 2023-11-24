SET(SOURCES screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
<<<<<<< HEAD
    fonts/ui_font_Font1.c
    fonts/ui_font_Font16.c
    fonts/ui_font_Font30.c
    fonts/ui_font_Font50.c)
=======
    fonts/ui_font_Font16.c
    fonts/ui_font_Font1.c
    fonts/ui_font_Font50.c
    fonts/ui_font_Font30.c)
>>>>>>> d953d3e0f59e41b633e8c1c313e449d102ff31e8

add_library(ui ${SOURCES})
