# Define the local library
add_library(analog5b
    AnalogFiveButtons.cpp
)

# Where to find the includes
include_directories( ${libspath}/Wire )
#include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../libraries/ST7565 )
include_directories( ${ST7565_DIR} )

add_program( UserInterface analog5b st7565 arduinolibs corearduino m )

#add_subdirectory ( tests )

