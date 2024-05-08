add_library(pure_pursuit_lib pure_pursuit.cpp)

install(TARGETS pure_pursuit_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)