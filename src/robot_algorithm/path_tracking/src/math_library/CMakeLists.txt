add_library(math_library_lib math_library.cpp)

install(TARGETS math_library_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)