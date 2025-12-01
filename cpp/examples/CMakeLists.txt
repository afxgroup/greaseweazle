cmake_minimum_required(VERSION 3.14)

# Example: Info
add_executable(gw_info info.cpp)
target_link_libraries(gw_info PRIVATE greaseweazle)

# Example: Read track
add_executable(gw_read_track read_track.cpp)
target_link_libraries(gw_read_track PRIVATE greaseweazle)
