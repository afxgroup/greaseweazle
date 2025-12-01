cmake_minimum_required(VERSION 3.14)

# Example: Info
add_executable(gw_info info.cpp)
target_link_libraries(gw_info PRIVATE greaseweazle)

# Example: Read track
add_executable(gw_read_track read_track.cpp)
target_link_libraries(gw_read_track PRIVATE greaseweazle)

# Example: ADF read/write (Amiga disk images)
add_executable(gw_adf_read_write adf_read_write.cpp)
target_link_libraries(gw_adf_read_write PRIVATE greaseweazle)
