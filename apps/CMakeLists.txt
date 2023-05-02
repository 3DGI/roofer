include_directories(../src)

add_executable(crop crop.cpp)
target_link_libraries(crop PRIVATE rooferlib spdlog::spdlog)


# add polyscope
add_subdirectory(external/polyscope)

add_executable(ransac_demo ransac_demo.cpp)
target_link_libraries(ransac_demo PRIVATE rooferlib polyscope)