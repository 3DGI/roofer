include_directories(../src)

add_executable(crop crop.cpp)
set_target_properties(
  crop PROPERTIES 
  CXX_STANDARD 17
)
target_link_libraries(crop PRIVATE rooferlib spdlog::spdlog)


# add polyscope
add_subdirectory(external/polyscope)

add_executable(ransac_demo ransac_demo.cpp)
set_target_properties(
  ransac_demo PROPERTIES 
  CXX_STANDARD 17
)
target_link_libraries(ransac_demo PRIVATE rooferlib polyscope)