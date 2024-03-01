add_executable(pick_place_demo src/pick_place_demo.cpp)
target_link_libraries(pick_place_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_place_demo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
