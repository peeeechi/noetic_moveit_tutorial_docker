add_executable(
    move_group_interface_tutorial src/move_group_interface_tutorial.cpp
)
add_executable(
    pick_and_place src/pick_and_place.cpp
)
add_executable(
    fk_test src/fk_test.cpp
)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(fk_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS pick_and_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS fk_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})