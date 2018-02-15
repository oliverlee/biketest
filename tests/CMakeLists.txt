include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_state_space test_state_space.cc ${BICYCLE_SOURCE})
target_link_libraries(test_state_space gtest_main)
add_test(NAME test_state_space COMMAND test_state_space)

add_executable(test_kalman test_kalman.cc test_convergence.cc ${BICYCLE_SOURCE})
target_link_libraries(test_kalman gtest_main)
add_test(NAME test_kalman COMMAND test_kalman)

add_executable(test_lqr test_lqr.cc test_convergence.cc ${BICYCLE_SOURCE})
target_link_libraries(test_lqr gtest_main)
add_test(NAME test_lqr COMMAND test_lqr)

add_executable(test_lqr_kalman test_lqr_kalman.cc test_convergence.cc ${BICYCLE_SOURCE})
target_link_libraries(test_lqr_kalman gtest_main)
add_test(NAME test_lqr_kalman COMMAND test_lqr_kalman)
