add_executable(test_launch launch_mujoco.cc)
target_link_libraries(test_launch
        PUBLIC
        mujoco
        ${mujoco_LIBRARIES}
        ${mujoco_GL_LIBRARIES}
        ${mujoco_GLUT_LIBRARIES})