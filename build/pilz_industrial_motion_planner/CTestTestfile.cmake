# CMake generated Testfile for 
# Source directory: /home/ubuntu/ros2_ws/src/moveit2/moveit_planners/pilz_industrial_motion_planner
# Build directory: /home/ubuntu/ros2_ws/build/pilz_industrial_motion_planner
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[cppcheck]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/ros2_ws/build/pilz_industrial_motion_planner/test_results/pilz_industrial_motion_planner/cppcheck.xunit.xml" "--package-name" "pilz_industrial_motion_planner" "--output-file" "/home/ubuntu/ros2_ws/build/pilz_industrial_motion_planner/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/humble/bin/ament_cppcheck" "--xunit-file" "/home/ubuntu/ros2_ws/build/pilz_industrial_motion_planner/test_results/pilz_industrial_motion_planner/cppcheck.xunit.xml" "--include_dirs" "/home/ubuntu/ros2_ws/src/moveit2/moveit_planners/pilz_industrial_motion_planner/include")
set_tests_properties([=[cppcheck]=] PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/ubuntu/ros2_ws/src/moveit2/moveit_planners/pilz_industrial_motion_planner" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;66;ament_add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;87;ament_cppcheck;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ubuntu/ros2_ws/src/moveit2/moveit_planners/pilz_industrial_motion_planner/CMakeLists.txt;215;ament_package;/home/ubuntu/ros2_ws/src/moveit2/moveit_planners/pilz_industrial_motion_planner/CMakeLists.txt;0;")
add_test([=[pep257]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/ros2_ws/build/pilz_industrial_motion_planner/test_results/pilz_industrial_motion_planner/pep257.xunit.xml" "--package-name" "pilz_industrial_motion_planner" "--output-file" "/home/ubuntu/ros2_ws/build/pilz_industrial_motion_planner/ament_pep257/pep257.txt" "--command" "/opt/ros/humble/bin/ament_pep257" "--xunit-file" "/home/ubuntu/ros2_ws/build/pilz_industrial_motion_planner/test_results/pilz_industrial_motion_planner/pep257.xunit.xml")
set_tests_properties([=[pep257]=] PROPERTIES  LABELS "pep257;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ubuntu/ros2_ws/src/moveit2/moveit_planners/pilz_industrial_motion_planner" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_pep257.cmake;41;ament_add_test;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;18;ament_pep257;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ubuntu/ros2_ws/src/moveit2/moveit_planners/pilz_industrial_motion_planner/CMakeLists.txt;215;ament_package;/home/ubuntu/ros2_ws/src/moveit2/moveit_planners/pilz_industrial_motion_planner/CMakeLists.txt;0;")
add_test([=[xmllint]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/ros2_ws/build/pilz_industrial_motion_planner/test_results/pilz_industrial_motion_planner/xmllint.xunit.xml" "--package-name" "pilz_industrial_motion_planner" "--output-file" "/home/ubuntu/ros2_ws/build/pilz_industrial_motion_planner/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/ubuntu/ros2_ws/build/pilz_industrial_motion_planner/test_results/pilz_industrial_motion_planner/xmllint.xunit.xml")
set_tests_properties([=[xmllint]=] PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ubuntu/ros2_ws/src/moveit2/moveit_planners/pilz_industrial_motion_planner" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ubuntu/ros2_ws/src/moveit2/moveit_planners/pilz_industrial_motion_planner/CMakeLists.txt;215;ament_package;/home/ubuntu/ros2_ws/src/moveit2/moveit_planners/pilz_industrial_motion_planner/CMakeLists.txt;0;")
subdirs("test")
