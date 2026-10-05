cmake_minimum_required(VERSION 3.8)
project(ai4r_policy)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The policy node script, and the policy package it imports. The package is
# installed as the Python package `policy` on the overlay's PYTHONPATH.
install(PROGRAMS scripts/policy_node.py DESTINATION lib/${PROJECT_NAME})
ament_python_install_package(policy)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(policy_node tests/test_policy_node.py TIMEOUT 60)
  ament_add_pytest_test(policy_mpc tests/test_mpc.py TIMEOUT 60)
  ament_add_pytest_test(policy_control tests/test_control.py TIMEOUT 60)
endif()
ament_package()
