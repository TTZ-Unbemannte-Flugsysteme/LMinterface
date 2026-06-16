cmake_minimum_required(VERSION 3.8)
project(cocoa_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Declare message files
set(msg_files
  "msg/SingleIntent.msg"
  "msg/Intent.msg"
  "msg/Context.msg"
  "msg/ActionPlan.msg"
  "msg/TaskStatus.msg"
  "msg/ExecutionFeedback.msg"
  "msg/UserFeedback.msg"
  "msg/ThesisMetrics.msg"
)

# Declare service files
set(srv_files
  "srv/QueryEKG.srv"
  "srv/QueryLGA.srv"
  "srv/PlanAction.srv"
  "srv/PlanPath.srv"
  "srv/GetKnowledgeSummary.srv"
  "srv/GetAllObjects.srv"
)

# Declare action files
set(action_files
  "action/ExecuteTask.action"
  "action/DroneCommand.action"
)

# Generate interfaces (messages, services, actions)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
