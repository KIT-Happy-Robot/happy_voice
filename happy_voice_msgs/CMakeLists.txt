cmake_minimum_required(VERSION 3.8)
project(happy_voice_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)  # アクションインターフェースに必要

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TextToSpeech.srv"
  "srv/SpeechToText.srv"
  "srv/YesNo.srv"
  "action/YesNo.action"  # アクションファイルを追加
  DEPENDENCIES builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)  # 実行時依存関係をエクスポート

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
