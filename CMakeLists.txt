cmake_minimum_required(VERSION 3.10)
set(CMAKE_MIN_REQ_VERSION "3.10")
# set the project name
project("Triangle")


option(GLFW_BUILD_DOCS OFF)
option(GLFW_BUILD_EXAMPLES OFF)
option(GLFW_BUILD_TESTS OFF)
add_subdirectory(Dependencies/Vendor/glfw)
add_subdirectory(Dependencies/Vendor/glad)


if(MSVC)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3 /MDd /std:c++17")

	add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
	add_definitions(-DCMAKE_GENERATOR_PLATFORM=x64)

	set (CMAKE_CXX_STANDARD 17)

else()
	if(UNIX)
		message("Linux")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -lvulkan -Wpedantic -std=c++17")

		set(GLAD_LIBRARIES g dl GL GLU glfw X11 Xrandr pthread Xi Xinerama Xcursor )

	endif()
endif()


include_directories(
					Dependencies/Include
					Dependencies/Vendor/glm
					Dependencies/Vendor/stb
					Dependencies/Vendor/imgui
					Dependencies/Vendor/imgui/backends
					Dependencies/Vendor/glfw/include
					Dependencies/Vendor/glad/include
					Dependencies/Vendor/ImGuizmo
					)


file(GLOB VENDORS_SOURCES
		Dependencies/Vendor/glad/src/glad.c
		Dependencies/Vendor/imgui/backends/imgui_impl_opengl3.cpp
		Dependencies/Vendor/imgui/backends/imgui_impl_glfw.cpp
		Dependencies/Vendor/imgui/*.cpp
		Dependencies/Vendor/ImGuizmo/*.cpp)
file(GLOB PROJECT_SOURCES Dependencies/Source/*.cpp Dependencies/Source/*.c)
file(GLOB PROJECT_HEADERS Dependencies/Include/*.hpp Dependencies/Include/*.h)

add_definitions(-DGLFW_INCLUDE_NONE -DPROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")

set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

source_group("vendorSrc" FILES ${VENDORS_SOURCES})
source_group("myInclude" FILES ${PROJECT_HEADERS})
source_group("mySrc" FILES ${PROJECT_SOURCES})

add_executable(${PROJECT_NAME} 
							${VENDORS_SOURCES} 
							${PROJECT_SOURCES}
							${PROJECT_HEADERS})

target_link_libraries(${PROJECT_NAME}
								${GLFW_LIBRARIES} 
								${GLAD_LIBRARIES} 
								glfw
								)

set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PROJECT_NAME})