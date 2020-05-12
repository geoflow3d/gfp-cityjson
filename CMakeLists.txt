cmake_minimum_required(VERSION 3.14)
project (CityJSON VERSION 0.0.1)

add_definitions(-DGF_PLUGIN_NAME=\"${PROJECT_NAME}\")

find_package(geoflow REQUIRED)
find_package(nlohmann_json REQUIRED)

set(GF_PLUGIN_NAME ${PROJECT_NAME})
set(GF_PLUGIN_TARGET_NAME "gfp_cityjson")
set(GF_PLUGIN_REGISTER ${PROJECT_SOURCE_DIR}/register.hpp)
# you can provide addtional .cpp files to be compiled as argument:
geoflow_create_plugin(
)

target_link_libraries(gfp_cityjson PRIVATE geoflow-core nlohmann_json::nlohmann_json)
