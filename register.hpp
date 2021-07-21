#include "nodes.hpp"

using namespace geoflow::nodes::cityjson;

void register_nodes(geoflow::NodeRegister& node_register) {
  node_register.register_node<CityJSONReaderNode>("CityJSONReader");
  node_register.register_node<CityJSONWriterNode>("CityJSONWriter");
}