#include <fstream>
#include <geoflow/geoflow.hpp>
#include <nlohmann/json.hpp>

namespace geoflow::nodes::cityjson {
  
  static std::unordered_map <std::string, int> st_map = 
  {
    {"RoofSurface", 0},
    {"GroundSurface",1},
    {"WallSurface", 2},
    {"ClosureSurface", 3},
    {"OuterCeilingSurface", 4},
    {"OuterFloorSurface", 5},
    {"Window", 6},
    {"Door", 7}
  };

  class CityJSONReaderNode : public Node {

    // parameter variables
    std::string filepath_;
    int extract_lod_ = 2;

    public:
    using Node::Node;
    
    void init() {
      // declare ouput terminals
      add_vector_output("faces", typeid(LinearRing));
      add_vector_output("surface_types", typeid(int));

      // declare parameters
      add_param(ParamPath(filepath_, "filepath", "File path"));
      add_param(ParamInt(extract_lod_, "extract_lod", "precision"));
    }

    void process() {
      // get filepath from paramter

      // read json file from disk
      std::ifstream inputStream(filepath_);
      nlohmann::json json;
      try {
        inputStream >> json;
      } catch (const std::exception& e) {
        std::cerr << e.what();
        return;
      }

      // extract geometries
      // WARNING: this is code is only written to work with the dataset 'DenHaag_01.json', expect crashes with other files
      std::vector<std::vector<double>> verts = json["vertices"];
      std::vector<double> scale = json["transform"]["scale"];

      auto& faces = vector_output("faces");
      auto& surface_types = vector_output("surface_types");
      for (const auto& cobject : json["CityObjects"]) {
        // iterate all geometries
        for (const auto& geom : cobject["geometry"]) {
          if (
            geom["type"] == "Solid" && // only care about solids
            geom["lod"] == extract_lod_ // of this LoD
          ) {
            size_t face_cnt = 0;
            // get faces of exterior shell
            for (const auto& ext_face : geom["boundaries"][0]) {
              LinearRing ring;
              for (const auto& i : ext_face[0]) { // get vertices of outer rings
                ring.push_back({
                  float(verts[i][0] * scale[0]), 
                  float(verts[i][1] * scale[1]), 
                  float(verts[i][2] * scale[2])
                });
                // get the surface type
              }
              int value = geom["semantics"]["values"][0][face_cnt++];
              const std::string type_string = geom["semantics"]["surfaces"][value]["type"];
              surface_types.push_back(st_map[type_string]);
              faces.push_back(ring);
            }
          }
        }
      }
    }
  };
}