#include <fstream>
#include <iomanip>
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

  class CityJSONWriterNode : public Node {

    // parameter variables
    std::string filepath_;

    public:
    using Node::Node;
    
    void init() {
      // declare ouput terminals
      add_vector_input("multisolids", typeid(std::unordered_map<int, Mesh>));
      // add_poly_input("attributes", {typeid(bool), typeid(int), typeid(float), typeid(std::string)});

      // declare parameters
      add_param(ParamPath(filepath_, "filepath", "File path"));
      // add_param(ParamInt(extract_lod_, "extract_lod", "precision"));
    }

    void add_vertices(std::map<arr3f, size_t>& vertex_map, std::vector<arr3f>& vertex_vec, std::set<arr3f>& vertex_set, const Mesh& mesh) {
      size_t v_cntr = vertex_vec.size();
      for (auto &face : mesh.get_polygons())
      {
        for (auto &vertex : face)
        {
          auto [it, did_insert] = vertex_set.insert(vertex);
          if (did_insert)
          {
            vertex_map[vertex] = v_cntr++;
            vertex_vec.push_back(vertex);
          }
        }
      }
    }

    void process() {
      // inputs
      auto& multisolids = vector_input("multisolids");

      nlohmann::json outputJSON;

      outputJSON["type"] = "CityJSON";
      outputJSON["version"] = "1.0";
      outputJSON["CityObjects"] = nlohmann::json::object();

      std::map<arr3f, size_t> vertex_map;
      std::vector<arr3f> vertex_vec;
      std::set<arr3f> vertex_set;
      size_t id_cntr = 0;

      for (size_t i=0; i<multisolids.size(); ++i) {
        auto& ms = multisolids.get<std::unordered_map<int, Mesh>>(i);

        auto building = nlohmann::json::object();
        auto b_id = std::to_string(++id_cntr);
        building["type"] = "Building";
        building["geometry"] = nlohmann::json::array();
        // building["attributes"]
        // building["children"]

        std::vector<std::string> buildingPartIds;

        for ( auto& [sid, solid] : ms ) {
          auto buildingPart = nlohmann::json::object();
          auto bp_id = std::to_string(++id_cntr);
          buildingPartIds.push_back(bp_id);
          buildingPart["type"] = "BuildingPart";
          buildingPart["parents"] = {b_id};
          add_vertices(vertex_map, vertex_vec, vertex_set, solid);
          auto geometry = nlohmann::json::object();
          geometry["type"] = "Solid";
          geometry["lod"] = 2.2;
          std::vector<std::vector<std::vector<size_t>>> exterior_shell;

          for (auto &face : solid.get_polygons())
          {
            std::vector<std::vector<size_t>> jface;
            std::vector<size_t> exterior_ring;
            for (auto &vertex : face) {
              exterior_ring.push_back(vertex_map[vertex]);
            }
            jface.emplace_back(std::move(exterior_ring));
            for (auto &iring : face.interior_rings()) {
              std::vector<size_t> interior_ring;
              for (auto &vertex : iring) {
                interior_ring.push_back(vertex_map[vertex]);
              }
              jface.emplace_back(std::move(interior_ring));
            }
            exterior_shell.emplace_back(std::move(jface));
          }

          geometry["boundaries"] = {exterior_shell};
          buildingPart["geometry"].push_back(geometry);
          outputJSON["CityObjects"][bp_id] = buildingPart;
        }

        building["children"] = buildingPartIds;

        outputJSON["CityObjects"][b_id] = building;
      }

      Box bbox;
      bbox.add(vertex_vec);
      // auto center = bbox.center();
      std::vector<std::array<int,3>>vertices_int;
      for (auto& vertex : vertex_vec) {
        vertices_int.push_back({ 
          int( vertex[0] * 1000 ),
          int( vertex[1] * 1000 ),
          int( vertex[2] * 1000 )
        });
      }
      outputJSON["vertices"] = vertices_int;
      outputJSON["transform"] = {
        {"translate", *manager.data_offset},
        {"scale", {0.001, 0.001, 0.001}}
      };

      std::ofstream ofs;
      ofs.open(manager.substitute_globals(filepath_));
      ofs << std::fixed << std::setprecision(2);
      try {
        ofs << outputJSON.dump(4);
      } catch (const std::exception& e) {
        std::cerr << e.what();
        return;
      }
    }
  };
}