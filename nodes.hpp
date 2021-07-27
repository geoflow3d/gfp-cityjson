#include <fstream>
#include <iomanip>
#include <ctime>
#include <filesystem>
#include <geoflow/geoflow.hpp>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

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
    std::string identifier_attribute_ = "";
    std::string referenceSystem_ = "https://www.opengis.net/def/crs/EPSG/0/7415";
    std::string citymodelIdentifier_ = "42";
    std::string datasetTitle_ = "3D BAG development";
    std::string datasetReferenceDate_ = "1970-01-01";
    std::string geographicLocation_ = "The Netherlands";

    bool prettyPrint_ = false;
    bool version_1_0_ = true;

    vec1s key_options;
    StrMap output_attribute_names;

    public:
    using Node::Node;
    
    void init() override {
      // declare ouput terminals
      add_vector_input("footprints", typeid(LinearRing));
      add_vector_input("geometry_lod12", typeid(std::unordered_map<int, Mesh>));
      add_vector_input("geometry_lod13", typeid(std::unordered_map<int, Mesh>));
      add_vector_input("geometry_lod22", typeid(std::unordered_map<int, Mesh>));
      add_poly_input("part_attributes", {typeid(bool), typeid(int), typeid(float), typeid(std::string)});
      add_poly_input("attributes", {typeid(bool), typeid(int), typeid(float), typeid(std::string)});

      // find current date
      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);

      std::ostringstream oss;
      oss << std::put_time(&tm, "%Y-%m-%d");
      datasetReferenceDate_ = oss.str();

      // declare parameters
      add_param(ParamPath(filepath_, "filepath", "File path"));
      add_param(ParamString(identifier_attribute_, "identifier_attribute", "(Renamed) attribute to use for CityObject ID (leave empty for auto ID generation). Only works for int and string attributes."));
      add_param(ParamString(referenceSystem_, "referenceSystem", "referenceSystem"));
      add_param(ParamString(citymodelIdentifier_, "citymodelIdentifier", "citymodelIdentifier"));
      add_param(ParamString(datasetTitle_, "datasetTitle", "datasetTitle"));
      add_param(ParamString(datasetReferenceDate_, "datasetReferenceDate", "datasetReferenceDate"));
      add_param(ParamString(geographicLocation_, "geographicLocation", "geographicLocation"));
      add_param(ParamBool(prettyPrint_, "prettyPrint", "Pretty print CityJSON output"));
      add_param(ParamBool(version_1_0_, "version_1_0", "Output CityJSON v1.0 instead of v1.1"));
      add_param(ParamStrMap(output_attribute_names, key_options, "output_attribute_names", "Output attribute names"));
    }

    void on_receive(gfMultiFeatureInputTerminal& it) {
      key_options.clear();
      if(&it == &poly_input("attributes")) {
        for(auto sub_term : it.sub_terminals()) {
          key_options.push_back(sub_term->get_name());
        }
      }
    };

    bool inputs_valid() override {
      return 
        input("footprints").has_data() &&
        input("geometry_lod12").has_data() &&
        input("geometry_lod13").has_data() &&
        input("geometry_lod22").has_data() &&
        poly_input("attributes").has_data()
        ;
      
    }

    void add_vertices_polygon(std::map<arr3f, size_t>& vertex_map, std::vector<arr3f>& vertex_vec, std::set<arr3f>& vertex_set, const LinearRing& polygon) {
      size_t v_cntr = vertex_vec.size();
      for (auto &vertex : polygon)
      {
        auto [it, did_insert] = vertex_set.insert(vertex);
        if (did_insert)
        {
          vertex_map[vertex] = v_cntr++;
          vertex_vec.push_back(vertex);
        }
      }
    }

    void add_vertices_mesh(std::map<arr3f, size_t>& vertex_map, std::vector<arr3f>& vertex_vec, std::set<arr3f>& vertex_set, const Mesh& mesh) {
      for (auto &face : mesh.get_polygons())
      {
        add_vertices_polygon(vertex_map, vertex_vec, vertex_set, face);
      }
    }

    std::vector<std::vector<size_t>> LinearRing2jboundary(std::map<arr3f, size_t>& vertex_map, const LinearRing& face) {
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
      return jface;
    }

    nlohmann::json::object_t mesh2jSolid(const Mesh& mesh, const char* lod, std::map<arr3f, size_t>& vertex_map) {
      auto geometry = nlohmann::json::object();
      geometry["type"] = "Solid";
      if(version_1_0_) {
        geometry["lod"] = atof(lod);
      } else {
        geometry["lod"] = lod;
      };
      std::vector<std::vector<std::vector<size_t>>> exterior_shell;

      for (auto &face : mesh.get_polygons())
      {
        exterior_shell.emplace_back( LinearRing2jboundary(vertex_map, face) );
      }
      geometry["boundaries"] = {exterior_shell};

      auto surfaces = nlohmann::json::array();
      surfaces.push_back(nlohmann::json::object({{
        "type", "GroundSurface"
      }}));
      surfaces.push_back(nlohmann::json::object({{
        "type", "RoofSurface"
      }}));
      surfaces.push_back(nlohmann::json::object({{
        "type", "+WallSurface_Outer"
      }}));
      surfaces.push_back(nlohmann::json::object({{
        "type", "+WallSurface_Inner"
      }}));
      geometry["semantics"] = {
        {"surfaces", surfaces},
        {"values", {mesh.get_labels()}}
      };
      return geometry;
    }

    void process() override {
      // inputs
      auto& footprints = vector_input("footprints");

      nlohmann::json outputJSON;

      outputJSON["type"] = "CityJSON";
      if (version_1_0_) {
        outputJSON["version"] = "1.0";
      } else {
        outputJSON["version"] = "1.1";
      };
      outputJSON["CityObjects"] = nlohmann::json::object();

      std::map<arr3f, size_t> vertex_map;
      std::vector<arr3f> vertex_vec;
      std::set<arr3f> vertex_set;
      size_t id_cntr = 0;
      size_t bp_counter = 0;
      std::string identifier_attribute = manager.substitute_globals(identifier_attribute_);

      auto& multisolids_lod12 = vector_input("geometry_lod12");
      auto& multisolids_lod13 = vector_input("geometry_lod13");
      auto& multisolids_lod22 = vector_input("geometry_lod22");
      for (size_t i=0; i<multisolids_lod22.size(); ++i) {

        auto building = nlohmann::json::object();
        auto b_id = std::to_string(++id_cntr);
        building["type"] = "Building";
        // building["attributes"]
        // building["children"]

        // Building atributes
        bool id_from_attr = false;
        auto jattributes = nlohmann::json::object();
        for (auto& term : poly_input("attributes").sub_terminals()) {
          if (!term->get_data_vec()[i].has_value()) continue;
          auto tname = term->get_name();
          
          //see if we need to rename this attribute
          auto search = output_attribute_names.find(tname);
          if(search != output_attribute_names.end()) {
            //ignore if the new name is an empty string
            if(search->second.size()!=0)
              tname = search->second;
          }

          if (term->accepts_type(typeid(bool))) {
            jattributes[tname] = term->get<const bool&>(i);
          } else if (term->accepts_type(typeid(float))) {
            jattributes[tname] = term->get<const float&>(i);
            if (tname == identifier_attribute) {
              b_id = std::to_string(term->get<const float&>(i));
            }
          } else if (term->accepts_type(typeid(int))) {
            jattributes[tname] = term->get<const int&>(i);
            if (tname == identifier_attribute) {
              b_id = std::to_string(term->get<const int&>(i));
              id_from_attr = true;
            }
          } else if (term->accepts_type(typeid(std::string))) {
            jattributes[tname] = term->get<const std::string&>(i);
            if (tname == identifier_attribute) {
              b_id = term->get<const std::string&>(i);
              id_from_attr = true;
            }
          }
        }

        building["attributes"] = jattributes;
        
        // footprint geometry
        auto fp_geometry = nlohmann::json::object();
        if (version_1_0_) { 
          fp_geometry["lod"] = 0;
        } else {
          fp_geometry["lod"] = "0";
        }
        fp_geometry["type"] = "MultiSurface";

        auto& footprint = footprints.get<LinearRing>(i);
        add_vertices_polygon(vertex_map, vertex_vec, vertex_set, footprint);
        fp_geometry["boundaries"] = {LinearRing2jboundary(vertex_map, footprint)};
        building["geometry"].push_back(fp_geometry);
        // building["geometry"] = nlohmann::json::array();

        std::vector<std::string> buildingPartIds;

        // geometries
        const auto& solids_lod12 = multisolids_lod12.get<std::unordered_map<int, Mesh>>(i);
        const auto& solids_lod13 = multisolids_lod13.get<std::unordered_map<int, Mesh>>(i);

        for ( const auto& [sid, solid_lod22] : multisolids_lod22.get<std::unordered_map<int, Mesh>>(i) ) {
          auto buildingPart = nlohmann::json::object();
          auto bp_id = b_id + "-" + std::to_string(sid);
          
          buildingPartIds.push_back(bp_id);
          buildingPart["type"] = "BuildingPart";
          buildingPart["parents"] = {b_id};

         
          add_vertices_mesh(vertex_map, vertex_vec, vertex_set, solids_lod12.at(sid));
          add_vertices_mesh(vertex_map, vertex_vec, vertex_set, solids_lod13.at(sid));
          add_vertices_mesh(vertex_map, vertex_vec, vertex_set, solid_lod22);
          
          buildingPart["geometry"].push_back(mesh2jSolid(solids_lod12.at(sid), "1.2", vertex_map));
          buildingPart["geometry"].push_back(mesh2jSolid(solids_lod13.at(sid), "1.3", vertex_map));
          buildingPart["geometry"].push_back(mesh2jSolid(solid_lod22, "2.2", vertex_map));

          //attrubutes
          auto jattributes = nlohmann::json::object();
          for (auto& term : poly_input("part_attributes").sub_terminals()) {
            if (!term->get_data_vec()[i].has_value()) continue;
            auto tname = term->get_name();
            if (term->accepts_type(typeid(bool))) {
              jattributes[tname] = term->get<const bool&>(bp_counter);
            } else if (term->accepts_type(typeid(float))) {
              jattributes[tname] = term->get<const float&>(bp_counter);
            } else if (term->accepts_type(typeid(int))) {
              jattributes[tname] = term->get<const int&>(bp_counter);
            } else if (term->accepts_type(typeid(std::string))) {
              jattributes[tname] = term->get<const std::string&>(bp_counter);
            }
          }
          ++bp_counter;
          buildingPart["attributes"] = jattributes;

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

      // metadata
      auto metadata = nlohmann::json::object();
      auto minp = bbox.min();
      auto maxp = bbox.max();
      metadata["geographicalExtent"] = {
        minp[0]+(*manager.data_offset)[0],
        minp[1]+(*manager.data_offset)[1],
        minp[2]+(*manager.data_offset)[2],
        maxp[0]+(*manager.data_offset)[0],
        maxp[1]+(*manager.data_offset)[1],
        maxp[2]+(*manager.data_offset)[2]
      };

      // TODO create node parameters for these
      metadata["referenceSystem"] = referenceSystem_;
      metadata["citymodelIdentifier"] = citymodelIdentifier_;
      metadata["datasetTitle"] = datasetTitle_;
      metadata["datasetReferenceDate"] = datasetReferenceDate_;
      metadata["geographicLocation"] = geographicLocation_;
      // "metadata":{"geographicalExtent":[84372.90299658204,446339.80099951173,-1.6206239461898804,85051.81354956055,447006.0341881409,35.51251220703125],"citymodelIdentifier":"6118726d-ed69-4c62-8eb6-0b39f3a8623e","datasetReferenceDate":"2021-03-04","datasetCharacterSet":"UTF-8","datasetTopicCategory":"geoscientificInformation","distributionFormatVersion":"1.0","spatialRepresentationType":"vector","metadataStandard":"ISO 19115 - Geographic Information - Metadata","metadataStandardVersion":"ISO 19115:2014(E)","metadataCharacterSet":"UTF-8","metadataDateStamp":"2021-03-04","textures":"absent","materials":"absent","cityfeatureMetadata":{"Building":{"uniqueFeatureCount":1304,"aggregateFeatureCount":3912,"presentLoDs":{"1.2":1304,"1.3":1304,"2.2":1304}}},"presentLoDs":{"1.2":1304,"1.3":1304,"2.2":1304},"thematicModels":["Building"],"referenceSystem":"urn:ogc:def:crs:EPSG::7415","fileIdentifier":"5907.json"}

      outputJSON["metadata"] = metadata;

      auto fname = fs::path(manager.substitute_globals(filepath_));
      fs::create_directories(fname.parent_path());

      std::ofstream ofs;
      ofs.open(fname);
      ofs << std::fixed << std::setprecision(2);
      try {
        if (prettyPrint_)
          ofs << outputJSON.dump(2);
        else
          ofs << outputJSON;
      } catch (const std::exception& e) {
        std::cerr << e.what();
        return;
      }
    }
  };
}