#include "fusion_common/object_classes.hpp"

#include <unordered_map>

namespace fusion_common
{

// Lookup table detector label -> fused class.
std::string canonicalClass(const std::string & l)
{
  static const std::unordered_map<std::string, std::string> map = {
    {"person", "pedestrian"}, {"pedestrian", "pedestrian"},
    {"bicycle", "bicycle"}, {"motorcycle", "motorcycle"}, {"motorbike", "motorcycle"},
    {"car", "car"}, {"truck", "truck"}, {"bus", "bus"},
    {"dog", "animal"}, {"cat", "animal"}, {"cow", "animal"}, {"horse", "animal"},
  };
  auto it = map.find(l);
  return it == map.end() ? "unknown" : it->second;
}

// Fixed class list shared by fusion, messages and visualization.
const std::vector<std::string> & fusedClasses()
{
  static const std::vector<std::string> c = {
    "pedestrian", "bicycle", "motorcycle", "car", "truck", "bus", "animal", "unknown"};
  return c;
}

// Lookup table of nominal (length, width, height).
const ClassInfo & classInfo(const std::string & c)
{
  // typical dimensions (length, width, height)
  static const std::unordered_map<std::string, ClassInfo> table = {
    {"pedestrian", {"pedestrian", 0.6, 0.6, 1.75}},
    {"bicycle", {"bicycle", 1.8, 0.6, 1.6}},
    {"motorcycle", {"motorcycle", 2.1, 0.8, 1.5}},
    {"car", {"car", 4.5, 1.8, 1.5}},
    {"truck", {"truck", 8.0, 2.5, 3.2}},
    {"bus", {"bus", 12.0, 2.55, 3.2}},
    {"animal", {"animal", 1.0, 0.4, 0.8}},
    {"unknown", {"unknown", 1.0, 1.0, 1.0}},
  };
  auto it = table.find(c);
  return it == table.end() ? table.at("unknown") : it->second;
}

}  // namespace fusion_common
