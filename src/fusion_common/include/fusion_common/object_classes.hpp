/**
 * @file object_classes.hpp
 * @brief Fused object classes and nominal box dimensions used for association and display.
 */
#pragma once

#include <string>
#include <vector>

namespace fusion_common
{

/// Canonical class name and nominal 3D box size.
struct ClassInfo
{
  std::string name;   // canonical fused class name
  double length;      // along heading [m]
  double width;       // [m]
  double height;      // [m]
};

/**
 * @brief Maps a detector label (COCO etc.) to the fused class.
 * @param[in] detector_label  label from the detector, e.g. "person"
 * @return one of pedestrian, bicycle, motorcycle, car, truck, bus, animal, unknown
 */
std::string canonicalClass(const std::string & detector_label);

/**
 * @brief All canonical classes.
 * @return class names, "unknown" is last (index order of the class probability vectors)
 */
const std::vector<std::string> & fusedClasses();

/**
 * @brief Nominal dimensions for a canonical class.
 * @param[in] canonical_class  fused class name
 * @return dimensions; unknown names map to a small generic 1 m box
 */
const ClassInfo & classInfo(const std::string & canonical_class);

}  // namespace fusion_common
