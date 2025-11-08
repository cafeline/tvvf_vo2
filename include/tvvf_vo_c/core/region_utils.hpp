#ifndef TVVF_VO_C_CORE_REGION_UTILS_HPP_
#define TVVF_VO_C_CORE_REGION_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include "tvvf_vo_c/core/field_types.hpp"

namespace tvvf_vo_c {

inline FieldRegion create_path_region(
    const Position& start,
    const Position& goal,
    double path_width,
    double margin) {
    
    const double half_width = std::max(path_width * 0.5, 0.0);
    const double safe_margin = std::max(margin, 0.0);

    FieldRegion region;
    region.min.x = std::min(start.x, goal.x) - half_width - safe_margin;
    region.max.x = std::max(start.x, goal.x) + half_width + safe_margin;
    region.min.y = std::min(start.y, goal.y) - half_width - safe_margin;
    region.max.y = std::max(start.y, goal.y) + half_width + safe_margin;
    return region;
}

}  // namespace tvvf_vo_c

#endif  // TVVF_VO_C_CORE_REGION_UTILS_HPP_
