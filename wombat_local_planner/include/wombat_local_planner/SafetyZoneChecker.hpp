#pragma once


#include <iostream>
#include <vector>
#include <wombat_utility/wombat_types.hpp>
#include <wombat_utility/wombatCostmap2Extend.hpp>
#include <wombat_utility/wombatPolygon.hpp>
// #include <nav2_costmap_2d/costmap_math.hpp>
#include <nav2_costmap_2d/cost_values.hpp>


namespace wombat{

class SafetyZoneChecker
{
public:
  SafetyZoneChecker()
  { }
  ~SafetyZoneChecker()
  { }

  static inline double check(const Polygon2d& zone, const Costmap2Extend::SharedPtr& costmap)
  {
    double val = 0.0;

    //get zone in pixel
    PolygonPixel zone_pixels = costmap->polygon2dToPolygonPixel(zone);
    //get pixels in zone
    auto in_pixel = costmap->pointsInPolygon(zone_pixels);

    //get cell values
    auto costs = costmap->getCost(in_pixel);

    for(auto& e : costs)
    {
      // auto hans = nav2_costmap_2d::LETHAL_OBSTACLE;
      if(e == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        val += 1.0;
      }
    }

    return val;
  }
  
  private:
};


} //namespace wombat