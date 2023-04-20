#pragma once

#include <iostream>
#include <vector>
#include <cstdint>
#include <wombat_utility/wombat_types.hpp>
#include <wombat_utility/wombatPolygon.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace wombat{


class Costmap2Extend
{
public:
  using SharedPtr = std::shared_ptr<Costmap2Extend>;

  Costmap2Extend(nav2_costmap_2d::Costmap2D* costmap) : 
    _costmap(costmap)
  {
    
  }
  
  ~Costmap2Extend()
  {}

  Point2 mapToWorld(const Pixel2& map_cell) const
  { 
    Point2 ret;
    _costmap->mapToWorld(map_cell.x(), map_cell.y(), ret.x(), ret.y());
    return ret;
  }

  Pixel2 worldToMap(const Point2& world_point) const
  {
    unsigned int x;
    unsigned int y;
    _costmap->worldToMap(world_point.x(), world_point.y(), x, y);
    return Pixel2(static_cast<int>(x), static_cast<int>(y));
  }

  Pixel2 worldToMapEnforceBounds(const Point2& world_point) const
  {
    Pixel2 ret;
    _costmap->worldToMapEnforceBounds(world_point.x(), world_point.y(), ret.x(), ret.y());
    return ret;
  }

  PolygonPixel polygon2dToPolygonPixel(const Polygon2d& polygon) const
  {
    PolygonPixel ret;
    ret.header() = polygon.header();
    for(const auto& point : polygon.points())
    {
      ret.push_back(worldToMap(point));
      //todo check if enfoce bounds is needed or is reasonable
    }
    return ret;
  }

  Polygon2d polygonPixelToPolygon2d(const PolygonPixel& polygon) const
  {
    Polygon2d ret;
    ret.header() = polygon.header();
    for(const auto& point : polygon.points())
    {
      ret.push_back(mapToWorld(point));
    }
    return ret;
  }

  std::vector<Pixel2> pointsInPolygon(const PolygonPixel& polygon) const
  {
    // auto rect = polygon.bounding_box();

    // for(int x = rect.p().x(); x < rect.width(); x++)
    // { 
    //   for(int y = rect.p().y(); y < rect.height(); y++)
    //   {
    //     if(polygon.is_inside(Pixel2(x,y)))
    //     {
    //       ret.emplace_back(Pixel2(x,y));
    //     }
    //   }
    // }

    //use costmap2d polygon stuff
    // _costmap->con
    // std::cout << "covert to map loc" << std::endl;
    auto ml_points = Pixel2::toMapLocationList(polygon.points());
    // std::cout << "find cells in polygon" << std::endl;
    std::vector<nav2_costmap_2d::MapLocation> out_points;
    _costmap->convexFillCells(ml_points, out_points);


    return Pixel2::fromMapLocationList(out_points);
  }

  std::vector<Pixel2> pointsInPolygon(const Polygon2d& polygon) const
  {
    return pointsInPolygon(polygon2dToPolygonPixel(polygon));
  }

  std::vector<Pixel2> traceLine(const Pixel2& start, const Pixel2& end) const
  {
    std::vector<Pixel2> ret;

    (void)start; //unused
    (void)end;   //unused

    // //basic line drawing
    // auto dx = end.x() - start.x();
    // auto dy = end.y() - start.y();

    // auto x_dominant = std::abs(dx) > std::abs(dy);

    // int incr = 0;
    // auto dn = x_dominant ? dx : dy;

    // incr = dn > 0 ? 1 : -1;

    // auto simpleLine = [](const int dx, const int dy, const int xs, const int xe, const int ys, const int icnr) -> std::vector<Pixel2>
    // {
    //   std::vector<Pixel2> line;
    //   for(int x = xs; x < xe; x++)
    //   {
    //     int y = ys + dy * (x - xs) / dx;
    //     line.push_back(Pixel2(x, y));
    //   }

    //   return line
    // }


    return ret;
  }



  uint8_t getCost(const Pixel2& pixel) const
  {
    return _costmap->getCost(pixel.x(), pixel.y());
  }

  std::vector<uint8_t> getCost(const std::vector<Pixel2>& pixels) const
  {
    std::vector<uint8_t> ret;
    ret.reserve(pixels.size());
    for(const auto& pixel : pixels)
    {
      ret.emplace_back(getCost(pixel));
    }
    return ret;
  }
  
  void setCost(const Pixel2& pixel, const uint8_t cost)
  {
    _costmap->setCost(pixel.x(), pixel.y(), cost);
  }

  nav2_costmap_2d::Costmap2D& costmap()
  {
    return *_costmap;
  }


private:
  nav2_costmap_2d::Costmap2D* _costmap;
};




} //namespace wombat
