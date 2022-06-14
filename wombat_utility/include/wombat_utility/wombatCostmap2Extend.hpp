#include <iostream>
#include <vector>
#include <wombat_utility/wombat_types.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace wombat{


class Costmap2Extend
{
public:
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

private:


  nav2_costmap_2d::Costmap2D* _costmap;

};




} //namespace wombat
