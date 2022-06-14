#include <iostream>
#include <memory>
#include <vector>



#include <wombat_utility/wombatLocalPath2.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <wombat_utility/wombatPath2.hpp>
#include <wombat_utility/wombatCostmap2Extend.hpp>

namespace wombat{


/**
 * @brief Class for checking collision with obstacles based on costmap
 * 
 * check each path element in costmap
 * also check the distance between following path elements -> if they are greater than a defined threshhold then interpolate as line (using fnc from costmap2d)
 +  --> distance is measured in integer steps
 */
class CollisionChecker{
public:
  CollisionChecker(wombat::Costmap2Extend& costmap, std::shared_ptr<wombat::LocalPath2>& local_path) : 
    _costmap(costmap),
    _local_path(local_path)
  {

  }

  ~CollisionChecker()
  { }
  
  /**
   * @brief check local path for collisions and returns a slowness factor 0..1 (1 = no slowness, 0 =  full stop)
   * 
   * @return double 
   */
  double checkCollision()
  {
    // _costmap.getCost(0,0);

    return 0.0;
  }

private:
  std::vector<nav2_costmap_2d::MapLocation> pathToMapCells(const wombat::Path2& path)
  {
    std::vector<nav2_costmap_2d::MapLocation> path_cells;
    if(path.empty())
    {
      return path_cells;
    }

    (void)_costmap;

    // path_cells.reserve(path.size());
    // for(const auto& p : path.poses())
    // {
    //   nav2_costmap_2d::MapLocation map_cell;
    //   _costmap.worldToMap(p.position.x(), p.position.y(), map_cell.x, map_cell.y);
    //   path_cells.push_back(map_cell);
    // }

    // if(_max_cell_distance)
    // {//check distance between cells
    //   return checkInterpolate(path_cells);
    // }
    return path_cells;
  }

  // std::vector<nav2_costmap_2d::MapLocation> checkInterpolate(const std::vector<nav2_costmap_2d::MapLocation>& path_cells)
  // {

  //   for(auto it = path_cells.begin() + 1; it != path_cells.end(); ++it)
  //   {
  //     //compute dist
  //     auto dist = std::max(it->x - (it-1)->x, it->y - (it-1)->y);
  //     if(dist > _max_cell_distance)
  //     {//if dist to great than raytraceLine and use this cells
  //       // _costmap.raytraceLine(take_cell, it->x, it->y, (it-1)->x, (it-1)->y);
  //       // _costmap.traceLine(const Pixel2 &start, const Pixel2 &end)
  //     }
  //   }
  //   return interpolated_cells;
  // }

private:
  wombat::Costmap2Extend& _costmap;
  std::shared_ptr<wombat::LocalPath2> _local_path; //todo maybe use path2d obj no local path


  // std::size_t _max_cell_distance = 2;   //if 0 then no check + interpolation
  

};


} //namespace wombat
