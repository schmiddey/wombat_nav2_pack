#pragma once

#include <ostream>

#include <limits>

#include <wombat_utility/wombat_types.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>


namespace wombat{



template<class P>
class Polygon
{
public:
  Polygon() = default;
  Polygon(const Polygon<P>&) = default;
  Polygon(Polygon<P>&&) = default;
  Polygon& operator=(const Polygon<P>&) = default;
  Polygon& operator=(Polygon<P>&&) = default;

  Polygon(const std::vector<P>& points): _points(points) { }
  Polygon(const geometry_msgs::msg::Polygon& polygon)
  {
    this->fromRosPolygon(polygon);
  }

  Polygon(const std::vector<geometry_msgs::msg::Point>& ros_points)
  {
    this->fromRosPoints(ros_points);
  }

  ~Polygon() = default;

  bool is_inside(const P& p) const
  {
    //   int i, j, c = 0;
    //   for(i = 0, j = nvert-1; i < nvert; j = i++) {
    //     if ( ((verty[i]>testy) != (verty[j]>testy)) &&
    //     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
    //        c = !c;
    //   }
    //
    unsigned int i,m;
    bool c = false;
    const std::vector<P>& ps = _points;
    for(i=0, m=ps.size()-1; i<ps.size(); m=i++)
    {
      if( ((ps[i].y() > p.y()) != (ps[m].y() > p.y())) &&
          (p.x() < (ps[m].x() - ps[i].x()) * (p.y() - ps[i].y()) / (ps[m].y() - ps[i].y()) + ps[i].x()) )
        c = !c;
    }
    return c;
  }

  P centroid() const
  {
    //just implemented formula...
    P center;
    if(_points.empty())
       return center;
 
    auto points = _points; //tmp copy
    points.push_back(points[0]); //make last element as same as first element... needed for algorithm
    //std::reverse(p.points.begin(),p.points.end());
    double xs = 0;
    double ys = 0;
    for(unsigned int i=0; i<points.size() - 1; ++i)
    {
       double fac = (points[i].x() * points[i+1].y() - points[i+1].x() * points[i].y());
       xs += (points[i].x() + points[i+1].x()) * fac;
       ys += (points[i].y() + points[i+1].y()) * fac;
    }
 
    double area = this->area();
    if(area == 0.0)
       return center;
    center.x() = xs / (area * 6);
    center.y() = ys / (area * 6);
 
    //std::cout << "debug: center:" << center << std::endl;
 
    return center;
  }

  P mean_center() const
  {
    P center;
    if(_points.empty())
       return center;
 
    for(auto& p : _points)
    {
       center.x() += p.x();
       center.y() += p.y();
    }
 
    center *= 1.0 / _points.size();
 
    return center;
  }

  double area() const
  {
    //Gauss'sche Dreiecksformel, just implemented formula...
    double area = 0.0;
    if(_points.empty())
      return area;
    auto points = _points; //copy
    points.push_back(_points[0]); //make last element as same as first element... needed for algorithm

    //std::reverse(p.points.begin(),p.points.end());
    for(unsigned int i=0; i<points.size() - 1; ++i)
    {
       area += points[i].x() * points[i+1].y() - points[i+1].x() * points[i].y();
    }
    //std::cout << "debug: area: " << area * 0.5 << std::endl;
    return std::abs(area * 0.5);
  }

  void scale(const double factor)
  {
    (void)factor; //todo
    auto c = this->centroid();
    Eigen::Matrix2Xd m(2, _points.size());
    int i = 0;
    for(auto e : _points)
    {
      //translate by c
      m(0,i)   = e.x() - c.x();
      m(1,i++) = e.y() - c.y();
    }
    Eigen::Matrix2d aff;
    aff(0,0) = (factor);
    aff(1,1) = (factor);

    Eigen::MatrixXd res = aff * m;

    for(unsigned int i = 0; i < _points.size(); i++)
    {
      _points[i].x() = res(0,i) + c.x();
      _points[i].y() = res(1,i) + c.y();
    }
  }

  Polygon<P> scaled(const double factor) const
  {
    Polygon<P> p(*this);
    p.scale(factor);
    return p;
  }

  Rect2<P> bounding_box() const
  {
    using val_t = typename P::value_type;
    val_t min_x = std::numeric_limits<val_t>::max();
    val_t min_y = std::numeric_limits<val_t>::max();
    val_t max_x = std::numeric_limits<val_t>::min();
    val_t max_y = std::numeric_limits<val_t>::min();
 
    //todo use std::min and std::max

    for(auto e : _points)
    {
       if(min_x > e.x())
          min_x = e.x();
       if(min_y > e.y())
          min_y = e.y();
       if(max_x < e.x())
          max_x = e.x();
       if(max_y < e.y())
          max_y = e.y();
    }
 

    Rect2<P> rect(P(min_x, min_y), max_x - min_x, max_y - min_y);

    return rect;
  }

  geometry_msgs::msg::PolygonStamped toRosPolygon(const std::string& frame, const rclcpp::Time& stamp) const
  {
    geometry_msgs::msg::PolygonStamped ret;
    ret.polygon.points.reserve(_points.size());

    ret.header.frame_id = frame;
    ret.header.stamp = stamp;
    for(auto e : _points)
    {
       geometry_msgs::msg::Point32 p;
       p.x = e.x();
       p.y = e.y();
       p.z = 0.0; // not used;
       ret.polygon.points.emplace_back(p);
    }
    return ret;
  }

  std::vector<geometry_msgs::msg::Point> toRosPoints() const
  {
    std::vector<geometry_msgs::msg::Point> ret;
    ret.reserve(_points.size());
    for(auto e : _points)
    {
       geometry_msgs::msg::Point p;
       p.x = e.x();
       p.y = e.y();
       p.z = 0.0; // not used;
       ret.emplace_back(p);
    }
    return ret;
  }

  void fromRosPolygon(const geometry_msgs::msg::Polygon& ros_polygon)
  {
    _points.clear();
    for(auto& p : ros_polygon.points)
    {
      _points.emplace_back(p.x, p.y);
    }
  }

  void fromRosPoints(const std::vector<geometry_msgs::msg::Point>& points)
  {
    _points.clear();
    for(auto& p : points)
    {
      _points.emplace_back(p.x, p.y);
    }
  }

 
  const std::vector<P>& points() const
  {
    return _points;
  }

  void push_back(const P& p)
  {
    _points.push_back(p);
  }

  void pop_back()
  {
    _points.pop_back();
  }

  void clear()
  {
    _points.clear();
  }

  std::size_t size() const
  {
    return _points.size();
  }

  bool empty() const
  {
    return _points.empty();
  }

  std_msgs::msg::Header& header()
  {
    return _hdr;
  }

  const std_msgs::msg::Header& header() const
  {
    return _hdr;
  }

private:
  std::vector<P> _points;
  std_msgs::msg::Header _hdr;
};


using Polygon2d = Polygon<Point2>;
using PolygonPixel = Polygon<Pixel2>;

} //namespace wombat



//ostream operator for polygon
std::ostream& operator<<(std::ostream& os, const wombat::Polygon2d& pol)
{
  for(auto& e : pol.points())
  {
    os << e << ",  ";
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const wombat::PolygonPixel& pol)
{
  for(auto& e : pol.points())
  {
    os << e << ",  ";
  }
  return os;
}