#define CATCH_CONFIG_MAIN 

#include "catch2/catch.hpp"

#include <wombat_utility/wombatPolygon.hpp>
#include <wombat_utility/utility.hpp>

wombat::PolygonPixel getPolygonPix()
{
  wombat::PolygonPixel poly;
  poly.push_back(wombat::Pixel2(0, 0));
  poly.push_back(wombat::Pixel2(0, 2));
  poly.push_back(wombat::Pixel2(2, 2));
  poly.push_back(wombat::Pixel2(2, 0));
  return poly;
}

wombat::Polygon2d getPolygon2d()
{
  wombat::Polygon2d poly;
  poly.push_back(wombat::Point2(0, 0));
  poly.push_back(wombat::Point2(0, 2));
  poly.push_back(wombat::Point2(2, 2));
  poly.push_back(wombat::Point2(2, 0));
  return poly;
}


TEST_CASE("Polygon stuff", "[wombat_utility_1]"){

  //create a polygon
  wombat::PolygonPixel poly = getPolygonPix();


  //check the area
  REQUIRE(poly.area() == 4);
  REQUIRE(poly.size() == 4);
  // SECTION("Hans")
  REQUIRE(poly.is_inside(wombat::Pixel2(5,3)) == false);
  REQUIRE(poly.is_inside(wombat::Pixel2(1,1)) == true);

}


TEST_CASE("more poly stuff", "[wombat_utility_2]"){

  //create a polygon
  wombat::Polygon2d poly = getPolygon2d();

  //check the area
  REQUIRE(poly.area() == 4);
  REQUIRE(poly.size() == 4);
  // SECTION("Hans")
  REQUIRE(poly.is_inside(wombat::Point2(1.1,1.1)) == true);

  auto scaled = poly.scaled(2.0);
  REQUIRE(scaled.area() == 16);

}


TEST_CASE("poly fom string", "[wombat_utility_3]")
{
  std::string poly_string("[[-0.450, -0.3], [-0.450, 0.3], [0.450, 0.3], [0.450, -0.3]]");
  auto poly = wombat::Utility::createFootprint(poly_string);

  REQUIRE(poly.size() == 4);


  REQUIRE(poly.points()[0].x() == Approx(-0.450));
  REQUIRE(poly.points()[0].y() == Approx(-0.3));
  REQUIRE(poly.points()[1].x() == Approx(-0.450));
  REQUIRE(poly.points()[1].y() == Approx(0.3));
  REQUIRE(poly.points()[2].x() == Approx(0.450));
  REQUIRE(poly.points()[2].y() == Approx(0.3));
  REQUIRE(poly.points()[3].x() == Approx(0.450));
  REQUIRE(poly.points()[3].y() == Approx(-0.3));



}

TEST_CASE("boundingBox", "[wombat_utility_4]")
{
  auto poly = getPolygon2d();

  // auto bb = poly.bounding_box();

  wombat::Rect2<wombat::Point2> rect;
  rect = poly.bounding_box();

  // REQUIRE(bb.min().x() == Approx(0));
  // REQUIRE(bb.min().y() == Approx(0));
  // REQUIRE(bb.max().x() == Approx(2));
  // REQUIRE(bb.max().y() == Approx(2));

}