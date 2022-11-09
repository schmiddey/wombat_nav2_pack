#test
find_package(Catch2 REQUIRED)
find_package(ament_cmake_test REQUIRED)

add_executable(polygon_test test_polygon.cpp)
# add_executable(polygon_test2 test_polygon.cpp)
# target_link_libraries(polygon_test PRIVATE Catch2::Catch2)
ament_target_dependencies(polygon_test ${deps} Catch2)
# ament_target_dependencies(polygon_test2 ${deps} Catch2)


include(CTest)
add_test(NAME pt COMMAND polygon_test)
# add_test(NAME pt2 COMMAND polygon_test2)

# ament_add_test("pt" COMMAND "test/python_test") 
# include(ParseAndAddCatchTests)
# ParseAndAddCatchTests(polygon_test)