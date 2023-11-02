#include <freespace_planning_algorithms/astar_search.hpp>
#include <freespace_planning_algorithms/abstract_algorithm.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/python.hpp>
#include <boost/bind/bind.hpp>


using namespace freespace_planning_algorithms;

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME)
{
  using namespace boost::python;
  class_<AstarSearch, bases<AbstractPlanningAlgorithm>>(
    "AstarSeach", init<PlannerCommonParam, VehicleShape, AstarParam>())
    // .def(init<PlannerCommonParam, VehicleShape, rclcpp::Node>())
    .def("set_map", &AstarSearch::setMap)
    .def("make_plan", &AstarSearch::makePlan);
}