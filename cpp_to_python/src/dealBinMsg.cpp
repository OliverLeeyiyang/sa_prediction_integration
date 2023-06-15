#include "dealBinMsg.h"
#include <boost/python.hpp>



void fromBinMsg(const autoware_auto_mapping_msgs::msg::HADMapBin & msg, lanelet::LaneletMapPtr map)
{
  if (!map) {
    std::cerr << __FUNCTION__ << ": map is null pointer!";
    return;
  }

  std::string data_str;
  data_str.assign(msg.data.begin(), msg.data.end());

  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive oa(ss);
  oa >> *map;
  lanelet::Id id_counter = 0;
  oa >> id_counter;
  lanelet::utils::registerId(id_counter);
  // *map = std::move(laneletMap);
}



BOOST_PYTHON_MODULE(dealBinMsg)
{
  using namespace boost::python;
  def("fromBinMsg", fromBinMsg);
}
