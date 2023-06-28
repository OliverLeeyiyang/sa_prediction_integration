#ifndef HADMAP2OSM__2OSM_HPP_
#define HADMAP2OSM__2OSM_HPP_

#include "rclcpp/rclcpp.hpp"
#include <lanelet2_core/LaneletMap.h>
#include <boost/archive/binary_iarchive.hpp>
#include "lanelet2_io/Io.h"

#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

using autoware_auto_mapping_msgs::msg::HADMapBin;

namespace
{
  std::string tempfile(const std::string &name)
  {
    char tmpDir[] = "/tmp/lanelet2_example_XXXXXX";
    auto *file = mkdtemp(tmpDir);
    if (file == nullptr)
    {
      throw lanelet::IOError("Failed to open a temporary file for writing");
    }
    return std::string(file) + '/' + name;
  }
} // namespace

class HADMap2osm : public rclcpp::Node
{
public:
  HADMap2osm();

private:
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  lanelet::Id id_counter;
  std::string topic_name;

  void mapCallback(const HADMapBin::ConstSharedPtr msg);

  void fromBinMsg(const autoware_auto_mapping_msgs::msg::HADMapBin &msg, lanelet::LaneletMapPtr map);
};

#endif // HADMAP2OSM__2OSM_HPP_