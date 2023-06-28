#include "hadmap2osm/2osm.hpp"


HADMap2osm::HADMap2osm()
: Node("hadmap2osm")
{
    // topic_name = "/map/vector_map";
    topic_name = "/output/lanelet2_map";
    sub_map_ = this->create_subscription<HADMapBin>(
        topic_name, rclcpp::QoS{1}.transient_local(),
        std::bind(&HADMap2osm::mapCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "[parellel_Map Based Prediction]: Start HADMap2osm node");
}

void HADMap2osm::mapCallback(const HADMapBin::ConstSharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "[Map Based Prediction]: Start loading lanelet");
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    write(tempfile("map.osm"),*lanelet_map_ptr_);

    HADMap2osm::fromBinMsg(*msg, lanelet_map_ptr_);
    RCLCPP_INFO(get_logger(), "[Map Based Prediction]: Map is loaded");
}

void HADMap2osm::fromBinMsg(const autoware_auto_mapping_msgs::msg::HADMapBin & msg, lanelet::LaneletMapPtr map)
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
    printf("id_counter: %ld\n", id_counter);
    // lanelet::utils::registerId(id_counter);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HADMap2osm>());
    rclcpp::shutdown();
    return 0;
}