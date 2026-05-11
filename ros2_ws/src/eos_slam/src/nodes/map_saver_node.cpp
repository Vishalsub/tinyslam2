#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <filesystem>
#include <cmath>

namespace eos_slam {

class MapSaverNode : public rclcpp::Node {
public:
  MapSaverNode() : Node("eos_map_saver") {
    declare_parameter("save_dir",      std::string("/tmp/eos_slam_maps"));
    declare_parameter("map_name",      std::string("eos_map"));
    declare_parameter("auto_save_s",   0.0);   // 0 = disabled

    save_dir_  = get_parameter("save_dir").as_string();
    map_name_  = get_parameter("map_name").as_string();
    double auto_s = get_parameter("auto_save_s").as_double();

    std::filesystem::create_directories(save_dir_);

    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/eos/map/grid",
        rclcpp::QoS(1).transient_local().reliable(),
        [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
          latest_grid_ = msg;
        });

    save_srv_ = create_service<std_srvs::srv::Trigger>(
        "/eos/map/save",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
          if (!latest_grid_) {
            resp->success = false;
            resp->message = "No map received yet.";
            return;
          }
          auto path = saveMap(*latest_grid_);
          resp->success = true;
          resp->message = "Saved to " + path;
          RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
        });

    if (auto_s > 0.0) {
      auto_timer_ = create_wall_timer(
          std::chrono::duration<double>(auto_s),
          [this]() {
            if (latest_grid_) {
              saveMap(*latest_grid_);
            }
          });
      RCLCPP_INFO(get_logger(), "Auto-save every %.0f s → %s", auto_s, save_dir_.c_str());
    }

    RCLCPP_INFO(get_logger(),
        "Map saver ready. Call /eos/map/save to save, or set auto_save_s.");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr auto_timer_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_grid_;

  std::string save_dir_;
  std::string map_name_;

  // Returns the stem path (without extension) of the saved files.
  std::string saveMap(const nav_msgs::msg::OccupancyGrid& grid) {
    std::string stem = save_dir_ + "/" + map_name_;
    savePgm(grid, stem + ".pgm");
    saveYaml(grid, stem + ".yaml", stem + ".pgm");
    return stem;
  }

  void savePgm(const nav_msgs::msg::OccupancyGrid& grid, const std::string& path) {
    std::ofstream f(path, std::ios::binary);
    const int W = static_cast<int>(grid.info.width);
    const int H = static_cast<int>(grid.info.height);
    f << "P5\n" << W << " " << H << "\n255\n";
    // ROS occupancy convention: 0=free(white), 100=occupied(black), -1=unknown(grey).
    // PGM: 0=black, 255=white.
    for (int row = H - 1; row >= 0; --row) {           // flip Y for image convention
      for (int col = 0; col < W; ++col) {
        int8_t val = grid.data[static_cast<size_t>(row * W + col)];
        uint8_t pixel;
        if (val == 0)         pixel = 254;              // free → white
        else if (val == 100)  pixel = 0;               // occupied → black
        else                  pixel = 205;             // unknown → grey
        f.write(reinterpret_cast<const char*>(&pixel), 1);
      }
    }
  }

  void saveYaml(const nav_msgs::msg::OccupancyGrid& grid,
                const std::string& yaml_path,
                const std::string& pgm_path) {
    std::ofstream f(yaml_path);
    f << "image: "       << pgm_path                              << "\n"
      << "resolution: "  << grid.info.resolution                  << "\n"
      << "origin: ["
          << grid.info.origin.position.x << ", "
          << grid.info.origin.position.y << ", 0.0]\n"
      << "negate: 0\n"
      << "occupied_thresh: 0.65\n"
      << "free_thresh: 0.196\n";
  }
};

}  // namespace eos_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eos_slam::MapSaverNode>());
  rclcpp::shutdown();
}
