#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>

// void mostrarBarraProgreso(double porcentaje)
// {
//   // Calcula el número de caracteres llenos y vacíos en la barra de progreso
//   int num_caracteres_llenos = static_cast<int>(porcentaje / 2);
//   int num_caracteres_vacios = 50 - num_caracteres_llenos;

//   // Construye la barra de progreso
//   std::string barra_llena(num_caracteres_llenos, '█');
//   std::string barra_vacia(num_caracteres_vacios, '░');

//   // Imprime la barra de progreso en la consola
//   std::cout << "Progreso trayectoria: [" << barra_llena << barra_vacia << "]\n";
// }

class LecturaArchivo : public rclcpp::Node
{
public:
  explicit LecturaArchivo(const rclcpp::NodeOptions& options) : Node("Lectura_del_archivo", options)
  {
    file_path_ = "./src/Universal_Robots_ROS2_Driver/trayectorias/points.csv";
    waypoints_buffer = waypoints_to_buffer(file_path_);
  }

private:
  std::string file_path_;
  std::vector<geometry_msgs::msg::Pose> waypoints_buffer;

  std::vector<geometry_msgs::msg::Pose> waypoints_to_buffer(const std::string& file_path)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    RCLCPP_INFO(this->get_logger(), "Reading waypoints from %s", file_path.c_str());

    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file %s", file_path.c_str());
      return waypoints;
    }

    std::string line;

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      geometry_msgs::msg::Pose msg;

      char delimiter;
      // if (ss >> msg.position.x >> delimiter >> msg.position.y >> delimiter >> msg.position.z >> delimiter >>
      //     msg.orientation.w >> delimiter >> msg.orientation.x >> delimiter >> msg.orientation.y >> delimiter >>
      //     msg.orientation.z) {
      if (ss >> msg.position.x >> delimiter >> msg.position.y >> delimiter >> msg.position.z) {
        msg.orientation.w = 1.0;
        waypoints.push_back(msg);
      } else {
        RCLCPP_WARN(this->get_logger(), "Skipping invalid line in file %s: %s", file_path.c_str(), line.c_str());
      }
    }

    RCLCPP_INFO(this->get_logger(), "Lectura completada.");
    file.close();
    return waypoints;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto const logger = rclcpp::get_logger("Proyecto de equipo BBB");
  auto node = std::make_shared<LecturaArchivo>(rclcpp::NodeOptions());
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}