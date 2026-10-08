#pragma once
#include <memory>
#include <string>
namespace rclcpp { class Node; }
namespace compatibility {
bool running();
int run(const std::string &entry, int argc, char **argv);
int telemetry(const std::string &entry, const std::shared_ptr<rclcpp::Node> &node);
int manual(const std::string &entry, const std::shared_ptr<rclcpp::Node> &node);
std::string backend_namespace(const std::shared_ptr<rclcpp::Node> &node);
class BackendLease {
public:
  explicit BackendLease(const std::shared_ptr<rclcpp::Node> &node);
  ~BackendLease();
  BackendLease(const BackendLease &) = delete;
  BackendLease &operator=(const BackendLease &) = delete;
private:
  int clients_ = -1;
};
int backend_host(const std::string &directory, const std::string &ns, const std::string &port);
}
