#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

/*
 * CSV UTM coordinates → /csv_path (nav_msgs/Path, frame: utm)
 *
 * utm_to_odometry 와 동일하게 /utm_datum 수신 시점의 UTM 좌표를 datum 으로
 * 래치한다. 두 노드가 같은 datum 을 공유하므로 /csv_path 는 별도 TF 없이
 * 바로 utm 프레임 경로로 사용할 수 있다.
 *
 * 변환 규칙 (utm_to_odometry 와 동일):
 *   local_x =  (utm_x - datum_x)
 *   local_y = -(utm_y - datum_y)   // CARLA 좌수계 보정
 */

class CsvToUtm : public rclcpp::Node
{
public:
    CsvToUtm() : Node("csv_to_utm")
    {
        this->declare_parameter<std::string>("csv_file_path", "");

        load_csv();

        // /utm_datum (transient_local): utm_to_odometry 가 먼저 시작했어도
        // 래치된 마지막 datum 을 즉시 수신한다 → 타이밍 경쟁 없음
        rclcpp::QoS datum_qos(rclcpp::KeepLast(1));
        datum_qos.reliable().transient_local();
        datum_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/utm_datum", datum_qos,
            std::bind(&CsvToUtm::datum_cb, this, std::placeholders::_1));

        rclcpp::QoS path_qos(rclcpp::KeepLast(1));
        path_qos.reliable().transient_local();
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/csv_path", path_qos);

        RCLCPP_INFO(this->get_logger(),
            "csv_to_utm: %zu waypoints loaded, waiting for /utm_datum...",
            utm_points_.size());
    }

private:
    std::vector<std::pair<double, double>> utm_points_;
    std::optional<double> datum_x_;
    std::optional<double> datum_y_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr datum_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    void load_csv()
    {
        const std::string path = this->get_parameter("csv_file_path").as_string();
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "CSV not found: %s", path.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;
            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }
            if (tokens.size() < 2) continue;
            try {
                utm_points_.emplace_back(std::stod(tokens[0]), std::stod(tokens[1]));
            } catch (const std::exception&) {
                // 헤더 행 또는 파싱 불가 행 무시
            }
        }
    }

    void datum_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (datum_x_.has_value()) return;  // datum 은 최초 1회만 래치

        datum_x_ = msg->point.x;
        datum_y_ = msg->point.y;
        RCLCPP_INFO(this->get_logger(),
            "Datum latched: easting=%.3f, northing=%.3f",
            datum_x_.value(), datum_y_.value());
        publish_path();
    }

    void publish_path()
    {
        // UTM 절대 좌표 → utm 프레임 상대 좌표 (CARLA Y 반전 포함)
        std::vector<std::pair<double, double>> local_pts;
        local_pts.reserve(utm_points_.size());
        for (const auto& [x, y] : utm_points_) {
            local_pts.emplace_back(x - datum_x_.value(), -(y - datum_y_.value()));
        }

        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "utm";

        const size_t n = local_pts.size();
        for (size_t i = 0; i < n; ++i) {
            const double mx = local_pts[i].first;
            const double my = local_pts[i].second;

            // 진행 방향 yaw: 다음 웨이포인트 방향으로 계산
            double yaw = 0.0;
            if (i < n - 1) {
                yaw = std::atan2(local_pts[i + 1].second - my,
                                 local_pts[i + 1].first  - mx);
            } else if (n >= 2) {
                // 마지막 점: 직전 구간과 동일한 yaw 유지
                yaw = std::atan2(local_pts[n - 1].second - local_pts[n - 2].second,
                                 local_pts[n - 1].first  - local_pts[n - 2].first);
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = mx;
            pose.pose.position.y = my;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            path.poses.push_back(pose);
        }

        path_pub_->publish(path);
        RCLCPP_INFO(this->get_logger(),
            "/csv_path published in utm frame: %zu poses", path.poses.size());
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CsvToUtm>());
    rclcpp::shutdown();
    return 0;
}
