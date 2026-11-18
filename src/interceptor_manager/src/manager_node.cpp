// Manager node: the interceptor drone's decision-maker.
//
// It reads drone state from MAVROS and target estimates from the Tracker, and
// sends one absolute position setpoint (local ENU) to MAVROS every tick.
// Subscriptions only store the latest message; every decision happens in
// tick(), which runs at 20 Hz.
//
// State machine (manager_node_decisions.md explains the reasoning):
//
//   WAIT_FOR_READY --(armed + OFFBOARD)--> TAKEOFF --(at altitude)--> SWEEP
//   SWEEP  --(target seen)--> FOLLOW --(target lost)--> SEARCH
//   SEARCH --(target seen again)--> FOLLOW
//   SEARCH --(search_duration with no target)--> SWEEP
//   INTERCEPT: placeholder for Phase 3; nothing enters it yet.
//
// From any state, losing the MAVROS link, armed, or OFFBOARD (for example,
// the safety pilot takes over) returns to WAIT_FOR_READY. That state holds the current
// position, so re-entering OFFBOARD never pulls the drone toward an old target.
//
// The manager never arms the drone or changes its flight mode. The arm script
// in interceptor_bringup does that.

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "interceptor_manager/manager_math.hpp"
#include "interceptor_msgs/msg/target_data.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

using namespace std::chrono_literals;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Vector3;
using interceptor_msgs::msg::TargetData;
using mavros_msgs::msg::PositionTarget;
using mavros_msgs::msg::State;
using std_srvs::srv::SetBool;

// PX4 leaves OFFBOARD if setpoints arrive slower than about 2 Hz.
// 20 Hz leaves a wide margin.
constexpr auto kTickPeriod = 50ms;

enum class FlightState { WAIT_FOR_READY, TAKEOFF, SWEEP, FOLLOW, SEARCH, INTERCEPT };

const char * to_string(FlightState state)
{
  switch (state) {
    case FlightState::WAIT_FOR_READY: return "WAIT_FOR_READY";
    case FlightState::TAKEOFF: return "TAKEOFF";
    case FlightState::SWEEP: return "SWEEP";
    case FlightState::FOLLOW: return "FOLLOW";
    case FlightState::SEARCH: return "SEARCH";
    case FlightState::INTERCEPT: return "INTERCEPT";
  }
  return "UNKNOWN";
}

class ManagerNode : public rclcpp::Node
{
public:
  ManagerNode()
  : Node("manager_node")
  {
    // Read once at startup. config/manager.yaml describes each one.
    const double sweep_center_x = declare_parameter("sweep_center_x", 0.0);
    const double sweep_center_y = declare_parameter("sweep_center_y", 0.0);
    const double sweep_radius = declare_parameter("sweep_radius", 152.4);
    const double lane_spacing = declare_parameter("lane_spacing", 30.0);
    sweep_altitude_ = declare_parameter("sweep_altitude", 20.0);
    waypoint_tolerance_ = declare_parameter("waypoint_tolerance", 3.0);
    standoff_distance_ = declare_parameter("standoff_distance", 10.0);
    min_altitude_ = declare_parameter("min_altitude", 5.0);
    target_lost_timeout_ = declare_parameter("target_lost_timeout", 1.0);
    search_duration_ = declare_parameter("search_duration", 5.0);

    sweep_waypoints_ = interceptor_manager::make_sweep_waypoints(
      sweep_center_x, sweep_center_y, sweep_radius, lane_spacing, sweep_altitude_);
    RCLCPP_INFO(get_logger(), "Sweep has %zu waypoints", sweep_waypoints_.size());

    mavros_state_sub_ = create_subscription<State>(
      "/mavros/state", 10, [this](State::SharedPtr msg) {mavros_state_ = *msg;});
    // MAVROS publishes pose as best-effort; a default (reliable) subscriber
    // would receive nothing.
    pose_sub_ = create_subscription<PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](PoseStamped::SharedPtr msg) {pose_ = msg;});
    target_sub_ = create_subscription<TargetData>(
      "/target_data", 10, [this](TargetData::SharedPtr msg) {target_ = msg;});

    setpoint_pub_ = create_publisher<PositionTarget>("/mavros/setpoint_raw/local", 10);

    // Stub: nothing calls it yet. It's on/off rather than a toggle so it can
    // never get out of sync with the camera's real state.
    camera_toggle_client_ = create_client<SetBool>("/camera_toggle");
    // TODO: /camera_pos client, once its service type is defined.
    // TODO: MAVROS command service clients, when a later phase needs them.

    // A wall timer, not a ROS timer, so the setpoint stream PX4 depends on
    // keeps flowing even if the sim clock stalls or isn't published.
    timer_ = create_wall_timer(kTickPeriod, [this]() {tick();});
  }

private:
  void tick()
  {
    if (!pose_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "No pose from MAVROS yet; not publishing setpoints");
      return;
    }

    if (state_ != FlightState::WAIT_FOR_READY && !ready_to_fly()) {
      change_state(
        FlightState::WAIT_FOR_READY,
        !mavros_state_.connected ? "MAVROS lost connection to PX4" :
        !mavros_state_.armed ? "disarmed" :
        "left OFFBOARD (mode " + mavros_state_.mode + ")");
    }

    switch (state_) {
      case FlightState::WAIT_FOR_READY:
        // Hold wherever the drone is now, so switching into OFFBOARD causes no jump.
        publish_setpoint(pose_->pose.position, current_yaw());
        if (ready_to_fly()) {
          change_state(FlightState::TAKEOFF, "armed and in OFFBOARD");
        }
        break;

      case FlightState::TAKEOFF: {
        // Climb straight up from where we started, keeping that heading.
        Point climb_to = entry_point_;
        climb_to.z = sweep_altitude_;
        publish_setpoint(climb_to, entry_yaw_);
        if (std::abs(pose_->pose.position.z - sweep_altitude_) < waypoint_tolerance_) {
          change_state(FlightState::SWEEP, "reached sweep altitude");
        }
        break;
      }

      case FlightState::SWEEP: {
        const Point & waypoint = sweep_waypoints_[sweep_index_];
        const Point & here = pose_->pose.position;
        // Face the direction of travel; the camera points forward.
        publish_setpoint(waypoint, interceptor_manager::yaw_toward(here, waypoint));
        if (interceptor_manager::distance(here, waypoint) < waypoint_tolerance_) {
          // After the last waypoint, start the sweep over.
          sweep_index_ = (sweep_index_ + 1) % sweep_waypoints_.size();
          RCLCPP_INFO(
            get_logger(), "Waypoint reached; heading to %zu/%zu",
            sweep_index_ + 1, sweep_waypoints_.size());
        }
        if (target_fresh()) {
          change_state(FlightState::FOLLOW, "target seen");
        }
        break;
      }

      case FlightState::FOLLOW: {
        const Point & target = target_->position;
        const Point & here = pose_->pose.position;
        // Face the target so the camera keeps it in view.
        publish_setpoint(
          interceptor_manager::standoff_point(target, here, standoff_distance_, min_altitude_),
          interceptor_manager::yaw_toward(here, target),
          interceptor_manager::standoff_velocity(target_->velocity, target.z, min_altitude_));
        if (!target_fresh()) {
          change_state(FlightState::SEARCH, "target lost");
        }
        break;
      }

      case FlightState::SEARCH: {
        // Spin in place where the target was lost; we're already near where it
        // was last seen. One full turn over search_duration. A moving yaw
        // setpoint keeps every state on the same publish helper, with no
        // separate yaw-rate mode.
        const double elapsed = (now() - entry_time_).seconds();
        const double yaw = std::remainder(
          entry_yaw_ + elapsed * 2.0 * M_PI / search_duration_, 2.0 * M_PI);
        publish_setpoint(entry_point_, yaw);
        if (target_fresh()) {
          change_state(FlightState::FOLLOW, "target seen again");
        } else if (elapsed >= search_duration_) {
          change_state(FlightState::SWEEP, "search finished, no target");
        }
        break;
      }

      case FlightState::INTERCEPT:
        // Phase 3 placeholder: nothing enters this state yet.
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "INTERCEPT is not implemented; holding position");
        publish_setpoint(entry_point_, entry_yaw_);
        break;
    }
  }

  // One check for both entering and leaving WAIT_FOR_READY, so they can't disagree.
  bool ready_to_fly() const
  {
    return mavros_state_.connected && mavros_state_.armed && mavros_state_.mode == "OFFBOARD";
  }

  // The Tracker updates last_seen only on real detections, so a stale
  // last_seen means the target is lost. A crashed Tracker looks the same.
  bool target_fresh() const
  {
    if (!target_) {
      return false;
    }
    const rclcpp::Time last_seen(target_->last_seen, get_clock()->get_clock_type());
    return (now() - last_seen).seconds() < target_lost_timeout_;
  }

  void change_state(FlightState next, const std::string & reason)
  {
    RCLCPP_INFO(
      get_logger(), "%s -> %s: %s", to_string(state_), to_string(next), reason.c_str());
    state_ = next;
    // TAKEOFF, SEARCH, and INTERCEPT all work from where the drone was, and
    // which way it faced, when the state began. Recording it here means no
    // state has to remember to do it.
    entry_point_ = pose_->pose.position;
    entry_yaw_ = current_yaw();
    entry_time_ = now();
  }

  // Values are ENU even though coordinate_frame says FRAME_LOCAL_NED. That's a
  // known MAVROS naming quirk: MAVROS takes ENU here and converts it for PX4.
  void publish_setpoint(
    const Point & position, double yaw, const std::optional<Vector3> & velocity = std::nullopt)
  {
    PositionTarget msg;
    msg.header.stamp = now();
    msg.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY |
      PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW_RATE;
    msg.position = position;
    msg.yaw = static_cast<float>(yaw);
    if (velocity) {
      msg.velocity = *velocity;  // feedforward on top of PX4's position control
    } else {
      msg.type_mask |= PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY |
        PositionTarget::IGNORE_VZ;
    }
    setpoint_pub_->publish(msg);
  }

  double current_yaw() const
  {
    const auto & q = pose_->pose.orientation;
    return tf2::getYaw(tf2::Quaternion(q.x, q.y, q.z, q.w));
  }

  // Parameters used after startup
  double sweep_altitude_;
  double waypoint_tolerance_;
  double standoff_distance_;
  double min_altitude_;
  double target_lost_timeout_;
  double search_duration_;

  // Latest inputs
  State mavros_state_;
  PoseStamped::SharedPtr pose_;
  TargetData::SharedPtr target_;

  FlightState state_ = FlightState::WAIT_FOR_READY;

  // Pose and time when the current state began; set by change_state().
  Point entry_point_;
  double entry_yaw_ = 0.0;
  rclcpp::Time entry_time_;

  // Never reset, so after a chase or a pilot takeover the drone flies straight
  // to the waypoint it was heading to.
  std::vector<Point> sweep_waypoints_;
  size_t sweep_index_ = 0;

  rclcpp::Subscription<State>::SharedPtr mavros_state_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<TargetData>::SharedPtr target_sub_;
  rclcpp::Publisher<PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Client<SetBool>::SharedPtr camera_toggle_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManagerNode>());
  rclcpp::shutdown();
  return 0;
}
