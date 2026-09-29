#include "cxx/navigation.hh"
#include "cxx/util/node_mixin.hh"
#include "cxx/util/service.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rmcs::navigation::details {

struct Navigation::Impl : rmcs::navigation::NodeMixin {

    using GoalAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<GoalAction>;
    using GoalResult = GoalHandle::WrappedResult;
    using GoalClient = rclcpp_action::Client<GoalAction>;
    using GoalSubscription = rclcpp::Subscription<geometry_msgs::msg::PoseStamped>;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
    using Pose2D = std::tuple<double, double, double>;

    using TfBuffer = tf2_ros::Buffer;
    using TfListener = tf2_ros::TransformListener;
    using Twist = geometry_msgs::msg::Twist;

    static constexpr auto kPositionEpsilon = 1e-2;
    static constexpr auto kNavigateToPoseActionName = "/navigate_to_pose";
    static constexpr auto kWorldFrame = "world";
    static constexpr auto kBaseFrame = "base_link";
    static constexpr auto kMoveBaseGoalTopic = "/move_base_simple/goal";
    static constexpr auto kGoalPoseTopic = "/goal_pose";
    static constexpr auto kInitialPoseTopic = "/initialpose";
    static constexpr auto kRelocalizeTriggerFifo = "/tmp/rmcs-localization/relocalize/trigger";
    static constexpr auto kServerNotReadyWarnInterval = std::chrono::seconds{5};

    struct Target final {
        double x = std::numeric_limits<double>::quiet_NaN();
        double y = std::numeric_limits<double>::quiet_NaN();

        auto operator==(const Target& rhs) const noexcept -> bool {
            return std::abs(x - rhs.x) <= Impl::kPositionEpsilon
                && std::abs(y - rhs.y) <= Impl::kPositionEpsilon;
        }
    };

    rclcpp::Node& node;

    // TF Query
    std::shared_ptr<TfBuffer> tf_buffer = std::make_shared<TfBuffer>(node.get_clock());
    std::shared_ptr<TfListener> tf_listener =
        std::make_shared<TfListener>(*tf_buffer, &node, false);

    // Navigation Action
    std::shared_ptr<GoalClient> client =
        rclcpp_action::create_client<GoalAction>(&node, kNavigateToPoseActionName);
    GoalAction::Goal navigation_goal_message = [] {
        auto message = GoalAction::Goal{};
        message.pose.header.frame_id = kWorldFrame;
        message.pose.pose.orientation.w = 1.0;
        message.pose.pose.orientation.x = 0.0;
        message.pose.pose.orientation.y = 0.0;
        message.pose.pose.orientation.z = 0.0;
        return message;
    }();

    // Topic Forwarding (Optional)
    std::shared_ptr<GoalSubscription> move_base_goal_subscription;
    std::shared_ptr<GoalSubscription> goal_pose_subscription;
    bool topic_forward_enabled = false;

    // Clicked Pose Capture (Optional)
    // 订阅回调运行在 rclcpp executor 线程，consume_pose 运行在 component update 线程，
    // 因此缓存与开关均由 capture_mutex 保护。
    std::mutex capture_mutex;
    std::shared_ptr<rclcpp::Subscription<PoseWithCovarianceStamped>> initial_pose_subscription;
    std::shared_ptr<GoalSubscription> captured_goal_pose_subscription;
    bool pose_capture_enabled = false;
    std::optional<Pose2D> initial_pose_cache;
    std::optional<Pose2D> goal_pose_cache;

    // Localization Trigger (FIFO)
    // 触发：向 /tmp/rmcs-localization/relocalize/trigger 写入 "red"/"blue"
    // 结果：独立线程监听 /tmp/rmcs-navigation/relocalize/result 并打日志
    std::thread service_thread;
    std::atomic<bool> stop_service{false};

    // Goal Runtime State
    std::shared_ptr<GoalHandle> current_goal_handle;
    std::optional<Target> active_goal;

    // cmd_vel Subscription
    std::shared_ptr<rclcpp::Subscription<Twist>> cmd_vel_subscription;
    Eigen::Vector2d latest_cmd_vel = Eigen::Vector2d::Zero();
    std::chrono::steady_clock::time_point latest_cmd_vel_time;

    std::uint64_t latest_request_id = 0;
    std::chrono::steady_clock::time_point last_server_not_ready_warn_time{};

    auto has_same_goal(const Target& goal) const -> bool {
        return active_goal.has_value() && *active_goal == goal;
    }

    auto ensure_server_ready() -> bool {
        if (client->action_server_is_ready())
            return true;

        auto now = std::chrono::steady_clock::now();
        if (now - last_server_not_ready_warn_time >= kServerNotReadyWarnInterval) {
            warn("{} action server is not available", kNavigateToPoseActionName);
            last_server_not_ready_warn_time = now;
        }
        return false;
    }

    auto update_target_message(const Target& goal) -> void {
        navigation_goal_message.pose.header.stamp = node.now();
        navigation_goal_message.pose.pose.position.x = goal.x;
        navigation_goal_message.pose.pose.position.y = goal.y;
    }

    auto log_result(const Target& goal, rclcpp_action::ResultCode code) const -> void {
        switch (code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            info("navigation goal reached: x={}, y={}", goal.x, goal.y);
            return;
        case rclcpp_action::ResultCode::ABORTED:
            warn("navigation goal aborted: x={}, y={}", goal.x, goal.y);
            return;
        case rclcpp_action::ResultCode::CANCELED:
            info("navigation goal canceled: x={}, y={}", goal.x, goal.y);
            return;
        default:
            warn("navigation goal finished with unknown status: x={}, y={}", goal.x, goal.y);
            return;
        }
    }

    auto send_target(const Target& goal, std::uint64_t request_id) -> void {
        update_target_message(goal);

        auto options = GoalClient::SendGoalOptions{};
        options.goal_response_callback = //
            [request_id, this, goal](const std::shared_ptr<GoalHandle>& handle) {
                if (!handle) {
                    if (request_id == latest_request_id)
                        active_goal.reset();

                    warn("navigate_to_pose goal rejected: x={}, y={}", goal.x, goal.y);
                    return;
                }

                auto should_cancel = request_id != latest_request_id;
                if (!should_cancel)
                    current_goal_handle = handle;

                if (should_cancel) {
                    client->async_cancel_goal(handle);
                    return;
                }
            };
        options.result_callback = [this, goal, request_id](const GoalResult& result) {
            if (request_id != latest_request_id)
                return;

            active_goal.reset();
            current_goal_handle.reset();
            log_result(goal, result.code);
        };

        client->async_send_goal(navigation_goal_message, options);
    }

    static auto is_world_frame(const std::string& frame_id) -> bool {
        return frame_id.empty() || frame_id == kWorldFrame;
    }

    static auto yaw_from_quaternion(const geometry_msgs::msg::Quaternion& rotation) -> double {
        const auto sin_yaw = 2.0 * ((rotation.w * rotation.z) + (rotation.x * rotation.y));
        const auto cos_yaw = 1.0 - (2.0 * ((rotation.y * rotation.y) + (rotation.z * rotation.z)));
        return std::atan2(sin_yaw, cos_yaw);
    }

    /// 将任意坐标系下的点击位姿转换到 world 坐标系。
    ///
    /// Foxglove 的 2D pose 工具会把点发布在 3D 面板的显示坐标系下（常见为 base_link），
    /// 因此非 world 的输入需要用 TF 变换到 world 后才能作为导航目标。
    auto resolve_world_pose(const PoseStamped& pose) -> std::optional<Pose2D> {
        const auto& frame_id = pose.header.frame_id;
        if (is_world_frame(frame_id)) {
            return Pose2D{
                pose.pose.position.x,
                pose.pose.position.y,
                yaw_from_quaternion(pose.pose.orientation),
            };
        }

        try {
            const auto transform =
                tf_buffer->lookupTransform(kWorldFrame, frame_id, tf2::TimePointZero);
            const auto& translation = transform.transform.translation;
            const auto& rotation = transform.transform.rotation;
            const auto& position = pose.pose.position;

            const auto tx = 2.0 * (rotation.y * position.z - rotation.z * position.y);
            const auto ty = 2.0 * (rotation.z * position.x - rotation.x * position.z);
            const auto tz = 2.0 * (rotation.x * position.y - rotation.y * position.x);

            const auto x = position.x + rotation.w * tx + (rotation.y * tz - rotation.z * ty);
            const auto y = position.y + rotation.w * ty + (rotation.z * tx - rotation.x * tz);

            return Pose2D{
                x + translation.x,
                y + translation.y,
                yaw_from_quaternion(rotation) + yaw_from_quaternion(pose.pose.orientation),
            };
        } catch (const tf2::TransformException& exception) {
            warn(
                "failed to transform pose from [{}] to [{}]: {}", frame_id, kWorldFrame,
                exception.what());
            return std::nullopt;
        }
    }

    auto make_topic_forward_subscription(std::string_view topic) {
        return node.create_subscription<PoseStamped>(
            std::string{topic}, 10,
            [this, topic = std::string{topic}](const std::unique_ptr<PoseStamped>& message) {
                if (!message)
                    return;

                auto& position = message->pose.position;
                send_target(position.x, position.y);
                info("forward {} -> ({:.1}, {:.1})", topic, position.x, position.y);
            });
    }

public:
    explicit Impl(rclcpp::Node& node)
        : node{node} {
        auto cmd_vel_topic = node.get_parameter_or<std::string>("command_vel_name", "/cmd_vel");
        cmd_vel_subscription = node.create_subscription<Twist>(
            cmd_vel_topic, 10, [this](const std::unique_ptr<Twist>& msg) {
                latest_cmd_vel = {msg->linear.x, msg->linear.y};
                latest_cmd_vel_time = std::chrono::steady_clock::now();
            });

        service_thread = std::thread{[this] {
            auto service = util::make_service<"relocalize">(
                util::make_action<"result">([this](std::string_view data) {
                    if (data.starts_with("SUCCESS"))
                        info("relocalize {}", data);
                    else
                        warn("relocalize {}", data);
                }));
            info("relocalize result fifo at /tmp/rmcs-navigation/relocalize/");
            while (!stop_service.load(std::memory_order::relaxed)) {
                service.spin_once();
                std::this_thread::sleep_for(std::chrono::milliseconds{100});
            }
        }};
    }

    ~Impl() {
        stop_service.store(true, std::memory_order::relaxed);
        if (service_thread.joinable())
            service_thread.join();

        auto current_handle = std::exchange(current_goal_handle, std::shared_ptr<GoalHandle>{});
        active_goal.reset();
        ++latest_request_id;

        if (current_handle)
            client->async_cancel_goal(current_handle);
    }

    auto get_logger() const -> rclcpp::Logger { return node.get_logger(); }

    auto send_target(double x, double y) -> void {
        auto goal = Target{.x = x, .y = y};
        if (has_same_goal(goal))
            return;

        if (!ensure_server_ready())
            return;

        if (has_same_goal(goal))
            return;

        active_goal = goal;
        auto request_id = ++latest_request_id;
        auto cancel_handle = std::exchange(current_goal_handle, std::shared_ptr<GoalHandle>{});

        if (cancel_handle)
            client->async_cancel_goal(cancel_handle);

        send_target(goal, request_id);
    }

    auto cancel_target() -> void {
        active_goal.reset();
        ++latest_request_id;

        auto handle = std::exchange(current_goal_handle, std::shared_ptr<GoalHandle>{});
        if (handle)
            client->async_cancel_goal(handle);
    }

    auto relocalize(rmcs_msgs::RobotColor color) const -> void {
        if (color == rmcs_msgs::RobotColor::UNKNOWN) {
            warn("relocalize ignored: robot color is unknown");
            return;
        }

        const auto mode = (color == rmcs_msgs::RobotColor::BLUE) ? std::string_view{"blue"}
                                                                 : std::string_view{"red"};

        const auto fd = ::open(kRelocalizeTriggerFifo, O_WRONLY | O_NONBLOCK);
        if (fd < 0) {
            warn("relocalize ignored: trigger fifo {} is not available", kRelocalizeTriggerFifo);
            return;
        }

        const auto written = ::write(fd, mode.data(), mode.size());
        ::close(fd);
        if (written != static_cast<ssize_t>(mode.size())) {
            warn("relocalize failed: cannot write to trigger fifo {}", kRelocalizeTriggerFifo);
            return;
        }
        info("relocalize triggered: mode={}", mode);
    }

    auto switch_topic_forward(bool enable) -> void {
        if (topic_forward_enabled == enable)
            return;

        topic_forward_enabled = enable;
        if (topic_forward_enabled) {
            move_base_goal_subscription = make_topic_forward_subscription(kMoveBaseGoalTopic);
            goal_pose_subscription = make_topic_forward_subscription(kGoalPoseTopic);
            info("goal topic forwarding enabled");
            return;
        }

        move_base_goal_subscription.reset();
        goal_pose_subscription.reset();
        info("goal topic forwarding disabled");
    }

    auto switch_pose_capture(bool enable) -> void {
        const std::lock_guard lock{capture_mutex};
        if (pose_capture_enabled == enable)
            return;

        pose_capture_enabled = enable;
        if (!pose_capture_enabled) {
            initial_pose_subscription.reset();
            captured_goal_pose_subscription.reset();
            initial_pose_cache.reset();
            goal_pose_cache.reset();
            info("pose capture disabled");
            return;
        }

        initial_pose_cache.reset();
        goal_pose_cache.reset();
        initial_pose_subscription = node.create_subscription<PoseWithCovarianceStamped>(
            kInitialPoseTopic, 10,
            [this](const std::unique_ptr<PoseWithCovarianceStamped>& message) {
                if (!message)
                    return;

                auto pose = PoseStamped{};
                pose.header = message->header;
                pose.pose = message->pose.pose;

                const auto world_pose = resolve_world_pose(pose);
                if (!world_pose)
                    return;

                {
                    const std::lock_guard cache_lock{capture_mutex};
                    initial_pose_cache = world_pose;
                }
                info(
                    "captured initial pose -> ({:.2}, {:.2})", std::get<0>(*world_pose),
                    std::get<1>(*world_pose));
            });
        captured_goal_pose_subscription = node.create_subscription<PoseStamped>(
            kGoalPoseTopic, 10, [this](const std::unique_ptr<PoseStamped>& message) {
                if (!message)
                    return;

                const auto world_pose = resolve_world_pose(*message);
                if (!world_pose)
                    return;

                {
                    const std::lock_guard cache_lock{capture_mutex};
                    goal_pose_cache = world_pose;
                }
                info(
                    "captured goal pose -> ({:.2}, {:.2})", std::get<0>(*world_pose),
                    std::get<1>(*world_pose));
            });
        info("pose capture enabled");
    }

    auto consume_pose(PoseSource source) -> std::optional<Pose2D> {
        const std::lock_guard lock{capture_mutex};
        auto& cache = (source == PoseSource::INITIAL) ? initial_pose_cache : goal_pose_cache;
        auto result = cache;
        cache.reset();
        return result;
    }

    auto check_position() const -> std::tuple<double, double, double> {
        try {
            auto transform =
                tf_buffer->lookupTransform(kWorldFrame, kBaseFrame, tf2::TimePointZero);
            auto& translation = transform.transform.translation;
            auto& rotation = transform.transform.rotation;

            auto sin_yaw = 2.0 * ((rotation.w * rotation.z) + (rotation.x * rotation.y));
            auto cos_yaw = 1.0 - (2.0 * ((rotation.y * rotation.y) + (rotation.z * rotation.z)));
            auto yaw = std::atan2(sin_yaw, cos_yaw);

            return {translation.x, translation.y, yaw};
        } catch (const tf2::TransformException& exception) {
            constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
            return {kNan, kNan, kNan};
        }
    }
};

Navigation::Navigation(rclcpp::Node& node) noexcept
    : pimpl{std::make_unique<Impl>(node)} {}

Navigation::~Navigation() noexcept = default;

auto Navigation::send_target(double x, double y) -> void { pimpl->send_target(x, y); }

auto Navigation::cancel_target() -> void { pimpl->cancel_target(); }

auto Navigation::relocalize(rmcs_msgs::RobotColor color) -> void { pimpl->relocalize(color); }

auto Navigation::switch_topic_forward(bool enable) -> void { pimpl->switch_topic_forward(enable); }

auto Navigation::switch_pose_capture(bool enable) -> void { pimpl->switch_pose_capture(enable); }

auto Navigation::consume_pose(PoseSource source) -> std::optional<std::tuple<double, double, double>> {
    return pimpl->consume_pose(source);
}

auto Navigation::check_position() const -> std::tuple<double, double, double> {
    return pimpl->check_position();
}

auto Navigation::current_command() const -> Command {
    return {.speed = pimpl->latest_cmd_vel, .timestamp = pimpl->latest_cmd_vel_time};
}

} // namespace rmcs::navigation::details
