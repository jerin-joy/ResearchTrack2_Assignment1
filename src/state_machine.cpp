#include <inttypes.h>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1/srv/random_position.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/command.hpp"

using namespace std::chrono_literals;
using CommandService = rt2_assignment1::srv::Command;
using PositionService = rt2_assignment1::srv::Position;
using RandomPositionService = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{

    class StateMachine : public rclcpp::Node
    {
    public:
        StateMachine(const rclcpp::NodeOptions & options)
            : Node("state_machine_node", options)
        {
            start = false;
            goal_reached = true;

            service_ = this->create_service<CommandService>(
                "/user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));

            client_rp_ = this->create_client<RandomPositionService>("/position_server");
            while (!client_rp_->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
            }

            client_p_ = this->create_client<PositionService>("/go_to_point");
            while (!client_p_->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
            }

            req_rp = std::make_shared<RandomPositionService::Request>();
            res_rp = std::make_shared<RandomPositionService::Response>();
            req_p = std::make_shared<PositionService::Request>();

            req_rp->x_max = 5.0;
            req_rp->x_min = -5.0;
            req_rp->y_max = 5.0;
            req_rp->y_min = -5.0;

            timer_ = this->create_wall_timer(
                1000ms, std::bind(&StateMachine::update_goal, this));
        }

    private:
        void user_interface(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<CommandService::Request> req,
            const std::shared_ptr<CommandService::Response> res)
        {
            (void)request_header;
            if (req->command == "start")
            {
                start = true;
            }
            else
            {
                start = false;
            }
            res->ok = start;
            RCLCPP_INFO(this->get_logger(), "Received request %s", req->command.c_str());
        }

        void update_goal()
        {

            if (!goal_reached)
                return;

            if (!start)
                return;

            call_goToPoint();
        }

        void call_goToPoint()
        {

            call_randomPosition();

            goal_reached = false;
            req_p->x = res_rp->x;
            req_p->y = res_rp->y;
            req_p->theta = res_rp->theta;

            RCLCPP_INFO(this->get_logger(), "Going to the position: x= %f y= %f theta= %f",
                        req_p->x, req_p->y, req_p->theta);

            auto point_reached_callback =
                [this](rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture future)
            {(void)future; goal_reached = true;
               RCLCPP_INFO(this->get_logger(), "Goal reached!"); };
            auto future_result = client_p_->async_send_request(req_p, point_reached_callback);
        }

        void call_randomPosition()
        {

            auto res_rp_received_callback =
                [this](rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture future)
            { res_rp = future.get(); };
            auto future_result = client_rp_->async_send_request(req_rp, res_rp_received_callback);
        }

        rclcpp::Service<CommandService>::SharedPtr service_;
        rclcpp::Client<RandomPositionService>::SharedPtr client_rp_;
        rclcpp::Client<PositionService>::SharedPtr client_p_;

        std::shared_ptr<RandomPositionService::Request> req_rp;
        std::shared_ptr<RandomPositionService::Response> res_rp;
        std::shared_ptr<PositionService::Request> req_p;

        rclcpp::TimerBase::SharedPtr timer_;

        bool start;
        bool goal_reached;
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)
