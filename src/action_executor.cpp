#include <memory>
#include <semaphore>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "dm_arm_interfaces/action/arm_movement.hpp"
using std::placeholders::_1;
std::counting_semaphore<1> action_signal(1); 

class ArmActionExecutor : public rclcpp::Node
{
  public:
    ArmActionExecutor()
    : Node("arm_action_executor")
    {
      // Reentrant callback group so another callback can start while first is executing
      callback_group_subscriber1_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

      callback_group_action1_ this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

      auto sub1_opt = rclcpp::SubscriptionOptions();
      sub1_opt.callback_group = callback_group_subscriber1_;
      thread_waiting = false

      //subscription to cmd topic
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "arm_cmd", 10, std::bind(&ArmActionExecutor::action_callback, this, _1), sub1_opt);

      //client to call arm movement action in driver package
      this->client_ptr_ = rclcpp_action::create_client<ArmMovement>(
      this,
      "arm_movement", callback_group_action1_);

      //publisher to update state of robot
      publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("arm_state", 10);

      
    }

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp_action::Client<ArmMovement>::SharedPtr client_ptr_;
    std::atomic<bool> thread_waiting;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_action1_;

    void action_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      using namespace std::placeholders;

      if(!(action_signal.try_acquire()){
        //signal that a thread is waiting to run an action
        this->thread_waiting = true;
        action_signal.acquire();
      }
      RCLCPP_INFO(this->get_logger(), "executing action");

      //get list of commands
      std::list<std::string> commands;
      std::string delimiter = ";;";
      std::string s = msg->data
      size_t pos = 0;
      std::string token;
      while ((pos = s.find(delimiter)) != std::string::npos) {
          token = s.substr(0, pos);
          commands.push_back(token)
          s.erase(0, pos + delimiter.length());
      }
      while(commands.size() > 0 && !(this->thread_waiting)){
        std::string current_command = commands.pop_front();
        std::string delimiter = "||";
        size_t pos = 0;
        pos = s.find(delimiter)

        //get the command type. pose, stop, or actuate end effector
        std::string type = current_command.substr(0, pos);
        s.erase(0, current_command + delimiter.length());
        std:string stop_type = "stop";
        
        //pose action
        if(type == "pose"){
          std::list<int> params;
          auto goal_msg = ArmMovement::Goal();
          delimiter = ","
          pos = 0;
          while ((pos = current_command.find(delimiter)) != std::string::npos) {
            std::string param = s.substr(0, pos);
            params.push_back(param)
            current_command.erase(0, pos + delimiter.length());
          }

          //populate pose
          goal_msg.goal_position.position.x = params[0];
          goal_msg.goal_position.position.y = params[1];
          goal_msg.goal_position.position.z = params[2];
          goal_msg.goal_position.orientation.x = params[3];
          goal_msg.goal_position.orientation.x = params[4];
          goal_msg.goal_position.orientation.x = params[5];
          goal_msg.goal_position.orientation.x = params[6];

          RCLCPP_INFO(this->get_logger(), "Sending goal");

          auto send_goal_options = rclcpp_action::Client<ArmMovement>::SendGoalOptions();
          send_goal_options.feedback_callback =
          std::bind(&ArmActionExecutor::feedback_callback, this, _1, _2);
            }

          auto gh_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);  
          auto gh = gh_future.get();
          auto res_future = this->client_ptr_->async_get_result(gh);
          auto status = res_future.wait_for(std::chrono::seconds(1);
          while(status != std::future_status::timeout && status != std::future_status::ready && !this->thread_waiting){
            status = res_future.wait_for(std::chrono::seconds(1);
          }
          if(status != std::future_status::timeout && status != std::future_status::ready && this->thread_waiting){
            //cancel goal because another action was requested
            RCLCPP_INFO(node->get_logger(), "canceling goal");
            
            auto cancel_result_future = this->client_ptr_->async_cancel_goal(gh);
            cancel_result_future.wait();

            auto result = cancel_result_future.get();
            if (result != rclcpp::FutureReturnCode::SUCCESS)
            {
              RCLCPP_ERROR(this->get_logger(), "failed to cancel goal");
              
            }
            RCLCPP_INFO(this->get_logger(), "goal is being canceled");
            
           else if (rclcpp::FutureReturnCode::SUCCESS != result) {
            RCLCPP_ERROR(this->get_logger(), "failed to get result");
            
          }
          }
          else {
            if (res_future.code != rclcpp_action::ResultCode::SUCCEEDED)
            {
              RCLCPP_ERROR(this->get_logger(), "Action did not succeed");
              return;
            }
          }
      }
      this->thread_waiting = false;
      action_signal.release();

      
    } 

    void feedback_callback(GoalHandleArmMovement::SharedPtr,
    const std::shared_ptr<const ArmMovement::Feedback> feedback) const {
      // report current robot state for tracking
      publisher_->publish(feedback->current_position);
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<ArmActionExecutor>();
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
