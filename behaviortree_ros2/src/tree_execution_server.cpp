// Copyright 2024 Marq Rasmussen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright
// notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#include <thread>
#pragma warning(pop)
#else
#include <thread>
#endif

// auto-generated header, created by generate_parameter_library
#include "behaviortree_ros2/bt_executor_parameters.hpp"
#include "behaviortree_ros2/tree_execution_server.hpp"
#include "behaviortree_ros2/bt_utils.hpp"


namespace BT
{

struct TreeExecutionServer::Pimpl
{
  rclcpp_action::Server<ExecuteTree>::SharedPtr action_server;

  // TODO: give each goal it's own parameters?
  std::shared_ptr<bt_server::ParamListener> param_listener;
  bt_server::Params params;

  BT::BehaviorTreeFactory factory;
  bool factory_initialized_ = false;

  std::shared_ptr<BT::Groot2Publisher> groot_publisher;
  rclcpp::WallTimer<rclcpp::VoidCallbackType>::SharedPtr single_shot_timer;
};

TreeExecutionServer::TreeExecutionServer(const rclcpp::Node::SharedPtr& node)
  : p_(new Pimpl), node_(node)
{
  p_->param_listener = std::make_shared<bt_server::ParamListener>(node_);
  p_->params = p_->param_listener->get_params();

  // create the action server
  const auto action_name = p_->params.action_name;
  RCLCPP_INFO(node_->get_logger(), "Starting Action Server: %s", action_name.c_str());
  p_->action_server = rclcpp_action::create_server<ExecuteTree>(
      node_, action_name,
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const ExecuteTree::Goal> goal) {
        return handle_goal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleExecuteTree> goal_handle) {
        return handle_cancel(std::move(goal_handle));
      },
      [this](const std::shared_ptr<GoalHandleExecuteTree> goal_handle) {
        handle_accepted(std::move(goal_handle));
      });

  // we use a wall timer to run asynchronously executeRegistration();
  rclcpp::VoidCallbackType callback = [this]() {
    if(!p_->factory_initialized_)
    {
      executeRegistration();
    }
    // we must cancel the timer after the first execution
    p_->single_shot_timer->cancel();
  };

  p_->single_shot_timer =
      node_->create_wall_timer(std::chrono::milliseconds(1), callback);
}

TreeExecutionServer::~TreeExecutionServer()
{}

void TreeExecutionServer::executeRegistration()
{
  // Before executing check if we have new Behaviors or Subtrees to reload
  p_->factory.clearRegisteredBehaviorTrees();

  p_->params = p_->param_listener->get_params();
  // user defined method
  registerNodesIntoFactory(p_->factory);
  // load plugins from multiple directories
  RegisterPlugins(p_->params, p_->factory, node_);
  // load trees (XML) from multiple directories
  RegisterBehaviorTrees(p_->params, p_->factory, node_);

  p_->factory_initialized_ = true;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
TreeExecutionServer::get_node_base_interface()
{
  return node_->get_node_base_interface();
}

rclcpp::Node::SharedPtr TreeExecutionServer::node()
{
  return node_;
}

rclcpp_action::GoalResponse
TreeExecutionServer::handle_goal(const rclcpp_action::GoalUUID& /* uuid */,
                                 std::shared_ptr<const ExecuteTree::Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(), "Received goal request to execute Behavior Tree: %s",
              goal->target_tree.c_str());

  if(!onGoalReceived(goal->target_tree, goal->payload))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TreeExecutionServer::handle_cancel(
    const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
  if(!goal_handle->is_active())
  {
    RCLCPP_WARN(node_->get_logger(), "Rejecting request to cancel goal because the server is not "
                         "processing the goal.");
    return rclcpp_action::CancelResponse::REJECT;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TreeExecutionServer::handle_accepted(
    const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
{
  std::thread{ [=]() { execute(goal_handle); } }.detach();
}

void TreeExecutionServer::execute(
    const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
{
  GoalResources session;
  session.global_blackboard = BT::Blackboard::create();


  const auto goal = goal_handle->get_goal();
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  auto action_result = std::make_shared<ExecuteTree::Result>();

  // Before executing check if we have new Behaviors or Subtrees to reload
  if(p_->param_listener->is_old(p_->params))
  {
    executeRegistration();
  }

  // Loop until something happens with ROS or the node completes
  try
  {
    // This blackboard will be owned by "MainTree". It parent is p_->global_blackboard
    auto root_blackboard = BT::Blackboard::create(session.global_blackboard);

    session.tree = p_->factory.createTree(goal->target_tree, root_blackboard);
    session.tree_name = goal->target_tree;
    session.payload = goal->payload;

    // call user defined function after the tree has been created
    onTreeCreated(session);
    p_->groot_publisher.reset();
    p_->groot_publisher =
        std::make_shared<BT::Groot2Publisher>(session.tree, p_->params.groot2_port);

    // Loop until the tree is done or a cancel is requested
    const auto period =
        std::chrono::milliseconds(static_cast<int>(1000.0 / p_->params.tick_frequency));
    auto loop_deadline = std::chrono::steady_clock::now() + period;

    // operations to be done if the tree execution is aborted, either by
    // cancel requested or by onLoopAfterTick()
    auto stop_action = [this, &action_result, &session](BT::NodeStatus status,
                                              const std::string& message) {
      session.tree.haltTree();
      action_result->node_status = ConvertNodeStatus(status);
      // override the message value if the user defined function returns it
      if(auto msg = onTreeExecutionCompleted(status, true, session))
      {
        action_result->return_message = msg.value();
      }
      else
      {
        action_result->return_message = message;
      }
      RCLCPP_WARN(node_->get_logger(), action_result->return_message.c_str());
    };

    while(rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
      if(goal_handle->is_canceling())
      {
        stop_action(status, "Action Server canceling and halting Behavior Tree");
        goal_handle->canceled(action_result);
        return;
      }

      // tick the tree once and publish the action feedback
      status = session.tree.tickExactlyOnce();

      if(const auto res = onLoopAfterTick(status, session); res.has_value())
      {
        stop_action(res.value(), "Action Server aborted by onLoopAfterTick()");
        goal_handle->abort(action_result);
        return;
      }

      if(const auto res = onLoopFeedback(session); res.has_value())
      {
        auto feedback = std::make_shared<ExecuteTree::Feedback>();
        feedback->message = res.value();
        goal_handle->publish_feedback(feedback);
      }

      const auto now = std::chrono::steady_clock::now();
      if(now < loop_deadline)
      {
        session.tree.sleep(std::chrono::duration_cast<std::chrono::system_clock::duration>(
            loop_deadline - now));
      }
      loop_deadline += period;
    }
  }
  catch(const std::exception& ex)
  {
    action_result->return_message = std::string("Behavior Tree exception:") + ex.what();
    RCLCPP_ERROR(node_->get_logger(), action_result->return_message.c_str());
    goal_handle->abort(action_result);
    return;
  }

  // Call user defined onTreeExecutionCompleted function.
  // Override the message value if the user defined function returns it
  if(auto msg = onTreeExecutionCompleted(status, false, session))
  {
    action_result->return_message = msg.value();
  }
  else
  {
    action_result->return_message =
        std::string("Tree finished with status: ") + BT::toStr(status);
  }

  // set the node_status result to the action
  action_result->node_status = ConvertNodeStatus(status);

  // return success or aborted for the action result
  if(status == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), action_result->return_message.c_str());
    goal_handle->succeed(action_result);
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), action_result->return_message.c_str());
    goal_handle->abort(action_result);
  }
}

const std::string& TreeExecutionServer::treeName(GoalResources& session) const
{
  return session.tree_name;
}

const std::string& TreeExecutionServer::goalPayload(GoalResources& session) const
{
  return session.payload;
}

const BT::Tree& TreeExecutionServer::tree(GoalResources& session) const
{
  return session.tree;
}

BT::Blackboard::Ptr TreeExecutionServer::globalBlackboard(GoalResources& session)
{
  return session.global_blackboard;
}

BT::BehaviorTreeFactory& TreeExecutionServer::factory()
{
  return p_->factory;
}

}  // namespace BT
