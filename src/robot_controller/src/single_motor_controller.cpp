#include "robot_controller/single_motor_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#define logger (rclcpp::get_logger("SingleMotorController"))

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

namespace robot_controller
{

using lifecycle_msgs::msg::State;


SingleMotorController::SingleMotorController()
{
}

SingleMotorController::~SingleMotorController()
{
}

controller_interface::CallbackReturn SingleMotorController::on_init()
{
	// get parameters
	// try
	// {
	// 	param_listener_ = std::make_shared<single_motor_controller::ParamListener>(get_node());
	// 	params_ = param_listener_->get_params();
	// 	if(params_.joint_name.empty())
	// 	{
	// 		RCLCPP_ERROR(logger, "Provide param   joint_name   ");
	// 		return controller_interface::CallbackReturn::ERROR;
	// 	}
	// }
	// catch(const std::exception& e)
	// {
	// 	RCLCPP_INFO(logger, "SingleMotorController on_init error: %s", e.what());
	// 	return controller_interface::CallbackReturn::ERROR;
	// }

	RCLCPP_INFO(logger, "SingleMotorController on_init");

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SingleMotorController::command_interface_configuration() const
{
	std::vector<std::string> conf_names;
	// std::string joint_name = params_.joint_name;
	// conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
	// RCLCPP_INFO(logger, "ntdat command_interface_configuration = %s", (joint_name + "/" + hardware_interface::HW_IF_VELOCITY).c_str());

	return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};

}

controller_interface::InterfaceConfiguration SingleMotorController::state_interface_configuration() const
{
	std::vector<std::string> conf_names;
	// std::string joint_name = params_.joint_name;
	// conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
	// RCLCPP_INFO(logger, "ntdat state_interface_configuration = %s", (joint_name + "/" + hardware_interface::HW_IF_POSITION).c_str());

	return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type SingleMotorController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
	if(get_state().id() == State::PRIMARY_STATE_INACTIVE)
	{
		if(!is_halted_)
		{
			halt();
			is_halted_ = true;
		}
		return controller_interface::return_type::OK;
	}

	// get velocity command
	bool last_command = previous_vel_commands_.back();
	// RCLCPP_INFO(logger, "ntdat last_command = %d", last_command);

	// calculate velocity output
	std_msgs::msg::Float64 vel_out_msg;
	double temp_vel = 0.0;
	if(last_command)
	{
		temp_vel = 3.0;
	}
	else
	{
		temp_vel = 0.0;
	}
	vel_out_msg.data = temp_vel;
	vel_out_ = temp_vel;

	// publish velocity output
	// RCLCPP_INFO(logger, "ntdat pub vel = %f", temp_vel);
	out_publisher_->publish(vel_out_msg);

	command_interfaces_.back().set_value(vel_out_);

	return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn SingleMotorController::on_configure(const rclcpp_lifecycle::State &)
{

	// auto logger = get_node()->get_logger();


	// check params
	// command_interfaces_.resize(1);
	// command_interfaces_.back().set_value(0.0);

	// check reset
	if(!reset())
	{
		return controller_interface::CallbackReturn::ERROR;
	}

	out_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());

	// // subscribe to command topic
	const bool empty_bool = false;
	// // received

	previous_vel_commands_.emplace(empty_bool);
	
	in_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
		DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
		[this](const std::shared_ptr<std_msgs::msg::Bool> msg) -> void
		{
			// received
			bool rec_command = msg->data;
			previous_vel_commands_.emplace(rec_command);
			
			RCLCPP_INFO(logger, "ntdat rec_command = %d", rec_command);
			RCLCPP_INFO(this->get_node()->get_logger(), "Subscriber and publisher are now active.");

		}
	);

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SingleMotorController::on_activate(const rclcpp_lifecycle::State &)
{
	is_halted_ = false;
	RCLCPP_INFO(get_node()->get_logger(), "Subscriber and publisher are now active.");

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SingleMotorController::on_deactivate(const rclcpp_lifecycle::State &)
{
	if(!is_halted_)
	{
		halt();
		is_halted_ = true;
	}
	RCLCPP_INFO(get_node()->get_logger(), "Subscriber and publisher are now inactive.");

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SingleMotorController::on_cleanup(const rclcpp_lifecycle::State &)
{
	if(!reset())
	{
		return controller_interface::CallbackReturn::FAILURE;
	}
	RCLCPP_INFO(get_node()->get_logger(), "Subscriber and publisher are now cleaned up.");

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SingleMotorController::on_error(const rclcpp_lifecycle::State &)
{
	if(!reset())
	{
		return controller_interface::CallbackReturn::FAILURE;
	}
	RCLCPP_INFO(get_node()->get_logger(), "Subscriber and publisher are now in error.");

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SingleMotorController::on_shutdown(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_node()->get_logger(), "Subscriber and publisher are now shut down.");

	return controller_interface::CallbackReturn::SUCCESS;
}

void SingleMotorController::halt()
{
	// stop motor
	// stop publisher
	// stop subscriber

	vel_out_ = 0.0;
	is_halted_ = true;

}

bool SingleMotorController::reset()
{
	// reset motor
	// reset subscriber
	// reset publisher

	std::queue<bool> empty;
	std::swap(previous_vel_commands_, empty);
	is_halted_ = false;

	return true;
}

} // namespace robot_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(robot_controller::SingleMotorController, controller_interface::ControllerInterface)
