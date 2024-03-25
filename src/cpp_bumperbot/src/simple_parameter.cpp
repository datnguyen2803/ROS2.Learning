#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>

#include <rcl_interfaces/msg/set_parameters_result.hpp>



using std::placeholders::_1;

class SimpleParameter : public rclcpp::Node
{
public:
	SimpleParameter() : Node("simple_parameter")
	{
		declare_parameter<int>("my_parameter", 0);
		declare_parameter<std::string>("my_string_param", "Mikey");
		
		param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::parameter_callback, this, std::placeholders::_1));

	}

private:
	OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

	rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters)
	{
		rcl_interfaces::msg::SetParametersResult result;
		for(const auto &param : parameters)
		{
			if(param.get_name() == "my_parameter" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
			{
				RCLCPP_INFO_STREAM(get_logger(), "my_parameter changed: " << param.as_int());
				result.successful = true;
			}
			if(param.get_name() == "my_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
			{
				RCLCPP_INFO_STREAM(get_logger(), "my_string_param changed: " << param.as_string());
				result.successful = true;
			}

		}
		return result;
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SimpleParameter>();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}