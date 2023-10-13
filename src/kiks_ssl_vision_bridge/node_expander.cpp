#include "kiks_ssl_vision_bridge/node_expander.hpp"

namespace kiks::ssl_vision_bridge
{

NodeExpander::NodeExpander(rclcpp::Node & node)
: node_(node),
#ifdef KIKS_ROS_DISTRO_DASHING
  params_setter_(node_.set_on_parameters_set_callback(
      [this](const auto & params) {return this->set_params(params);}))
#else
  params_setter_(
    node_.add_on_set_parameters_callback(
      [this](const auto & params) {return this->set_params(params);}))
#endif
{
}

inline NodeExpander::SetParamsResultMsg NodeExpander::set_params(
  const std::vector<rclcpp::Parameter> & params)
{
  SetParamsResultMsg set_params_result_msg;
  set_params_result_msg.successful = true;

  for (const auto & param : params) {
    const auto & name = param.get_name();
    const auto [begin, end] = param_setter_map_.equal_range(name);
    bool this_param_successful = true;
    for (auto itr = begin; itr != end; ++itr) {
      const auto error_handle =
        [this, &set_params_result_msg, &this_param_successful](const std::string & reason) {
          RCLCPP_ERROR(node_.get_logger(), "set_param : %s", reason.c_str());
          if (set_params_result_msg.successful) {
            set_params_result_msg.successful = false;
          } else {
            set_params_result_msg.reason += " & ";
          }
          set_params_result_msg.reason += reason;
          this_param_successful = false;
        };
      try {
        itr->second(param);
      } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
        error_handle(name + " : ros error : " + e.what());
      } catch (const std::exception & e) {
        error_handle(name + " : std error :" + e.what());
      } catch (...) {
        error_handle(name + " : unknown error");
      }
    }
    if (this_param_successful) {
      RCLCPP_INFO(
        node_.get_logger(), "succes to set param : %s : %s", name.c_str(),
        param.value_to_string().c_str());
    }
  }
  return set_params_result_msg;
}

}  // namespace kiks::ssl_vision_bridge
