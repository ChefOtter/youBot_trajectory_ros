#include "youbot_draw.h"

namespace youbot_arm_drawing {

DrawingController::DrawingController(ros::NodeHandle& nh, std::string name)
{
	nh.param("youBotDriverCycleFrequencyInHz", lr, 200);
}
} // namespace youbot_arm_drawing
