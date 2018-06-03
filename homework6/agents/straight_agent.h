#pragma once

#include "homework6/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "homework6/simulation/vehicle_agent_factory.h"


namespace ycdfwzy{

class StraightAgent : public simulation::VehicleAgent{
public:
	explicit StraightAgent(const std::string& name) : VehicleAgent(name) {}

	void Initialize(const interface::agent::AgentStatus& /* agent_status */);
	interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status);

	void PID_Control(const interface::agent::AgentStatus& agent_status,
					interface::control::ControlCommand);

	struct PID{
		double acceleration;
		double err;
		double err_last;
		double Kp, Ki, Kd;
		double velocity_threshold;
		double velocity_goal;
	};

private:
  double CalcDistance(const interface::geometry::Vector3d& position,
                      const interface::geometry::Point3D& destination) {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    ;
    return std::sqrt(sqr_sum);
  }

  double CalcVelocity(const interface::geometry::Vector3d& velocity) {
    double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
    ;
    return std::sqrt(sqr_sum);
  }

  double CalcAcceleration(const interface::geometry::Vector3d& acceleration){
  	double sqr_sum = math::Sqr(acceleration.x()) + math::Sqr(acceleration.y());
    ;
    return std::sqrt(sqr_sum);
  }

  bool position_reached_destination_ = false;
  bool velocity_reached_threshold_ = false;
  double last_velocity;
  PID pid_;
};

}  // namespace ycdfwzy