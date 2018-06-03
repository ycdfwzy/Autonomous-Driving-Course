#include "homework6/agents/straight_agent.h"

namespace ycdfwzy {

void StraightAgent::Initialize(const interface::agent::AgentStatus& /* agent_status */){
	last_velocity = 0;
	pid_.velocity_threshold = 5.0;
	pid_.Kp = 50;
	pid_.Ki = 0.1;
	pid_.Kd = 20;
}

interface::control::ControlCommand StraightAgent::RunOneIteration(
      const interface::agent::AgentStatus& agent_status){
	interface::control::ControlCommand command;
	// Vehicle's current position reaches the destination
    if (CalcDistance(agent_status.vehicle_status().position(),
                     agent_status.route_status().destination()) < 4.0) {
      position_reached_destination_ = true;
  	  command.set_brake_ratio(1.0);
  	  return command;
    }
    // Vehicle's current velocity reaches 5 m/s
    if (CalcVelocity(agent_status.vehicle_status().velocity()) > 5) {
      velocity_reached_threshold_ = true;

      double e = (5.0 - CalcVelocity(agent_status.vehicle_status().velocity()));
      double d = CalcVelocity(agent_status.vehicle_status().velocity()) - last_velocity;
      double Kp = 5.1;
      double Kd = 5.1;
      double pd = Kp*e + Kd*d;
      if (pd > 1) pd = 1;
      if (pd < -1) pd = -1;
      if (pd > 0)
      	command.set_throttle_ratio(pd);
      else command.set_brake_ratio(-pd);

      return command;
    }
    command.set_throttle_ratio(0.3);
	last_velocity = CalcVelocity(agent_status.vehicle_status().velocity());
/*
    if (position_reached_destination_) {
      // Set maximum brake ratio to stop the vehicle
      command.set_brake_ratio(1.0);
    } else {
      if (!velocity_reached_threshold_) {
        // Set throttle ratio to accelerate
        //command.set_throttle_ratio(0.3);
        command.set_throttle_ratio(0.3);
      }
    }

    PID_Control(agent_status, command);
*/
    return command;
}

void StraightAgent::PID_Control(const interface::agent::AgentStatus& agent_status,
									interface::control::ControlCommand command){
	/*
	if (CalcDistance(agent_status.vehicle_status().position(),
                     agent_status.route_status().destination()) < 3.0) {
      position_reached_destination_ = true;
    }

    if (CalcVelocity(agent_status.vehicle_status().velocity()) > 5) {
      velocity_reached_threshold_ = true;
    }


    if (position_reached_destination_){
    	//pid_.
    } else
    if (!velocity_reached_threshold_){

    }
    */
}

}