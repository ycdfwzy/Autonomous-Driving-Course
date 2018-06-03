#include "GenerateTable.h"
#include <fstream>
#include <iostream>

namespace ycdfwzy {

using interface::agent::AgentStatus;
using interface::agent::VehicleStatus;
using interface::agent::SimulationStatus;
using interface::control::ControlCommand;
using std::cout;
using std::endl;

AgentStatus GenerateInitialStatus(double vx, double vy, double vz) {
  AgentStatus agent_status;
  agent_status.mutable_vehicle_status()->mutable_position()->set_x(0);
  agent_status.mutable_vehicle_status()->mutable_position()->set_y(0);
  agent_status.mutable_vehicle_status()->mutable_position()->set_z(0);

  agent_status.mutable_vehicle_status()->mutable_orientation()->set_x(0);
  agent_status.mutable_vehicle_status()->mutable_orientation()->set_y(0);
  agent_status.mutable_vehicle_status()->mutable_orientation()->set_z(-1);
  agent_status.mutable_vehicle_status()->mutable_orientation()->set_w(0);

  agent_status.mutable_vehicle_status()->mutable_velocity()->set_x(vx);
  agent_status.mutable_vehicle_status()->mutable_velocity()->set_y(vy);
  agent_status.mutable_vehicle_status()->mutable_velocity()->set_z(vz);

  agent_status.mutable_route_status()->mutable_destination()->set_x(10);
  agent_status.mutable_route_status()->mutable_destination()->set_y(10);
  agent_status.mutable_route_status()->mutable_destination()->set_z(0);

  agent_status.mutable_simulation_status()->set_is_alive(true);
  return agent_status;
}

//DEFINE_string(route_file_path, "", "Path of route file");

void Generator(){
  //interface::homework6::SimulationConfig simulation_config_;
  //CHECK(file::ReadTextFileToProto(FLAGS_route_file_path, &simulation_config_));

  std::ofstream fout("/home/ycdfwzy/Desktop/a.txt");
  for (int vx = 0; vx <= 300; vx++)
    for (int theottle_brake = -10; theottle_brake <= 10; ++theottle_brake){
        fout << "vx = " << vx/10.0 << endl;
        fout << "theottle_brake = " << theottle_brake/10.0 << endl;
        AgentStatus agent_status;
        agent_status.CopyFrom(GenerateInitialStatus(-vx/10., 0.0, 0.0));

        interface::vehicle::VehicleParams vehicle_params_;
        CHECK(file::ReadTextFileToProto(utils::path::GetVehicleParamsPath(), &vehicle_params_));

        std::unique_ptr<vehicle_status_model::VehicleStatusModelSolver> solver =
            vehicle_status_model::CreateVehicleStatusModelSolver(vehicle_params_);

        solver->Initialize(0, agent_status.vehicle_status());

        fout << "Before" << endl;
        fout << agent_status.vehicle_status().velocity().x() << " " <<
                agent_status.vehicle_status().velocity().y() << " " <<
                agent_status.vehicle_status().velocity().z() << endl;
        fout << agent_status.vehicle_status().acceleration_vcs().x() << " " <<
                agent_status.vehicle_status().acceleration_vcs().y() << " " <<
                agent_status.vehicle_status().acceleration_vcs().z() << endl;


        ControlCommand command;

        if (theottle_brake < 0)
          command.set_brake_ratio(-(double)theottle_brake/10.0);
        else
          command.set_throttle_ratio((double)theottle_brake/10.0);
        //command.set_throttle_ratio(0.3);
        for (int i = 1; i <= 100; ++i)
          agent_status.mutable_vehicle_status()->CopyFrom(
            solver->UpdateVehicleStatus(0.01*i, command) );

        fout << "After" << endl;
        fout << agent_status.vehicle_status().velocity().x() << " " <<
                agent_status.vehicle_status().velocity().y() << " " <<
                agent_status.vehicle_status().velocity().z() << endl;
        fout << agent_status.vehicle_status().acceleration_vcs().x() << " " <<
                agent_status.vehicle_status().acceleration_vcs().y() << " " <<
                agent_status.vehicle_status().acceleration_vcs().z() << endl<< endl;
      /*
        cout << agent_status.vehicle_status().position().x() << endl;
        cout << agent_status.vehicle_status().position().y() << endl;
        cout << agent_status.vehicle_status().position().z() << endl;
      */
  }
  fout.close();
}

};	// namespace ycdfwzy