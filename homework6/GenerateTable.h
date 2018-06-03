#pragma once

#include "homework6/simulation/vehicle_agent.h"
#include "common/proto/agent_status.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/file/file.h"
#include "homework6/proto/simulation_config.pb.h"
#include "homework6/simulation/dynamic_lib/lib_vehicle_status_model_solver.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

namespace ycdfwzy {

interface::agent::AgentStatus GenerateInitialStatus(double, double, double);

void Generator();

}; //namespace ycdfwzy