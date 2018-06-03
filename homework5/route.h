#ifndef ROUTE_H_
#define ROUTE_H_

#include "homework5/predecessors_and_successors.h"
#include "common/proto/map.pb.h"
#include "common/utils/file/file.h"
#include "common/proto/route.pb.h"
#include "glog/logging.h"
#include <string>

void find_route(interface::route::Route& rt, const interface::map::Map& map_data, const std::string& filepath);

#endif