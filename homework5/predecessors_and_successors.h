#ifndef PREDECESSORS_AND_SUCCESSORS_H
#define PREDECESSORS_AND_SUCCESSORS_H

#include <string>
#include "homework5/map/map_lib.h"
#include "common/proto/map.pb.h"
#include "common/utils/file/file.h"
#include "glog/logging.h"

void FindPreSuc(const interface::map::Map& map_data_, const std::string path);

#endif