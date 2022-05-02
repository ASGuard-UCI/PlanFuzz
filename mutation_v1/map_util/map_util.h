#include <string>
#include "modules/tools/fuzz/planning/proto/on_lane_planning_msg.pb.h"
namespace apollo{
namespace hdmap{
  apollo::common::PointENU SLToXYZ(const std::string& lane_id, const double s, const double l);
  double GetHeading(const std::string& land_id, const double s);
  double GetHeading(const apollo::common::Point3D pos);
  void XYZToSL(const std::string& lane_id, const apollo::common::Point3D pos, double *s, double *l);
  double GetNearestL(const apollo::common::Point3D pos, double buffer_l);
  bool PointIsOnRoad(const apollo::common::Point3D pos);
}
}
