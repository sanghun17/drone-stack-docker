// Integration check: PCL preprocessing must preserve rays for the real mapper.
#include <octomap_world/octomap_world.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

int main() {
  using World = volumetric_mapping::OctomapWorld;
  volumetric_mapping::OctomapParameters parameters;
  parameters.resolution = .25;
  parameters.sensor_max_range = 5.;
  parameters.treat_unknown_as_occupied = false;
  for (bool preserve_far : {false, true}) {
    World world(parameters);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    source->push_back(pcl::PointXYZ(0., 0., 10.));
    source->push_back(pcl::PointXYZ(1., 0., 2.));
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(source);
    filter.setLeafSize(.07, .07, .07);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(.1, preserve_far ? 1.e9 : 5.1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    filter.filter(*filtered);
    volumetric_mapping::Transformation identity;
    world.insertPointcloud(identity, filtered);
    world.insertPointcloud(identity, filtered);
    if (world.getCellStatusPoint(Eigen::Vector3d(1., 0., 2.)) != World::CellStatus::kOccupied) {
      std::cerr << "In-range obstacle was lost\n";
      return 3;
    }
    auto status = world.getCellStatusPoint(Eigen::Vector3d(0., 0., 4.));
    if (status != (preserve_far ? World::CellStatus::kFree : World::CellStatus::kUnknown)) {
      std::cerr << "Unexpected in-range clearing; preserve_far=" << preserve_far << '\n';
      return 1;
    }
    for (double range : {5.5, 10.}) {
      if (world.getCellStatusPoint(Eigen::Vector3d(0., 0., range)) != World::CellStatus::kUnknown) {
        std::cerr << "Mapper wrote beyond its 5 m sensor range\n";
        return 2;
      }
    }
  }
  std::cout << "Clipped returns lose free-space evidence; full returns clear only within 5 m.\n";
}
