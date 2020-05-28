#ifndef MULTILEVELGRID_H_
#define MULTILEVELGRID_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "CellMap.h"
#include "yaml-cpp/yaml.h"

namespace BGK
{

class MultiLevelGrid
{
    enum NodeType {PLANE, LINEAR, SPHERICAL};

  public:

    MultiLevelGrid(const double resultion);
    void setInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
  CellMap* cellmap_;
  protected:

  std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f> > 
    circleSearch(const Eigen::Vector3f& point);
  void training(const Eigen::Vector3f &point);



    const double grid_resolution_;
    const double radius_;

};
} // namespace BGK

#endif
