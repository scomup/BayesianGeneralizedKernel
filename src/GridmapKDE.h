#ifndef MULTILEVELGRID_H_
#define MULTILEVELGRID_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <unordered_map>
#include <boost/functional/hash.hpp>

#include "Gridmap.h"
#include "yaml-cpp/yaml.h"

namespace BGK
{



class GridmapKDE : public Gridmap
{

  public:

    GridmapKDE(const float resultion);
    void update(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr getCloud() const;
  protected:

  void updateHeightPDF( Grid* const grid, const float height, const float weight) const;
  void updateCells( Grid* const grid) const;

  std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > 
    circleSearch(const Eigen::Vector3f& point) const;
  void training(const Eigen::Vector3f &point);

  std::vector<float> height_kernel_;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> circle_points_;


};
} // namespace BGK

#endif
