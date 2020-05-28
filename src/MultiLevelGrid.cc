
#include "MultiLevelGrid.h"

namespace BGK
{

  MultiLevelGrid::MultiLevelGrid(const double resultion)
      : grid_resolution_(resultion),
        radius_(0.5),
        cellmap_(new CellMap(resultion)) {}

  std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f> > 
   MultiLevelGrid::circleSearch(const Eigen::Vector3f& point)
  {
    std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f> > candidates;
    for (float x = point.x() -radius_; x < point.x() + radius_; x += grid_resolution_)
      for (float y = point.y() - radius_; y < point.y() + radius_; y += grid_resolution_)
      {
        Eigen::Vector3f new_point(x,y, point.z());
        float dist = (new_point - point).norm();
        if (dist < radius_)
          candidates.push_back(new_point); 
      }
      return candidates;
  }

  void MultiLevelGrid::setInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
  {
    for (auto p : point_cloud->points)
    {
      Eigen::Vector3f point = p.getVector3fMap();
      auto candidates = circleSearch(point);
      //cellmap_->addCell(point);
      for (auto point : candidates)
      {
        training(point);
      }
    }
  }

  void MultiLevelGrid::training(const Eigen::Vector3f &point)
  {
    cellmap_->tryGrow(point);
    auto &col = cellmap_->getCol(point);
    for (auto &cell : col)
    {
      float diff = std::abs(cell->height_ - point.z());
      //std::cout<<diff<<std::endl;
      //if(diff < r)
    }
    cellmap_->addCell(point);

  }

} // namespace BGK
