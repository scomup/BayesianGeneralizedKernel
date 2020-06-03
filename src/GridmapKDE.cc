#include <assert.h>

#include "GridmapKDE.h"
#include "common.h"

namespace BGK
{

/*
struct idx_hash
{
public:

  std::size_t operator()(const Eigen::Array2i &idx) const
  {
    size_t seed = 0;
    boost::hash_combine(seed, idx.x());
    boost::hash_combine(seed, idx.y());
    return seed;
  }
};

struct idx_equal {
    bool operator()(const Eigen::Array2i& a, const Eigen::Array2i& b) const
    {
        return a.x() == b.x() && a.y() == b.y();
    }
};
*/
GridmapKDE::GridmapKDE(const float resultion)
    : Gridmap(resultion)
{
  height_kernel_ = sparseKernel(height_resolution_, bandwidth_);
}


  std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > 
   GridmapKDE::circleSearch(const Eigen::Vector3f& point) const
  {
    std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > candidates;
    for (float x = point.x() -radius_; x < point.x() + radius_; x += grid_resolution_)
      for (float y = point.y() - radius_; y < point.y() + radius_; y += grid_resolution_)
      {
        
        float dist = std::sqrt( (x-point.x())*(x-point.x()) + (y-point.y())*(y-point.y()));
        if (dist < radius_)
          candidates.emplace_back(x, y, point.z(),dist); 
      }
      return candidates;
  }

  void GridmapKDE::updateHeightPDF(Grid* const grid, const float height, const float weight) const
  {
    if(height < min_height_ || height > max_height_)
      return;

    if(grid->pdf_.size() == 0)
      grid->pdf_.resize(height_n_,1);

    //int idx = std::round( (height - (min_height_ - bandwidth_))/height_resolution_);
    int idx = std::round( (height - min_height_ )/height_resolution_);
    for(size_t i = 0; i < height_kernel_.size();i++)
    {
      const float wk = weight * height_kernel_[i];
      grid->pdf_.coeffRef(idx, 0) +=  wk;
      idx++;
    }
    grid->weight_ += weight;
  }

  void GridmapKDE::updateCells(Grid *const grid) const
  {
    if(grid->weight_<5)
      return;
    const float sum = grid->pdf_.sum();

    for (int i = 1; i < height_n_ - 1; i++)
    {
      const float pre = grid->pdf_.coeffRef(i - 1, 0)/sum / height_resolution_;
      const float cur = grid->pdf_.coeffRef(i, 0)/sum / height_resolution_;
      const float nxt = grid->pdf_.coeffRef(i + 1, 0)/sum / height_resolution_;
      if (cur > 0.1 && cur > pre && cur > nxt)
      {
        const float cur_h = (height_resolution_ * i + (min_height_ - bandwidth_)) * height_resolution_ / height_resolution_;
        //std::cout<<"new!"<<std::endl;
        auto cell = new Cell(cur_h);
        grid->cells_.push_back(cell);
      }
    }
  }

  void GridmapKDE::setInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
  {
    std::cout<<"Points:"<<point_cloud->points.size()<<std::endl;
    for (size_t i = 0; i < point_cloud->points.size(); i++)
    {
      Eigen::Vector3f point = point_cloud->points[i].getVector3fMap();
      auto candidates = circleSearch(point);
      //cellmap_->addCell(point);
      
      for (auto &point : candidates)
      {
        Grid* gird = addGrid(point.head<3>());
        const float height = point.z();
        const float weight = sparse(point(3),radius_);
        updateHeightPDF(gird, height, weight);
      }

      if(i%1000==0)
        std::cout<<"KDE updating:"<<float(i)/point_cloud->points.size()*100.<<"%            \r";
    }

    for (auto &grid : grids_)
    {
      if(grid != nullptr){
        updateCells(grid);
      }
    }
    std::cout<<"\n"; 
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr GridmapKDE::getCloud() const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto &grid : grids_)
    {
      if (grid == nullptr)
        continue;

      for (auto &cell : grid->cells_)
      {
        pcl::PointXYZ p;
        p.x  = grid->loc_.x();
        p.y  = grid->loc_.y();
        p.z  = cell->height_;
        cloud->push_back(p);
      }
    }
    return cloud;
  };

} // namespace BGK
