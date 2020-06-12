#include <assert.h>
#include <chrono>

#include "GridmapKDE.h"
#include "common.h"

namespace BGK
{

  GridmapKDE::GridmapKDE(const float resultion)
      : Gridmap(resultion)
  {
    height_kernel_ = sparseKernel(height_resolution_, bandwidth_);
    circle_points_ = circleSearch(Eigen::Vector3f(0,0,0));
  }

  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
  GridmapKDE::circleSearch(const Eigen::Vector3f &point) const
  {
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> candidates;
    for (float x = point.x() - radius_; x < point.x() + radius_; x += grid_resolution_)
      for (float y = point.y() - radius_; y < point.y() + radius_; y += grid_resolution_)
      {

        float dist = std::sqrt((x - point.x()) * (x - point.x()) + (y - point.y()) * (y - point.y()));
        if (dist < radius_)
          candidates.emplace_back(x, y, point.z(), sparse(point(3), radius_));
      }
    return candidates;
  }

  void GridmapKDE::updateHeightPDF(Grid *const grid, const float height, const float weight) const
  {
    if (height < min_height_ || height > max_height_)
      return;

    if (grid->pdf_.size() == 0)
    {
      grid->pdf_.resize(height_n_);
      //std::cout<<"non zero;"<<grid->pdf_.nonZeros()<<std::endl;
      //grid->pdf_.Zero();
      //grid->dense_pdf_ = Eigen::ArrayXf::Zero(height_n_);
    }

    int idx = std::round((height - min_height_) / height_resolution_);

    assert(idx >= 0 && height_kernel_.size() + idx <= height_n_);

    //grid->pdf_.middleRows(idx, height_kernel_.size()) =
    //grid->pdf_.middleRows(idx, height_kernel_.size()) + (weight * height_kernel_);
    for (size_t i = 0; i < height_kernel_.size(); i++)
    {
      const float wk = weight * height_kernel_[i];
      if (wk != 0)
      {
        //grid->dense_pdf_[idx] += wk;
        //grid->mappdf_[idx] += wk;
        grid->pdf_.coeffRef(idx) += wk;
        //grid->triplet_list_.emplace_back(0, idx, wk);
      }
      idx++;
    }
    grid->weight_ += weight;
  }

  void GridmapKDE::updateCells(Grid *const grid) const
  {

    grid->marker = false;
    
    if (grid->weight_ < 1)
      return;

    Eigen::VectorXf pdf = grid->pdf_;
    pdf =  (-pdf.cwiseInverse()*10).array().exp();
    const float max = pdf.maxCoeff();

    //if (max < 1)
    //  return;

    for(auto c : grid->cells_){
      delete c;
    }
    grid->cells_.clear();

    //std::cout<<pdf.maxCoeff()<<std::endl;
    //float pre;
    //float cur = grid->pdf_.coeffRef(0, 0);
    //float nxt = grid->pdf_.coeffRef(1, 0);
    float last_cell_density = 0;
    bool flag = true;
    //float last_density = 0;
    //const float s = 1.02;
    std::vector<std::pair<int, float>> loc_max;

    
    for (int i = 1; i < height_n_-1; i++)
    {
      float pre = pdf(i - 1);
      float cur = pdf(i);
      float nxt = pdf(i + 1);

      if (cur > pre && cur > nxt)
      {
        loc_max.emplace_back(i, cur);
      }
    }
    for (auto l : loc_max)
    {
      const int i = l.first;
      const float cur_h = (height_resolution_ * i + (min_height_ - bandwidth_)) * height_resolution_ / height_resolution_;
      const float w = l.second;
      if (w < 0.5)
        continue;
      auto cell = new Cell(cur_h);
      grid->cells_.push_back(cell);
    }

    /*
    std::sort(loc_max.begin(), loc_max.end(),
              [](const std::pair<int, float> &a,
                 const std::pair<int, float> &b) {
                return a.first > b.first;
              });
    //
    std::vector<bool> marker(pdf.size(), false);
    //std::cout << loc_max.size() << std::endl;

    for (auto l : loc_max)
    {
      const int i = l.first;
      if (marker[i] == true)
        continue;
      marker[i] = true;
      const float w = l.second * 0.3;
      int lower = i - 1;
      int upper = i + 1;
      while (pdf(lower, 0) > w && lower > 0)
      {
        marker[lower] = true;
        lower--;
      }
      while (pdf(upper, 0) > w && upper < height_n_)
      {
        marker[upper] = true;
        upper++;
      }
      int idx;
      float cell_height = float(upper -  lower)*height_resolution_;
      //std::cout<<cell_height<<std::endl;
      //if(cell_height >  0.5)
      //  idx = upper;
      //else
        idx = i;
      const float cur_h = (height_resolution_ * idx + (min_height_ - bandwidth_)) * height_resolution_ / height_resolution_;
      auto cell = new Cell(cur_h);
      grid->cells_.push_back(cell);
    }*/
  }

  void GridmapKDE::update(const pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud)
  {
    std::cout << "Points:" << point_cloud->points.size() << std::endl;
    std::vector<Grid*> marked_grids;

    auto t0 = std::chrono::system_clock::now(); 
    for (size_t i = 0; i < point_cloud->points.size(); i++) //point_cloud->points.size()
    {
      Eigen::Vector3f point = point_cloud->points[i].getVector3fMap();
      //cellmap_->addCell(point);
      const float height = point.z();
      for (auto &p : circle_points_)
      {
        Eigen::Vector3f cur_point = p.head<3>() + point;
        Grid *gird = getGrid(cur_point);
        const float weight = p(3);
        
        updateHeightPDF(gird, height, weight);
        if (gird->marker == false)
        {
          marked_grids.push_back(gird);
          gird->marker = true;
        }
      }

      //if (i % 1000 == 0)
      //  std::cout << "KDE updating:" << float(i) / point_cloud->points.size() * 100. << "%            \r";
    }
    auto t1 = std::chrono::system_clock::now(); 


    for (auto &grid : marked_grids)
    {
      if (grid != nullptr)
      {
        updateCells(grid);
      }
    }
    auto t2 = std::chrono::system_clock::now();
    double elapsed10 = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    double elapsed21 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    std::cout  <<"update candidates("<<point_cloud->points.size() * circle_points_.size()<<"): "<< elapsed10 << " millisec" << std::endl;
    std::cout <<"update cells("<<marked_grids.size()<<"): "<<elapsed21 << " millisec" << std::endl; 
    std::cout << "----------------------\n";
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr GridmapKDE::getCloud() const
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (auto &grid : grids_)
    {
      if (grid == nullptr)
        continue;

      for (auto &cell : grid->cells_)
      {
        pcl::PointXYZI p;
        p.x = grid->loc_.x();
        p.y = grid->loc_.y();
        p.z = cell->height_;
        cloud->push_back(p);
      }
    }
    return cloud;
  };

} // namespace BGK
