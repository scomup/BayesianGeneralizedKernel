#include "Gridmap.h"
#include "make_unique.h"
namespace BGK
{

	Gridmap::Gridmap(double grid_resolution)
		: bits_(4),
		  grid_resolution_(grid_resolution),
		  height_resolution_(0.02),
		  radius_(0.5),
		  bandwidth_(0.3),
		  min_height_(-2),
		  max_height_(2),
		  height_n_(std::floor((max_height_ - min_height_ + 2 * bandwidth_) / height_resolution_))
	{
		std::vector<Grid *> grids(1 << 2 * bits_, nullptr);
		grids_ = std::move(grids);
	}

	Grid* Gridmap::getGrid(const Eigen::Vector3f &point)
	{
		auto idx = To2DIndex(point, bits_);
		if ((idx < 0).any() || (idx >= (1 << bits_)).any())
			return nullptr;
		return grids_[ToFlatIndex(idx, bits_)];
	}

	Grid* Gridmap::addGrid(const Eigen::Vector3f &point)
	{
		//If the new point is outside the current map, expand the map.
		auto idx = To2DIndex(point, bits_);
		while((idx < 0).any() || (idx >= (1<<bits_)).any())
		{
			grow();
			idx = To2DIndex(point, bits_);
		}
		const int flat_idx = ToFlatIndex(idx, bits_);

		//If gird already exists, return it
		if(grids_[flat_idx] != nullptr)
			return grids_[flat_idx];

		//create a grid for the new point.
		float x = std::round(point.x()/grid_resolution_)*grid_resolution_;
		float y = std::round(point.y()/grid_resolution_)*grid_resolution_;
		auto grid = new Grid(x, y);
		grids_[flat_idx] = grid;
		return grid;
	}



  void Gridmap::grow() {
    const int new_bits =
     bits_ + 1;
    
    CHECK_LE(new_bits, 16);
    std::vector<Grid*> new_grid(4 * grids_.size(),nullptr);
	for (int y = 0; y != (1 << bits_); ++y)
	{
		for (int x = 0; x != (1 << bits_); ++x)
		{
			const Eigen::Array2i original_meta_index(x, y);
			const Eigen::Array2i new_meta_index = original_meta_index + (1 << (bits_ - 1));
			new_grid[ToFlatIndex(new_meta_index, new_bits)] =
				std::move(grids_[ToFlatIndex(original_meta_index, bits_)]);
		}
	}

	grids_ = std::move(new_grid);
    bits_ = new_bits;
  }
	


} // namespace BGK
