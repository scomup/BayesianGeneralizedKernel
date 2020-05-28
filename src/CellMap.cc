#include "CellMap.h"
#include "make_unique.h"
namespace BGK
{

	CellMap::CellMap(double resolution)
		: bits_(4),
		  resolution_(resolution)
	{
    	std::vector<std::vector<Cell*>> new_meta_cells_(1<<2*bits_);
		meta_cells_ = std::move(new_meta_cells_);
	}

	void CellMap::addCell(const Eigen::Vector3f &point)
	{
		tryGrow(point);
		auto idx = To2DIndex(point, bits_);
		int flat_idx = ToFlatIndex(idx, bits_);
		auto cell = new Cell(point.z());
		meta_cells_[ToFlatIndex(idx, bits_)].push_back(cell);
	}

	void CellMap::tryGrow(const Eigen::Vector3f &point)
	{
		auto idx = To2DIndex(point, bits_);
		while((idx < 0).any() || (idx >= (1<<bits_)).any())
		{
			grow();
			idx = To2DIndex(point, bits_);
		}
	}

  void CellMap::grow() {
    const int new_bits =
     bits_ + 1;
    
    CHECK_LE(new_bits, 16);
    std::vector<std::vector<Cell*>> new_meta_cells_(4 * meta_cells_.size());
	for (int y = 0; y != (1 << bits_); ++y)
	{
		for (int x = 0; x != (1 << bits_); ++x)
		{
			const Eigen::Array2i original_meta_index(x, y);
			const Eigen::Array2i new_meta_index = original_meta_index + (1 << (bits_ - 1));
			new_meta_cells_[ToFlatIndex(new_meta_index, new_bits)] =
				std::move(meta_cells_[ToFlatIndex(original_meta_index, bits_)]);
		}
	}

	meta_cells_ = std::move(new_meta_cells_);
    bits_ = new_bits;
  }


} // namespace BGK
