
#ifndef CELLMAP_H_
#define CELLMAP_H_

#include "Cell.h"

#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>
#include "glog/logging.h"

namespace BGK
{

	class CellMap
	{
	public:
		CellMap(double resolution);
		void addCell(const Eigen::Vector3f &point);
		void tryGrow(const Eigen::Vector3f &point);


		inline int ToFlatIndex(const Eigen::Array2i &index, const int bits)
		{
			DCHECK((index >= 0).all() && (index < (1 << bits)).all()) << index;
			return (index.y() << bits) + index.x();
		}

		inline Eigen::Array2i To2DIndex(const Eigen::Vector3f &point, const int bits) const
		{
			Eigen::Vector3f index = point.array() / resolution_;
			return Eigen::Array2i(std::lround(index.x() + (1 << (bits - 1))),
								  std::lround(index.y() + (1 << (bits - 1))));
		}

		inline Eigen::Array2i To2DIndex(const Eigen::Vector3f &point) const
		{
			Eigen::Vector3f index = point.array() / resolution_;
			return Eigen::Array2i(std::lround(index.x() + (1 << (bits_ - 1))),
								  std::lround(index.y() + (1 << (bits_ - 1))));
		}

		inline int ToFlatIndex(const Eigen::Vector3f &point, const int bits)
		{
			const Eigen::Array2i idx = To2DIndex(point, bits);
			return ToFlatIndex(idx, bits);
		}

		inline Eigen::Array2i To2DIndex(const int index, const int bits)
		{
			DCHECK_LT(index, 1 << (2 * bits));
			const int mask = (1 << bits) - 1;
			return Eigen::Array2i(index & mask, (index >> bits) & mask);
		}

		inline std::vector<Cell *> &getCol(const Eigen::Vector3f &point)
		{
			return meta_cells_[ToFlatIndex(To2DIndex(point), bits_)];
		}

	protected:
		void grow();

	protected:
		int bits_;
		double resolution_;
		std::vector<std::vector<Cell *>> meta_cells_;
	};

} // namespace BGK

#endif