
#ifndef CELLMAP_H_
#define CELLMAP_H_

#include "Grid.h"

#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>
#include "glog/logging.h"

namespace BGK
{

	class Gridmap
	{
	public:
		Gridmap(double resolution);
		Grid* addGrid(const Eigen::Vector3f &point);
		Grid* getGrid(const Eigen::Vector3f &point);


		inline int ToFlatIndex(const Eigen::Array2i &index, const int bits)
		{
			DCHECK((index >= 0).all() && (index < (1 << bits)).all()) << index;
			return (index.y() << bits) + index.x();
		}

		inline Eigen::Array2i To2DIndex(const Eigen::Vector3f &point, const int bits) const
		{
			Eigen::Vector3f index = point.array() / grid_resolution_;
			return Eigen::Array2i(std::lround(index.x() + (1 << (bits - 1))),
								  std::lround(index.y() + (1 << (bits - 1))));
		}

		//inline Eigen::Array2i To2DIndex(const Eigen::Vector3f &point) const
		//{
		//	Eigen::Vector3f index = point.array() / resolution_;
		//	return Eigen::Array2i(std::lround(index.x() + (1 << (bits_ - 1))),
		//						  std::lround(index.y() + (1 << (bits_ - 1))));
		//}

		inline int ToFlatIndex(const Eigen::Vector3f &point, const int bits)
		{
			const Eigen::Array2i idx = To2DIndex(point, bits);
			return ToFlatIndex(idx, bits);
		}

		inline Eigen::Array2i To2DIndex(const int flatIdx, const int bits)
		{
			DCHECK_LT(flatIdx, 1 << (2 * bits));
			const int mask = (1 << bits) - 1;
			return Eigen::Array2i(flatIdx & mask, (flatIdx >> bits) & mask);
		}
//
		//inline Grid* getCol(const Eigen::Vector3f &point)
		//{
		//	return grids_[ToFlatIndex(To2DIndex(point), bits_)];
		//}

	protected:
		void grow();

	protected:
		int bits_;
		const float grid_resolution_;
		const float height_resolution_;
		const float radius_;
		const float bandwidth_;
		const float min_height_;
		const float max_height_;
		const int height_n_;
		std::vector<Grid*> grids_;
	};

} // namespace BGK

#endif