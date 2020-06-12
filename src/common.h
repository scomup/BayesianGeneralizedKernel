
#ifndef COMMON_H_
#define COMMON_H_

#include <cmath>
#include <vector>
#include <assert.h>

namespace BGK
{

    inline float sparse(const double x, const float l)
    {
        float d = std::abs(x);
        float r = d / l;
        float k = (2 + std::cos(2 * M_PI * r)) / 3 * (1 - r) + sin(2 * M_PI * r) / (2 * M_PI);
        return std::max(k,float(0.));
    }

    inline std::vector<float> sparseKernel(const float step, const float l)
    {
        std::vector<float> v;
        for (float x = -l; x < l; x += step)
        {
            float k = sparse(x, l);
            v.push_back(k);
        }
        return v;
    }
}

#endif