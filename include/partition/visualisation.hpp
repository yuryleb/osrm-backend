#ifndef OSRM_PARTITION_VISUALISATION_HPP_
#define OSRM_PARTITION_VISUALISATION_HPP_

#include "util/coordinate.hpp"
#include <vector>

namespace osrm
{
namespace partition
{
namespace visual
{

struct CutEntry
{
    std::vector<util::Coordinate> source_nodes;
    std::vector<util::Coordinate> sink_nodes;

    std::vector<std::vector<std::vector<util::Coordinate>>> augmenting_paths_by_step;
    std::vector<util::Coordinate> cut;
};

struct InertialFlowProgress
{
    std::vector<util::Coordinate> best_cut;
    std::vector<CutEntry> cuts_by_slope;
};

struct Bisection
{
    std::vector<std::vector<InertialFlowProgress>> algo_by_level;

    ~Bisection()
    {
        // visualise all received output
    }
};

} // namespace visual
} // namespace partition
} // namespace osrm

#endif // OSRM_PARTITION_VISUALISATION_HPP_
