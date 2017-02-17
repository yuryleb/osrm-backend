#ifndef OSRM_PARTITION_VISUALISATION_HPP_
#define OSRM_PARTITION_VISUALISATION_HPP_

#include "util/coordinate.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "partition/bisection_graph.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/geojson_debug_logger.hpp"
#include "util/geojson_debug_policies.hpp"
#include "util/json_container.hpp"

namespace osrm
{
namespace partition
{
namespace visual
{

enum SCENARIO
{
    CUT,
    AUGMENTATION,
    SOURCE,
    SINK
};

using CutGuard = util::ScopedGeojsonLoggerGuard<util::CoordinateVectorToMultiPoint,
                                                util::LoggingScenario(SCENARIO::CUT)>;
using AugmentationGuard =
    util::ScopedGeojsonLoggerGuard<util::CoordinateVectorToLineString,
                                   util::LoggingScenario(SCENARIO::AUGMENTATION)>;
using SourceGuard = util::ScopedGeojsonLoggerGuard<util::CoordinateVectorToMultiPoint,
                                                   util::LoggingScenario(SCENARIO::SOURCE)>;
using SinkGuard = util::ScopedGeojsonLoggerGuard<util::CoordinateVectorToMultiPoint,
                                                 util::LoggingScenario(SCENARIO::SINK)>;

inline void MakeCircle(std::vector<util::Coordinate> &coordinates)
{
    std::int64_t sum_lat = 0, sum_lon = 0;
    std::int32_t min_lon = std::numeric_limits<int32_t>::max();
    // auto leftmost = coordinates.front();
    for (const auto c : coordinates)
    {
        auto lon = static_cast<std::int32_t>(c.lon);
        auto lat = static_cast<std::int32_t>(c.lat);
        sum_lat += lat;
        sum_lon += lon;
        if (lon < min_lon)
        {
            //       leftmost = c;
            min_lon = lon;
        }
    }
    auto avg_lat = sum_lat / coordinates.size();
    auto avg_lon = sum_lon / coordinates.size();

    // const auto avg = util::Coordinate(util::FixedLongitude{static_cast<std::int32_t>(avg_lon)},
    //                                  util::FixedLatitude{static_cast<std::int32_t>(avg_lat)});

    const auto avg = util::Coordinate(util::FixedLongitude{0}, util::FixedLatitude{0});
    const auto leftmost = util::Coordinate(util::FixedLongitude{-1100000}, util::FixedLatitude{0});
    std::sort(coordinates.begin(), coordinates.end(), [&](const auto lhs, const auto rhs) {
        return util::coordinate_calculation::computeAngle(leftmost, avg, lhs) <
               util::coordinate_calculation::computeAngle(leftmost, avg, rhs);
    });
}

inline void MakeLine2(std::vector<util::Coordinate> &coordinates)
{
    if( coordinates.empty() )
        return;

    std::int32_t min_lat = std::numeric_limits<std::int32_t>::max(),
                 max_lat = std::numeric_limits<std::int32_t>::min();
    std::int32_t min_lon = std::numeric_limits<std::int32_t>::max(),
                 max_lon = std::numeric_limits<std::int32_t>::min();
    std::int64_t sum_lat = 0, sum_lon = 0;
    for (const auto c : coordinates)
    {
        auto lon = static_cast<std::int32_t>(c.lon);
        auto lat = static_cast<std::int32_t>(c.lat);
        min_lat = std::min(min_lat, lat);
        max_lat = std::max(max_lat, lat);
        min_lon = std::min(min_lon, lon);
        max_lon = std::max(max_lon, lon);
        sum_lat += lat;
        sum_lon += lon;
    }

    auto avg_lat = sum_lat / coordinates.size();
    auto avg_lon = sum_lon / coordinates.size();
    const auto dlat = max_lat - min_lat;
    const auto dlon = max_lon - min_lon;

    const auto avg = util::Coordinate(util::FixedLongitude{static_cast<std::int32_t>(avg_lon)},
                                      util::FixedLatitude{static_cast<std::int32_t>(avg_lat)});

    const auto base = util::Coordinate(util::FixedLongitude{static_cast<std::int32_t>(avg_lon - dlat)},
                                          util::FixedLatitude{static_cast<std::int32_t>(avg_lat + dlon)}); 
    util::Coordinate reference;
    if (dlat > dlon)
    {
        reference = util::Coordinate(util::FixedLongitude{static_cast<std::int32_t>(min_lon)},
                                     util::FixedLatitude{static_cast<std::int32_t>(avg_lat)});
    }
    else
    {
        reference = util::Coordinate(util::FixedLongitude{static_cast<std::int32_t>(avg_lon)},
                                     util::FixedLatitude{static_cast<std::int32_t>(min_lat)});
    }

    std::sort(coordinates.begin(), coordinates.end(), [&](const auto lhs, const auto rhs) {
        return util::coordinate_calculation::computeAngle(base, reference, lhs) <
               util::coordinate_calculation::computeAngle(base, reference, rhs);
    });
}

inline void MakeLine(std::vector<util::Coordinate> &coordinates)
{
    std::int32_t min_lat = std::numeric_limits<std::int32_t>::max(),
                 max_lat = std::numeric_limits<std::int32_t>::min();
    std::int32_t min_lon = std::numeric_limits<std::int32_t>::max(),
                 max_lon = std::numeric_limits<std::int32_t>::min();
    for (const auto c : coordinates)
    {
        auto lon = static_cast<std::int32_t>(c.lon);
        auto lat = static_cast<std::int32_t>(c.lat);
        min_lat = std::min(min_lat, lat);
        max_lat = std::max(max_lat, lat);
        min_lon = std::min(min_lon, lon);
        max_lon = std::max(max_lon, lon);
    }

    const double slope = [&]() {
        if (max_lon - min_lon < 10)
            return 1.0;

        auto x = (double)(max_lat - min_lat) / (max_lon - min_lon);
        return cos(x) / sin(x);
    }();

    const auto sloped = [slope](const util::Coordinate coord) {
        auto lon = static_cast<std::int32_t>(coord.lon);
        auto lat = static_cast<std::int32_t>(coord.lat);
        return slope * lon + (1. - std::fabs(slope)) * lat;
    };

    const auto by_slope = [&sloped](const util::Coordinate lhs, const util::Coordinate rhs) {
        return sloped(lhs) < sloped(rhs);
    };

    std::sort(coordinates.begin(), coordinates.end(), by_slope);
}

inline std::vector<util::Coordinate> sampled(const std::vector<util::Coordinate> &coordinates)
{
    std::vector<util::Coordinate> result;
    result.reserve(coordinates.size() / 10+1);
    for( std::size_t i = 0; i < coordinates.size(); i += 10 )
        result.push_back(coordinates[i]);
    return result;
}

struct CutEntry
{
    std::vector<util::Coordinate> source_nodes;
    std::vector<util::Coordinate> sink_nodes;

    std::vector<std::vector<std::vector<util::Coordinate>>> augmenting_paths_by_step;
    std::vector<util::Coordinate> cut;

    void visualise(const std::size_t level, const std::size_t algorithm_id, const std::size_t slope)
    {
        util::json::Object property;
        property.values["algorithm"] = json::Number(algorithm_id);
        property.values["rotation"] = json::Number(slope);
        property.values["level"] = json::Number(level);

        SourceGuard::Write(sampled(source_nodes),property);
        SinkGuard::Write(sampled(sink_nodes),property);
        CutGuard::Write(cut,property);

        for (std::size_t step = 0; step < augmenting_paths_by_step.size(); ++step)
        {
            property.values["step"] = json::Number(step);
            for (const auto &p : augmenting_paths_by_step[step])
                AugmentationGuard::Write(p, property);
        }
    }
};

struct InertialFlowProgress
{
    std::vector<util::Coordinate> best_cut;
    std::vector<CutEntry> cuts_by_slope;

    void visualise(const std::size_t level, const std::size_t algorithm_id)
    {
        // TODO sort into line string
        util::json::Object property;
        property.values["algorithm"] = json::Number(algorithm_id);
        property.values["rotation"] = json::Number(0);
        property.values["level"] = json::Number(level);

        CutGuard::Write(best_cut, property);
        for (std::size_t slope = 0; slope < cuts_by_slope.size(); ++slope)
        {
            cuts_by_slope[slope].visualise(level, algorithm_id, slope + 1);
        }
    }
};

struct Bisection
{
    std::vector<std::vector<InertialFlowProgress>> algo_by_level;

    // visualise all received output
    void Output()
    {

        CutGuard cut_guard("partition-blog/best_cuts.geojson");
        AugmentationGuard augmentation_guard("partition-blog/augmentations.geojson");
        SourceGuard source_guard("partition-blog/sources.geojson");
        SinkGuard sink_guard("partition-blog/sinks.geojson");

        for (std::size_t level = 0; level < algo_by_level.size(); ++level)
        {
            if (!algo_by_level[level].empty())
            {
                // generate the guards for the output
                for (std::size_t algorithm_id = 0; algorithm_id < algo_by_level[level].size();
                     ++algorithm_id)
                    algo_by_level[level][algorithm_id].visualise(level, algorithm_id);
            }
        }
    }
};

} // namespace visual
} // namespace partition
} // namespace osrm

#endif // OSRM_PARTITION_VISUALISATION_HPP_
