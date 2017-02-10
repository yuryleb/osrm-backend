#ifndef OSRM_UTIL_MULTI_LEVEL_PARTITION_HPP
#define OSRM_UTIL_MULTI_LEVEL_PARTITION_HPP

#include <stdint>

namespace osrm
{
namespace util
{

using LevelID = std::uint8_t;
using CellID = std::uint32_t;

// Mock interface, can be removed when we have an actual implementation
class MultiLevelPartition
{
        // Returns the cell id of `node` at `level`
        virtual CellID GetCell(LevelID level, NodeID node) = 0;

        // Returns the highest level in which `first` and `second` are still in different cells
        virtual LevelID GetHighestDifferentLevel(NodeID first, NodeID second) = 0;

        virtual std::size_t GetNumberOfLevels() = 0;
};

}
}

#endif
