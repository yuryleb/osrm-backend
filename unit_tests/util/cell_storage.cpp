#include <boost/numeric/conversion/cast.hpp>
#include <boost/test/unit_test.hpp>

#include "util/cell_storage.hpp"

using namespace osrm;
using namespace osrm::util;

class MockMLP : public MultiLevelPartition
{
  public:
    CellID GetCell(LevelID level, NodeID node) const { return levels[level][node]; };

    LevelID GetHighestDifferentLevel(NodeID, NodeID) const { return 3; };

    std::size_t GetNumberOfLevels() const { levels.size(); }

    std::vector<std::vector<CellID>> levels;
};

BOOST_AUTO_TEST_SUITE(cell_storage_tests)

BOOST_AUTO_TEST_CASE(constructor)
{
    struct EdgeData
    {
    };
    StaticGraph<EdgeData> graph;
    CellStorage storage(mlp, graph);
}

BOOST_AUTO_TEST_SUITE_END()
