#include <boost/numeric/conversion/cast.hpp>
#include <boost/test/unit_test.hpp>

#include "util/cell_storage.hpp"
#include "util/static_graph.hpp"

#define CHECK_RANGE_SIZE(range, ref) BOOST_CHECK_EQUAL(range.second - range.first, ref)

using namespace osrm;
using namespace osrm::util;

class MockMLP final : public MultiLevelPartition
{
  public:
    CellID GetCell(LevelID level, NodeID node) const { return levels[level - 1][node]; };

    LevelID GetHighestDifferentLevel(NodeID, NodeID) const { return 3; };

    std::size_t GetNumberOfLevels() const { return levels.size() + 1; }

    std::size_t GetNumberOfCells(LevelID level) const
    {
        auto max_id = 0;
        for (auto cell : levels[level - 1])
            max_id = std::max<CellID>(max_id, cell);
        return max_id + 1;
    }

    MockMLP(std::vector<std::vector<CellID>> levels_) : levels(std::move(levels_)) {}

    std::vector<std::vector<CellID>> levels;
};

struct MockEdge
{
    NodeID start;
    NodeID target;
};

auto makeGraph(const std::vector<MockEdge> &mock_edges)
{
    struct EdgeData
    {
        bool forward;
        bool backward;
    };
    using Edge = util::StaticGraph<EdgeData>::InputEdge;
    std::vector<Edge> edges;
    std::size_t max_id = 0;
    for (const auto &m : mock_edges)
    {
        max_id = std::max<std::size_t>(max_id, std::max(m.start, m.target));
        edges.push_back(Edge{m.start, m.target, EdgeData{true, false}});
        edges.push_back(Edge{m.target, m.start, EdgeData{false, true}});
    }
    std::sort(edges.begin(), edges.end());
    return util::StaticGraph<EdgeData>(max_id + 1, edges);
}

BOOST_AUTO_TEST_SUITE(cell_storage_tests)

BOOST_AUTO_TEST_CASE(constructor)
{
    // node:                0  1  2  3  4  5  6  7  8  9 10 11
    std::vector<CellID> l1{{0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5}};
    std::vector<CellID> l2{{0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3}};
    std::vector<CellID> l3{{0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1}};
    std::vector<CellID> l4{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    MockMLP mlp{{l1, l2, l3, l4}};

    std::vector<MockEdge> edges = {
        // edges sorted into border/internal by level
        //  level:  (1) (2) (3) (4)
        {0, 1},   //  i   i   i   i
        {2, 3},   //  i   i   i   i
        {3, 7},   //  b   b   b   i
        {4, 0},   //  b   b   b   i
        {4, 5},   //  i   i   i   i
        {5, 6},   //  b   i   i   i
        {6, 4},   //  b   i   i   i
        {6, 7},   //  i   i   i   i
        {7, 11},  //  b   b   i   i
        {8, 9},   //  i   i   i   i
        {9, 8},   //  i   i   i   i
        {10, 11}, //  i   i   i   i
        {11, 10}  //  i   i   i   i
    };

    auto graph = makeGraph(edges);

    // nodes sorted into border/internal by level
    //   (1) (2) (3) (4)
    // 0  b   b   b   i
    // 1  i   i   i   i
    // 2  i   i   i   i
    // 3  b   b   b   i
    // 4  b   b   b   i
    // 5  b   i   i   i
    // 6  b   i   i   i
    // 7  b   b   i   i
    // 8  i   i   i   i
    // 9  i   i   i   i
    // 10 i   i   i   i
    // 11 b   b   i   i

    // 1/0: 0 : 1,1,0
    // 1/2: 4 : 1,1,0
    // 1/3: 6 : 1,1,0
    // 1/5: 11 : 1,1,1

    // 1/1: 3 : 1,0,1
    // 1/2: 5 : 1,0,1
    // 1/3: 7 : 1,0,1

    // 2/0: 0 : 1,1,0
    // 2/0: 3 : 1,0,1
    // 2/1: 4 : 1,1,1
    // 2/1: 7 : 1,0,1
    // 2/3: 11 : 1,1,1

    // 3/0: 0 : 1,1,0
    // 3/0: 3 : 1,0,1
    // 3/1: 4 : 1,1,1
    // 3/1: 7 : 1,1,1

    // test const storage
    const CellStorage const_storage(mlp, graph);
    auto const_cell_1_0 = const_storage.GetCell(1, 0);
    auto const_cell_1_1 = const_storage.GetCell(1, 1);
    auto const_cell_1_2 = const_storage.GetCell(1, 2);
    auto const_cell_1_3 = const_storage.GetCell(1, 3);
    auto const_cell_1_4 = const_storage.GetCell(1, 4);
    auto const_cell_1_5 = const_storage.GetCell(1, 5);

    std::vector<NodeID> ref_sources_cell_1_0 = {0};
    std::vector<NodeID> ref_sources_cell_1_1 = {};
    std::vector<NodeID> ref_sources_cell_1_2 = {4};
    std::vector<NodeID> ref_sources_cell_1_3 = {6};
    std::vector<NodeID> ref_sources_cell_1_4 = {};
    std::vector<NodeID> ref_sources_cell_1_5 = {11};

    std::vector<NodeID> ref_destinations_cell_1_0 = {};
    std::vector<NodeID> ref_destinations_cell_1_1 = {3};
    std::vector<NodeID> ref_destinations_cell_1_2 = {5};
    std::vector<NodeID> ref_destinations_cell_1_3 = {7};
    std::vector<NodeID> ref_destinations_cell_1_4 = {};
    std::vector<NodeID> ref_destinations_cell_1_5 = {11};

    auto sources_cell_1_0 = const_cell_1_0.GetSourceNodes();
    auto sources_cell_1_1 = const_cell_1_1.GetSourceNodes();
    auto sources_cell_1_2 = const_cell_1_2.GetSourceNodes();
    auto sources_cell_1_3 = const_cell_1_3.GetSourceNodes();
    auto sources_cell_1_4 = const_cell_1_4.GetSourceNodes();
    auto sources_cell_1_5 = const_cell_1_5.GetSourceNodes();

    auto destinations_cell_1_0 = const_cell_1_0.GetDestinationNodes();
    auto destinations_cell_1_1 = const_cell_1_1.GetDestinationNodes();
    auto destinations_cell_1_2 = const_cell_1_2.GetDestinationNodes();
    auto destinations_cell_1_3 = const_cell_1_3.GetDestinationNodes();
    auto destinations_cell_1_4 = const_cell_1_4.GetDestinationNodes();
    auto destinations_cell_1_5 = const_cell_1_5.GetDestinationNodes();

    BOOST_CHECK_EQUAL_COLLECTIONS(sources_cell_1_0.first,
                                  sources_cell_1_0.second,
                                  ref_sources_cell_1_0.begin(),
                                  ref_sources_cell_1_0.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(sources_cell_1_1.first,
                                  sources_cell_1_1.second,
                                  ref_sources_cell_1_1.begin(),
                                  ref_sources_cell_1_1.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(sources_cell_1_2.first,
                                  sources_cell_1_2.second,
                                  ref_sources_cell_1_2.begin(),
                                  ref_sources_cell_1_2.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(sources_cell_1_3.first,
                                  sources_cell_1_3.second,
                                  ref_sources_cell_1_3.begin(),
                                  ref_sources_cell_1_3.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(sources_cell_1_4.first,
                                  sources_cell_1_4.second,
                                  ref_sources_cell_1_4.begin(),
                                  ref_sources_cell_1_4.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(sources_cell_1_5.first,
                                  sources_cell_1_5.second,
                                  ref_sources_cell_1_5.begin(),
                                  ref_sources_cell_1_5.end());

    BOOST_CHECK_EQUAL_COLLECTIONS(destinations_cell_1_0.first,
                                  destinations_cell_1_0.second,
                                  ref_destinations_cell_1_0.begin(),
                                  ref_destinations_cell_1_0.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(destinations_cell_1_1.first,
                                  destinations_cell_1_1.second,
                                  ref_destinations_cell_1_1.begin(),
                                  ref_destinations_cell_1_1.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(destinations_cell_1_2.first,
                                  destinations_cell_1_2.second,
                                  ref_destinations_cell_1_2.begin(),
                                  ref_destinations_cell_1_2.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(destinations_cell_1_3.first,
                                  destinations_cell_1_3.second,
                                  ref_destinations_cell_1_3.begin(),
                                  ref_destinations_cell_1_3.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(destinations_cell_1_4.first,
                                  destinations_cell_1_4.second,
                                  ref_destinations_cell_1_4.begin(),
                                  ref_destinations_cell_1_4.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(destinations_cell_1_5.first,
                                  destinations_cell_1_5.second,
                                  ref_destinations_cell_1_5.begin(),
                                  ref_destinations_cell_1_5.end());

    auto out_range_1_0_0 = const_cell_1_0.GetOutWeight(0);
    auto out_range_1_2_4 = const_cell_1_2.GetOutWeight(4);
    auto out_range_1_3_6 = const_cell_1_3.GetOutWeight(6);
    auto out_range_1_5_11 = const_cell_1_5.GetOutWeight(11);

    auto in_range_1_1_3 = const_cell_1_1.GetInWeight(3);
    auto in_range_1_2_5 = const_cell_1_2.GetInWeight(5);
    auto in_range_1_3_7 = const_cell_1_3.GetInWeight(7);
    auto in_range_1_5_11 = const_cell_1_5.GetInWeight(11);

    CHECK_RANGE_SIZE(out_range_1_0_0, 0);
    CHECK_RANGE_SIZE(out_range_1_2_4, 1);
    CHECK_RANGE_SIZE(out_range_1_3_6, 1);
    CHECK_RANGE_SIZE(out_range_1_5_11, 1);

    CHECK_RANGE_SIZE(in_range_1_1_3, 0);
    CHECK_RANGE_SIZE(in_range_1_2_5, 1);
    CHECK_RANGE_SIZE(in_range_1_3_7, 1);
    CHECK_RANGE_SIZE(in_range_1_5_11, 1);

    // test non-const storage
    //const CellStorage storage(mlp, graph);
    //auto cell_1_0 = storage.GetCell(1, 0);
    //auto cell_1_1 = storage.GetCell(1, 1);
    //auto cell_1_2 = storage.GetCell(1, 2);
    //auto cell_1_3 = storage.GetCell(1, 3);
    //auto cell_1_4 = storage.GetCell(1, 4);
    //auto cell_1_5 = storage.GetCell(1, 5);
}

BOOST_AUTO_TEST_SUITE_END()
