#ifndef OSRM_UTIL_CELL_STORAGE_HPP
#define OSRM_UTIL_CELL_STORAGE_HPP

#include "util/assert.hpp"
#include "util/multi_level_partition.hpp"
#include "util/typedefs.hpp"
#include "util/for_each_range.hpp"

#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

namespace osrm
{
namespace util
{

class CellStorage
{
  public:
    using WeightOffset = std::uint32_t;
    using BoundaryOffset = std::uint32_t;
    using BoundarySize = std::uint32_t;
    using SourceIndex = std::uint32_t;
    using DestinationIndex = std::uint32_t;

    static constexpr auto INVALID_WEIGHT_OFFSET = std::numeric_limits<WeightOffset>::max();
    static constexpr auto INVALID_BOUNDARY_OFFSET = std::numeric_limits<BoundaryOffset>::max();

  private:
    struct CellData
    {
        WeightOffset weight_offset = INVALID_WEIGHT_OFFSET;
        BoundaryOffset source_boundary_offset = INVALID_BOUNDARY_OFFSET;
        BoundaryOffset destination_boundary_offset = INVALID_BOUNDARY_OFFSET;
        BoundarySize num_source_nodes = 0;
        BoundarySize num_destination_nodes = 0;
    };

    // Implementation of the cell view. We need a template parameter here
    // because we need to derive a read-only and read-write view from this.
    template <typename WeightPtrT> class CellImpl
    {
      private:
        using WeightValueT = EdgeWeight;
        using WeightRefT = decltype(*WeightPtrT());
        BoundarySize num_source_nodes;
        BoundarySize num_destination_nodes;

        WeightPtrT const weights;
        const NodeID *const source_boundary;
        const NodeID *const destination_boundary;

        using RowIterator = WeightPtrT;
        class ColumnIterator : public std::iterator<std::random_access_iterator_tag, EdgeWeight>
        {
          public:
            explicit ColumnIterator(WeightPtrT begin, std::size_t row_length)
                : current(begin), offset(row_length)
            {
            }

            WeightRefT operator*() const { return *current; }

            ColumnIterator &operator++()
            {
                current += offset;
                return *this;
            }

            ColumnIterator &operator+=(int amount)
            {
                current += offset * amount;
                return *this;
            }

            bool operator==(const ColumnIterator& other) const
            {
                return current == other.current;
            }

            bool operator!=(const ColumnIterator& other) const
            {
                return current != other.current;
            }

            int operator-(const ColumnIterator &other) const { return (current - other.current) / offset; }

          private:
            WeightPtrT current;
            std::size_t offset;
        };

        std::size_t GetRow(NodeID node) const
        {
            return std::find(source_boundary, source_boundary + num_source_nodes, node) -
                   source_boundary;
        }
        std::size_t GetColumn(NodeID node) const
        {
            return std::find(
                       destination_boundary, destination_boundary + num_destination_nodes, node) -
                   destination_boundary;
        }

      public:
        std::pair<RowIterator, RowIterator> GetOutWeight(NodeID node) const
        {
            auto row = GetRow(node);
            auto begin = weights + num_destination_nodes * row;
            auto end = begin + num_destination_nodes;
            return std::make_pair(begin, end);
        }

        std::pair<ColumnIterator, ColumnIterator> GetInWeight(NodeID node) const
        {
            auto column = GetColumn(node);
            auto begin = ColumnIterator{weights + column, num_destination_nodes};
            auto end =
                ColumnIterator{weights + column + num_source_nodes * num_destination_nodes,
                               num_destination_nodes};
            return std::make_pair(begin, end);
        }

        auto GetSourceNodes() const
        {
            return std::make_pair(source_boundary, source_boundary + num_source_nodes);
        }

        auto GetDestinationNodes() const
        {
            return std::make_pair(destination_boundary,
                                  destination_boundary + num_destination_nodes);
        }

        CellImpl(const CellData &data,
                 WeightPtrT const all_weight,
                 const NodeID *const all_sources,
                 const NodeID *const all_destinations)
            : num_source_nodes{data.num_source_nodes},
              num_destination_nodes{data.num_destination_nodes},
              weights{all_weight + data.weight_offset},
              source_boundary{all_sources + data.source_boundary_offset},
              destination_boundary{all_destinations + data.destination_boundary_offset}
        {
        }
    };

    inline std::size_t LevelIDToIndex(LevelID level) const { return level-1; }

  public:
    using Cell = CellImpl<EdgeWeight *>;
    using ConstCell = CellImpl<const EdgeWeight *>;

    template <typename GraphT>
    CellStorage(const MultiLevelPartition &partition, const GraphT &base_graph)
    {
        // pre-allocate storge for CellData so we can have random access to it by cell id
        unsigned number_of_cells = 0;
        for (LevelID level = 1u; level < partition.GetNumberOfLevels(); ++level)
        {
            level_to_cell_offset.push_back(number_of_cells);
            number_of_cells += partition.GetNumberOfCells(level);
        }
        level_to_cell_offset.push_back(number_of_cells);
        cells.resize(number_of_cells);

        for (LevelID level = 1u; level < partition.GetNumberOfLevels(); ++level)
        {
            auto level_offset = level_to_cell_offset[LevelIDToIndex(level)];

            std::vector<std::pair<CellID, NodeID>> level_source_boundary;
            std::vector<std::pair<CellID, NodeID>> level_destination_boundary;

            for (auto node = 0u; node < base_graph.GetNumberOfNodes(); ++node)
            {
                const CellID cell_id = partition.GetCell(level, node);
                bool is_source_node = false;
                bool is_destination_node = false;
                bool is_boundary_node = false;

                for (auto edge = base_graph.BeginEdges(node); edge < base_graph.EndEdges(node);
                     ++edge)
                {
                    auto other = base_graph.GetTarget(edge);
                    const auto &data = base_graph.GetEdgeData(edge);

                    is_boundary_node |= partition.GetCell(level, other) != cell_id;
                    is_source_node |= partition.GetCell(level, other) == cell_id && data.forward;
                    is_destination_node |=
                        partition.GetCell(level, other) == cell_id && data.backward;
                }

                if (is_boundary_node)
                {
                    if (is_source_node)
                        level_source_boundary.emplace_back(cell_id, node);
                    if (is_destination_node)
                        level_destination_boundary.emplace_back(cell_id, node);
                    // a partition that contains boundary nodes that have no arcs going into
                    // the cells or coming out of it is invalid. These nodes should be reassigned
                    // to a different cell.
                    BOOST_ASSERT_MSG(
                        is_source_node || is_destination_node,
                        "Node needs to either have incoming or outgoing edges in cell");
                }
            }

            std::sort(level_source_boundary.begin(), level_source_boundary.end());
            std::sort(level_destination_boundary.begin(), level_destination_boundary.end());

            util::for_each_range(
                level_source_boundary.begin(),
                level_source_boundary.end(),
                [&](auto begin, auto end) {
                    BOOST_ASSERT(std::distance(begin, end) > 0);

                    const auto cell_id = begin->first;
                    BOOST_ASSERT(level_offset + cell_id < cells.size());
                    auto &cell = cells[level_offset + cell_id];
                    cell.num_source_nodes = std::distance(begin, end);
                    cell.source_boundary_offset = source_boundary.size();

                    std::transform(begin,
                                   end,
                                   std::back_inserter(source_boundary),
                                   [](const auto &cell_and_node) { return cell_and_node.second; });
                });

            util::for_each_range(
                level_destination_boundary.begin(),
                level_destination_boundary.end(),
                [&](auto begin, auto end) {
                    BOOST_ASSERT(std::distance(begin, end) > 0);

                    const auto cell_id = begin->first;
                    BOOST_ASSERT(level_offset + cell_id < cells.size());
                    auto &cell = cells[level_offset + cell_id];
                    cell.num_destination_nodes = std::distance(begin, end);
                    cell.destination_boundary_offset = destination_boundary.size();

                    std::transform(begin,
                                   end,
                                   std::back_inserter(destination_boundary),
                                   [](const auto &cell_and_node) { return cell_and_node.second; });
                });
        }

        // Set weight offsets and calculate total storage size
        WeightOffset weight_offset = 0;
        for (auto &cell : cells)
        {
            cell.weight_offset = weight_offset;
            weight_offset += cell.num_source_nodes * cell.num_destination_nodes;
        }

        weights.resize(weight_offset+1, INVALID_EDGE_WEIGHT);
    }

    CellStorage(std::vector<EdgeWeight> weights_,
                std::vector<NodeID> source_boundary_,
                std::vector<NodeID> destination_boundary_,
                std::vector<CellData> cells_,
                std::vector<std::size_t> level_to_cell_offset_)
        : weights(std::move(weights_)), source_boundary(std::move(source_boundary_)),
          destination_boundary(std::move(destination_boundary_)), cells(std::move(cells_)),
          level_to_cell_offset(std::move(level_to_cell_offset_))
    {
    }

    ConstCell GetCell(LevelID level, CellID id) const
    {
        auto offset = level_to_cell_offset[LevelIDToIndex(level)];
        return ConstCell{
            cells[offset + id], &weights[0], &source_boundary[0], &destination_boundary[0]};
    }

    Cell GetCell(LevelID level, CellID id)
    {
        auto offset = level_to_cell_offset[LevelIDToIndex(level)];
        return Cell{cells[offset + id], &weights[0], &source_boundary[0], &destination_boundary[0]};
    }

  private:
    std::vector<EdgeWeight> weights;
    std::vector<NodeID> source_boundary;
    std::vector<NodeID> destination_boundary;
    std::vector<CellData> cells;
    std::vector<std::size_t> level_to_cell_offset;
};
}
}

#endif
