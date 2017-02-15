#ifndef OSRM_UTIL_CELL_STORAGE_HPP
#define OSRM_UTIL_CELL_STORAGE_HPP

#include "util/assert.hpp"
#include "util/multi_level_partition.hpp"
#include "util/typedefs.hpp"

#include <algorithm>
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

    struct CellData
    {
        WeightOffset weight_offset;
        BoundaryOffset source_boundary_offset;
        BoundaryOffset destination_boundary_offset;
        BoundarySize num_source_nodes;
        BoundarySize num_destination_nodes;
    };

    // R/W View into one cell
    class Cell
    {
      private:
        BoundarySize num_source_nodes;
        BoundarySize num_destination_nodes;

        EdgeWeight *const weights;
        const NodeID *const source_boundary;
        const NodeID *const destination_boundary;

        using CellRowIterator = EdgeWeight *;
        class CellColumnIterator : public std::iterator<std::random_access_iterator_tag, EdgeWeight>
        {
            explicit CellColumnIterator(EdgeWeight *begin, std::size_t row_length)
                : current(begin), offset(row_length)
            {
            }

          private:
            EdgeWeight *current;
            std::size_t offset;
        };

      public:
        Cell(const CellData &data,
             EdgeWeight *const all_weight,
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

    template <typename GraphT>
    CellStorage(const MultiLevelPartition &partition, const GraphT &base_graph)
    {
        for (LevelID level = 0; level < partition.GetNumberOfLevels(); ++level)
        {
            std::vector<std::pair<CellID, NodeID>> level_source_boundary;
            std::vector<std::pair<CellID, NodeID>> level_destination_boundary;

            for (auto node = 0; node < base_graph.NumberOfNodes(); ++node)
            {
                const CellID cell_id = partition.GetCell(level, node);
                bool is_source_node = false;
                bool is_destination_node = false;
                bool is_boundary_node = false;

                for (auto edge = base_graph.BeginEdges(node); edge < base_graph.EndEdges(node);
                     ++edge)
                {
                    auto other = base_graph.GetDestination(edge);
                    is_boundary_node |= partition.GetCell(level, other) != cell_id;
                    is_source_node |= partition.GetCell(level, other) == cell_id &&
                                      base_graph.GetData(edge).forward;
                    is_destination_node |= partition.GetCell(level, other) == cell_id &&
                                           base_graph.GetData(edge).backward;
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
                    BOOST_ASSERT_MSG(is_source_node || is_destination_node,
                                "Node needs to either have incoming or outgoing edges in cell");
                }
            }

            std::sort(level_source_boundary.begin(), level_source_boundary.end());
            for (auto source_iter = level_source_boundary.begin(),
                      source_end = level_source_boundary.end();
                 source_iter != source_end;
                 ++source_iter)
            {
                const auto cell_id = source_iter->first;
                cells[cell_id].source_boundary_offset = source_boundary.size();
                while (source_iter != source_end && source_iter->first == cell_id)
                {
                    source_boundary.push_back(source_iter->second);
                    source_iter++;
                }
                cells[cell_id].num_source_nodes =
                    source_boundary.size() - cells[cell_id].source_boundary_offset;
            }

            std::sort(level_destination_boundary.begin(), level_destination_boundary.end());
            for (auto destination_iter = level_destination_boundary.begin(),
                      destination_end = level_destination_boundary.end();
                 destination_iter != destination_end;
                 ++destination_iter)
            {
                const auto cell_id = destination_iter->first;
                cells[cell_id].destination_boundary_offset = destination_boundary.size();
                while (destination_iter != destination_end && destination_iter->first == cell_id)
                {
                    destination_boundary.push_back(destination_iter->second);
                    destination_iter++;
                }
                cells[cell_id].num_destination_nodes =
                    destination_boundary.size() - cells[cell_id].destination_boundary_offset;
            }
        }
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

    Cell getCell(LevelID level, CellID id) const
    {
        auto offset = level_to_cell_offset[level];
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
