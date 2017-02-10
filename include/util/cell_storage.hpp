#ifndef OSRM_UTIL_CELL_STORAGE_HPP
#define OSRM_UTIL_CELL_STORAGE_HPP

#include "util/assert.hpp"
#include "util/multi_level_partition.hpp"

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
             EdgeWeights *const all_weight,
             const NodeID *const all_sources,
             const NodeID *const all_destinations)
            : num_source_nodes{data.num_source_nodes},
              num_destination_nodes{data.num_destination_nodes},
              weights{all_weight + data.weight_offset},
              source_boundary{all_sources + data.source_boundary_offset},
              destination_boundary{all_destination + data.destination_boundary_offset}
        {
        }
    };

    template <typename GraphT>
    CellStorage(const MultiLevelPartition &partition, const GraphT &base_graph)
    {
        for (LevelID level = 0; level < partition.NumberOfLevels(); ++level)
        {
            std::vector<std::pair<CellID, NodeID>> source_boundary;
            std::vector<std::pair<CellID, NodeID>> destination_boundary;

            for (auto node = 0; node < base_graph.NumberOfNodes(); ++node)
            {
                const CellID cell_id = partition.GetCell(level, node);
                bool is_source_node = false;
                bool is_destination_node = false;
                bool is_boundary_node = f for (auto edge = base_graph.BeginEdges(node);
                                               edge < base_graph.EndEdges(node);
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
                        source_boundary.emplace_back(cell_id, node);
                    if (is_destination_node)
                        destination_boundary.emplace_back(cell_id, node);
                    // a partition that contains boundary nodes that have no arcs going into
                    // the cells or coming out of it is invalid. These nodes should be reassigned
                    // to a different cell.
                    OSRM_ASSERT(is_source_node || is_destination_node);
                }
            }

            std::sort(source_boundary.begin(), source_boundary.end());
            for (auto source_iter = source_boundary.begin(), source_end = source_boundary.end();
                 source_iter != source_end;
                 ++source_iter)
            {
                const auto cell_id = source_iter->first;
                cells[cell_id].source_boundary_offset = source_boundart.size();
                while (source_iter != source_end && source_iter->first == cell_id)
                {
                    source_boundary.push_back(source_iter->second);
                    source_iter++;
                }
                cells[cell_id].num_source_nodes =
                    source_boundart.size() - cells[cell_id].source_boundary_offset;
            }

            std::sort(destination_boundary.begin(), destination_boundary.end());
            for (auto destination_iter = destination_boundary.begin(),
                      destination_end = destination_boundary.end();
                 destination_iter != destination_end;
                 ++destination_iter)
            {
                const auto cell_id = destination_iter->first;
                cells[cell_id].destination_boundary_offset = destination_boundart.size();
                while (destination_iter != destination_end && destination_iter->first == cell_id)
                {
                    destination_boundary.push_back(destination_iter->second);
                    destination_iter++;
                }
                cells[cell_id].num_destination_nodes =
                    destination_boundart.size() - cells[cell_id].destination_boundary_offset;
            }
        }
    }

    CellStorage(std::vector<EdgeWeight> &&weights_,
                std::vector<NodeID> &&source_boundary_,
                std::vector<NodeID> &&destination_boundary_,
                std::vector<CellData> &&cells_,
                std::vector<std::size_t> &&level_to_cell_offset_)
        : weights(std::forward(weights_)), source_boundary(std::forward(source_boundary_)),
          destination_boundary(std::forward(destination_boundary_)), cells(std::forward(cell)),
          level_to_cell_offset(std::forward(level_to_cell))
    {
    }

    Cell getCell(LevelID level, CellID id) const
    {
        auto offset = level_to_cell_offset[level];
        return Cell{cells[offset + id], weights.data(), source_boundry, destination_boundary};
    }

  private:
    std::vector<EdgeWeight> weights;
    std::vector<NodeID> source_boundary;
    std::vector<NodeID> destination_boundary;
    std::vector<CellData> cells;
    std::vector<std::size_t> level_to_cell_offset;
};

#endif
