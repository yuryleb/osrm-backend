#ifndef OSRM_UTIL_CELL_STORAGE_HPP
#define OSRM_UTIL_CELL_STORAGE_HPP

// Mock interface, can be removed when we have an actual implementation
class MultiLevelPartition
{
    virtual GetCell(LevelID level, NodeID node) = 0;
};

class CellStorage
{
    public:
    using WeightOffset = std::uint32_t;
    using BoundaryOffset = std::uint32_t;
    using BoundarySize = std::uint32_t;
    using StartIndex = std::uint32_t;
    using TargetIndex = std::uint32_t;

    struct CellData
    {
        WeightOffset weight_offset;
        BoundaryOffset start_boundary_offset;
        BoundaryOffset target_boundary_offset;
        BoundarySize num_start_nodes;
        BoundarySize num_target_nodes;
    };



    // R/W View into one cell
    class Cell
    {
        private:
        BoundarySize num_start_nodes;
        BoundarySize num_target_nodes;

        EdgeWeight *const weights;

        using CellRowIterator = EdgeWeight*;
        class CellColumnIterator : public std::iterator<std::random_access_iterator_tag, EdgeWeight>
        {
            explicit CellColumnIterator(EdgeWeight *begin, std::size_t row_length)
                : current(begin), offset(row_length)
            {}

            private:
                EdgeWeight *current;
                std::size_t offset;
        };

        public:
        Cell(const CellData& data, EdgeWeights* const all_weight, const NodeID* const all_starts, const NodeID* const all_targets)
            : num_start_nodes{data.num_start_nodes},
            num_target_nodes{data.num_target_nodes},
            weights{all_weight + data.weight_offset},
            start_boundary{all_starts + data.start_boundary_offset},
            target_boundary{all_target + data.target_boundary_offset}
        {
        }

    };

    template<typename GraphT>
    CellStorage(const MultiLevelPartition &partition, const GraphT &base_graph)
    {
        const auto addLevel = [&](LevelID level) {
            std::vector<std::pair<CellID, NodeID>> start_boundary;
            std::vector<std::pair<CellID, NodeID>> target_boundary;

            for (auto node = 0; node < base_graph.NumberOfNodes(); ++node) {
                const CellID cell_id = partition.GetCell(level, node);
                bool is_start_node = false;
                bool is_target_node = false;
                bool is_boundary_node = f
                for (auto edge = base_graph.BeginEdges(node); edge < base_graph.EndEdges(node); ++edge) {
                    auto other = base_graph.GetTarget(edge);
                    is_boundary_node |= partition.GetCell(level, other) != partition;
                    is_source_node |= partition.GetCell(level, other) == partition && base_graph.GetData(edge).forward;
                    is_target_node |= partition.GetCell(level, other) == partition && base_graph.GetData(edge).backward;
                }

                if (is_boundary_node)
                {
                    if (is_source_node) source_boundary.emplace_back(cell_id, node);
                    if (is_target_node) target_boundary.emplace_back(cell_id, node);
                    // a partition that contains boundary nodes that have no arcs going into
                    // the cells or coming out of it is invalid. These nodes should be reassigned
                    // to a different cell.
                    OSRM_ASSERT(is_source_node || is_target_node);
                }
            }

            std::sort(source_boundary.begin(), source_boundary.end());
            for (auto source_iter = source_boundary.begin(),
                      source_end = source_boundary.end();
                 source_iter != source_end; ++source_iter)
            {
                const auto cell_id = source_iter->first;
                cells[cell_id].source_boundary_offset =
                while (source_iter != source_end && source_iter->first == cell_id)
                {
                    source_boundary.push_back(source_iter->second);
                    source_iter++;
                }
                cells[cell_id].num_source_nodes =
            }
            std::sort(target_boundary.begin(), target_boundary.end());
        };

        for (auto level = 0; level < partition.NumberOfLevels(); ++level)
        {
            addLevel(level);
        }
    }

    CellStorage(std::vector<EdgeWeight> &&weights_, std::vector<NodeID> &&start_boundary_,
                std::vector<NodeID> &&target_boundary_, std::vector<CellData> &&cells_,
                std::vector<std::size_t> &&level_to_cell_offset_)
    : weights(std::forward(weights_)),
    start_boundary(std::forward(start_boundary_)),
    target_boundary(std::forward(target_boundary_)),
    cells(std::forward(cell)),
    level_to_cell_offset(std::forward(level_to_cell))
    {
    }


    Cell getCell(LevelID level, CellID id) const {
       auto offset = level_to_cell_offset[level];
       return Cell {cells[offset + id], weights.data(), start_boundry, target_boundary};
    }

    private:

    std::vector<EdgeWeight> weights;
    std::vector<NodeID> start_boundary;
    std::vector<NodeID> target_boundary;
    std::vector<CellData> cells;
    std::vector<std::size_t> level_to_cell_offset;
};

#endif
