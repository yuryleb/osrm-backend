#ifndef OSRM_GUIDANCE_TURN_HANDLER_HPP_
#define OSRM_GUIDANCE_TURN_HANDLER_HPP_

#include "extractor/name_table.hpp"
#include "extractor/query_node.hpp"

#include "guidance/intersection.hpp"
#include "guidance/intersection_handler.hpp"
#include "guidance/is_through_street.hpp"

#include "util/node_based_graph.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace osrm::guidance
{

// Intersection handlers deal with all issues related to intersections.
class TurnHandler final : public IntersectionHandler
{
  public:
    TurnHandler(const util::NodeBasedDynamicGraph &node_based_graph,
                const extractor::EdgeBasedNodeDataContainer &node_data_container,
                const std::vector<util::Coordinate> &coordinates,
                const extractor::CompressedEdgeContainer &compressed_geometries,
                const extractor::RestrictionMap &node_restriction_map,
                const extractor::ObstacleMap &obstacle_nodes,
                const extractor::TurnLanesIndexedArray &turn_lanes_data,
                const extractor::NameTable &name_table,
                const extractor::SuffixTable &street_name_suffix_table);

    ~TurnHandler() override final = default;

    // check whether the handler can actually handle the intersection
    bool canProcess(const NodeID nid,
                    const EdgeID via_eid,
                    const Intersection &intersection) const override final;

    // process the intersection
    Intersection operator()(const NodeID nid,
                            const EdgeID via_eid,
                            Intersection intersection) const override final;

  private:
    struct Fork
    {
        const Intersection::iterator intersection_base;
        const Intersection::iterator begin;
        const Intersection::iterator end;
        const std::size_t size;
        Fork(const Intersection::iterator intersection_base,
             const Intersection::iterator begin,
             const Intersection::iterator end);
        ConnectedRoad &getRight() const;
        ConnectedRoad &getLeft() const;
        ConnectedRoad &getMiddle() const;
        ConnectedRoad &getRight();
        ConnectedRoad &getLeft();
        ConnectedRoad &getMiddle();
        std::size_t getRightIndex() const;
        std::size_t getLeftIndex() const;
    };

    bool isObviousOfTwo(const EdgeID via_edge,
                        const ConnectedRoad &road,
                        const ConnectedRoad &other) const;

    bool hasObvious(const EdgeID &via_edge, const Fork &fork) const;

    std::optional<Fork> findForkCandidatesByGeometry(Intersection &intersection) const;

    bool isCompatibleByRoadClass(const Intersection &intersection, const Fork fork) const;

    // Dead end.
    [[nodiscard]] Intersection handleOneWayTurn(Intersection intersection) const;

    // Mode Changes, new names...
    [[nodiscard]] Intersection handleTwoWayTurn(const EdgeID via_edge,
                                                Intersection intersection) const;

    // Forks, T intersections and similar
    [[nodiscard]] Intersection handleThreeWayTurn(const EdgeID via_edge,
                                                  Intersection intersection) const;

    // Handling of turns larger then degree three
    [[nodiscard]] Intersection handleComplexTurn(const EdgeID via_edge,
                                                 Intersection intersection) const;

    void
    handleDistinctConflict(const EdgeID via_edge, ConnectedRoad &left, ConnectedRoad &right) const;

    // Classification
    std::optional<Fork> findFork(const EdgeID via_edge, Intersection &intersection) const;

    [[nodiscard]] Intersection assignLeftTurns(const EdgeID via_edge,
                                               Intersection intersection,
                                               const std::size_t starting_at) const;

    [[nodiscard]] Intersection assignRightTurns(const EdgeID via_edge,
                                                Intersection intersection,
                                                const std::size_t up_to) const;
};

} // namespace osrm::guidance

#endif /*OSRM_GUIDANCE_TURN_HANDLER_HPP_*/
