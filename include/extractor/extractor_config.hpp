/*

Copyright (c) 2017, Project OSRM contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef EXTRACTOR_CONFIG_HPP
#define EXTRACTOR_CONFIG_HPP

#include "storage/io_config.hpp"

#include <filesystem>
#include <string>

namespace osrm::extractor
{

struct ExtractorConfig final : storage::IOConfig
{
    ExtractorConfig() noexcept
        : IOConfig(
              {
                  "",
              },
              {},
              {".osrm.nbg",
               ".osrm.restrictions",
               ".osrm.names",
               ".osrm.tls",
               ".osrm.tld",
               ".osrm.geometry",
               ".osrm.nbg_nodes",
               ".osrm.ebg_nodes",
               ".osrm.timestamp",
               ".osrm.edges",
               ".osrm.ebg",
               ".osrm.ramIndex",
               ".osrm.fileIndex",
               ".osrm.turn_duration_penalties",
               ".osrm.turn_weight_penalties",
               ".osrm.turn_penalties_index",
               ".osrm.enw",
               ".osrm.properties",
               ".osrm.icd",
               ".osrm.cnbg",
               ".osrm.cnbg_to_ebg",
               ".osrm.maneuver_overrides"})
    {
    }

    void UseDefaultOutputNames(const std::filesystem::path &base)
    {
        IOConfig::UseDefaultOutputNames(base);
    }

    std::filesystem::path input_path;
    std::filesystem::path profile_path;
    std::vector<std::filesystem::path> location_dependent_data_paths;
    std::string data_version;

    unsigned requested_num_threads = 0;
    unsigned small_component_size = 1000;

    bool use_metadata = false;
    bool parse_conditionals = false;
    bool use_locations_cache = true;
    bool dump_nbg_graph = false;
};
} // namespace osrm::extractor

#endif // EXTRACTOR_CONFIG_HPP
