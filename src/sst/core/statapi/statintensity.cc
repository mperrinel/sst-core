/**
Copyright 2009-2020 National Technology and Engineering Solutions of Sandia,
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S.  Government 
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly 
owned subsidiary of Honeywell International, Inc., for the U.S. Department of 
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2020, NTESS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Questions? Contact sst-macro-help@sandia.gov
*/
#include "sst/core/statapi/statintensity.h"

namespace SST {
namespace Statistics {

IntensityStatistic::IntensityStatistic(BaseComponent* comp, const std::string& statName,
                 const std::string& statSubId, Params& statParams) :
  MultiStatistic<uint64_t, int, double>(comp, statName, statSubId, statParams), stat_3d_viz_(statParams)
{
    std::cout<<"StatVTK::StatVTK "<<" "<<statName<< " "<<statSubId << this->getCompName() <<std::endl;
    this->setStatisticTypeName("StatVTK");
    stat_3d_viz_.setName(this->getCompName());
}

void
IntensityStatistic::registerOutput(StatisticOutput * /*statOutput*/)
{

}

void
IntensityStatistic::registerOutputFields(StatisticFieldsOutput * /*statOutput*/)
{

}

void IntensityStatistic::outputStatisticFields(StatisticFieldsOutput* statOutput, bool UNUSED(EndOfSimFlag))
{

}

void IntensityStatistic::addData_impl(uint64_t time, int port, double intensity) {
    // Create a new intensity_event with the a new traffic event
    intensity_event event(time, intensity);
    intensity_event_vector_.push_back(std::move(event));
}

void IntensityStatistic::addData_impl_Ntimes(uint64_t N, uint64_t time, int port, double intensity) {

}

const std::vector<intensity_event>& IntensityStatistic::getEvents() const {
    return intensity_event_vector_;
}

Stat3DViz IntensityStatistic::geStat3DViz() const {
    return stat_3d_viz_;
}

}
}
