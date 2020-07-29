// Copyright 2009-2020 NTESS. Under the terms
// of Contract DE-NA0003525 with NTESS, the U.S.
// Government retains certain rights in this software.
//
// Copyright (c) 2009-2020, NTESS
// All rights reserved.
//
// This file is part of the SST software package. For license
// information, see the LICENSE file in the top level directory of the
// distribution.

#ifndef _H_SST_CORE_INTENSITY_STATISTIC_
#define _H_SST_CORE_INTENSITY_STATISTIC_

#include "sst/core/sst_types.h"
#include "sst/core/warnmacros.h"

#include "sst/core/statapi/statbase.h"
#include "sst/core/statapi/statoutput.h"
#include "sst/core/statapi/stat3dviz.h"

namespace SST {
namespace Statistics {


/**
 * @brief The intensity_event struct
 * A intensity_event event contains the collected data through the StatVTK
 * time  = this is the time of the event.
 * port  = this is a the port on which the collection is done
 * intensity = this is the value (a double) that is written as a VTK state
 *         at a given timepoint. Depending on configuration,
 *         either intensity or level could be written as the color
*/
struct intensity_event {
    uint64_t time_; // progress time
    double intensity_;
    intensity_event(uint64_t t, double intensity) :
    time_(t), intensity_(intensity)
    {
    }

};

struct sorted_intensity_event {
  intensity_event ie_;
  uint64_t id_;
  sorted_intensity_event(uint64_t id, intensity_event event) :
  ie_(event), id_(id)
  {
  }
};

struct compare_intensity_events {
  bool operator()(const intensity_event& l, const intensity_event& r) {
      if (l.time_ != r.time_) return l.time_ < r.time_;
//      if (l.compName_ != r.compName_) return l.compName_ < r.compName_;
//      return l.port_ < r.port_;
  }
};

class IntensityStatistic : public MultiStatistic<uint64_t, int, double>
{

public:
  SST_ELI_REGISTER_MULTI_STATISTIC(
      IntensityStatistic,
      "sst",
      "IntensityStatistic",
      SST_ELI_ELEMENT_VERSION(1,0,0),
      "Collect intensity at each time point for a component",
      uint64_t,int,double)

  IntensityStatistic(BaseComponent* comp, const std::string& statName,
          const std::string& statSubId, Params& statParams);

  void registerOutput(StatisticOutput* statOutput);

  void registerOutputFields(StatisticFieldsOutput* statOutput) override;
  void outputStatisticFields(StatisticFieldsOutput* statOutput, bool UNUSED(EndOfSimFlag)) override;

  void addData_impl(uint64_t time, int port, double intensity) override;

  void addData_impl_Ntimes(uint64_t N, uint64_t time, int port, double intensity) override;

  const std::vector<intensity_event>& getEvents() const;
  Stat3DViz geStat3DViz() const;


private:
  std::vector<intensity_event> intensity_event_vector_;
//  std::multimap<uint64_t, intensity_event> intensity_event_map_;
  Stat3DViz stat_3d_viz_;
};

}
}

#endif
