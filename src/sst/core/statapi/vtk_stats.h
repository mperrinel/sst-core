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

#ifndef _H_SST_CORE_VTK_STATISTIC_
#define _H_SST_CORE_VTK_STATISTIC_

#include "sst/core/sst_types.h"
#include "sst/core/warnmacros.h"

#include "sst/core/statapi/statbase.h"
#include "sst/core/statapi/statoutput.h"
#include "sst/core/statapi/stat3dviz.h"

namespace SST {
namespace Statistics {


/**
 * @brief The traffic_event struct
 * A traffic event contains the collected data through the StatVTK
 * time  = this is the time of the event.
 * port  = this is a the port on which the collection is done
 * color = this is the value (a double) that is written as a VTK state
 *         at a given timepoint. Depending on configuration,
 *         either intensity or level could be written as the color
*/
struct traffic_event {
  uint64_t time_; // progress time
  int port_;
  //this is mutable due to the nonsense that is
  //C++ sets that does not allow modifying items in the set
  //even after collision
  mutable double color_;
  std::string compName_;

  traffic_event(uint64_t t, int port, double color, std::string compName) :
    time_(t), port_(port), color_(color), compName_(compName)
  {
  }

};



struct VTKStat3DViz : Stat3DViz {
    VTKStat3DViz(Params& params) : Stat3DViz(params){};

};

class StatVTK : public MultiStatistic<uint64_t, int, double>
{

public:
  SST_ELI_REGISTER_MULTI_STATISTIC(
      StatVTK,
      "sst",
      "StatVTK",
      SST_ELI_ELEMENT_VERSION(1,0,0),
      "Collect intensity at each time point for every component",
      uint64_t,int,double)

  StatVTK(BaseComponent* comp, const std::string& statName,
          const std::string& statSubId, Params& statParams);


  void registerOutput(StatisticOutput* statOutput);

  void registerOutputFields(StatisticFieldsOutput* statOutput) override;
  void outputStatisticFields(StatisticFieldsOutput* statOutput, bool UNUSED(EndOfSimFlag)) override;

  void addData_impl(uint64_t time, int port, double intensity) override;

  void addData_impl_Ntimes(uint64_t N, uint64_t time, int port, double intensity) override;

  const std::multimap<uint64_t, traffic_event>& getEvents() const;
  Stat3DViz geStat3DViz() const;
  static void outputExodus(const std::string& fileroot,
        std::multimap<uint64_t, traffic_event>&& traffMap,
        std::set<Stat3DViz, compare_stat3dviz>&& vtkStat3dVizSet);

private:
  struct compare_events {
    bool operator()(const traffic_event& l, const traffic_event& r){
      if (l.time_ != r.time_) return l.time_ < r.time_;
      if (l.compName_ != r.compName_) return l.compName_ < r.compName_;
      return l.port_ < r.port_;
    }
  };

  std::set<traffic_event, compare_events> sorted_event_list_;

  uint64_t lastTime_;
  std::multimap<uint64_t, traffic_event> traffic_event_map_;
  VTKStat3DViz vtk_stat_3d_viz_;

};

}
}

#endif
