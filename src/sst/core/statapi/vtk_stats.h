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

#define VTK_NUM_CELLS_PER_SWITCH 7

struct vtk_link {
  uint16_t id1;
  uint16_t id2;
  uint16_t port1;
  uint16_t port2;

  vtk_link(uint16_t i1, uint16_t p1, uint16_t i2, uint16_t p2) :
    id1(i1), port1(p1), id2(i2), port2(p2)
  {
    //if (i1 > i2){ //link is identified with i1 < i2
    //  std::swap(id1,id2);
    //  std::swap(port1,port2);
    //}
  }

  uint64_t id64() const {
    uint32_t i1 = id1;
    uint32_t p1 = port1;
    uint32_t side1 = (i1 << 16) | p1;
    uint32_t i2 = id2;
    uint32_t p2 = port2;
    uint32_t side2 = (i2 << 16) | p2;

    uint64_t s1 = side1;
    uint64_t s2 = side2;
    uint64_t id = (s1 << 32) | s2;
    return id;
  }

  static vtk_link construct(uint64_t id){
    uint64_t upper32_mask = (~uint64_t(0)) << 32;
    uint32_t upper16_mask  = (~uint32_t(0)) << 16;
    uint64_t lower32_mask = (~uint64_t(0)) >> 32;
    uint32_t lower16_mask = (~(uint32_t(0))) >> 16;


    uint32_t upper32 = (upper32_mask & id) >> 32;
    uint32_t lower32 = lower32_mask & id;

    uint16_t i1 = (upper16_mask & upper32) >> 16;
    uint16_t p1 = lower16_mask & upper32;
    uint16_t i2 = (upper16_mask & lower32) >> 16;
    uint16_t p2 = lower16_mask & lower32;

    return vtk_link(i1,p1,i2,p2);
  }
};

struct vtk_port {
  uint16_t id;
  uint16_t port;
  vtk_port(uint16_t i, uint16_t p)  :
    id(i), port(p) {}

  static vtk_port construct(uint32_t id){
    uint32_t upper16_mask  = (~uint32_t(0)) << 16;
    uint32_t lower16_mask = (~(uint32_t(0))) >> 16;
    uint16_t i = (upper16_mask & id) >> 16;
    uint16_t p = (lower16_mask & id);
    return vtk_port(i,p);
  }

  uint32_t id32() const {
    uint32_t i = id;
    uint32_t p = port;
    uint32_t myid = (i << 16) | p;
    return myid;
  }
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

  struct display_config {
    double idle_switch_color;
    double idle_link_color;
    double highlight_switch_color;
    double highlight_link_color;
    double bidirectional_shift;
    double min_face_color;
    double max_face_color_sum;
    double scale_face_color_sum;
    double active_face_width;
    std::string name;
    std::set<int> special_fills;
  };


  StatVTK(BaseComponent* comp, const std::string& statName,
          const std::string& statSubId, Params& statParams);


  void registerOutput(StatisticOutput* statOutput);
  void outputStatistic(StatisticOutput* statOutput, bool UNUSED(EndOfSimFlag));


  void registerOutputFields(StatisticFieldsOutput* statOutput) override;
  void outputStatisticFields(StatisticFieldsOutput* statOutput, bool UNUSED(EndOfSimFlag)) override;

  void addData_impl(uint64_t time, int port, double intensity) override;

  void addData_impl_Ntimes(uint64_t N, uint64_t time, int port, double intensity) override;

  const std::multimap<uint64_t, traffic_event>& getEvents() const;
  //  static void outputExodus(const std::string& fileroot,
  //      std::multimap<uint64_t, traffic_event>&& traffMap,
  //      const display_config& cfg,
  //      Topology *topo =nullptr);

  //  int id() const {
  //    return id_;
  //  }

  //  void configure(SwitchId sid, hw::Topology* top);

private:
  /**
   * @brief The port_state struct
   * The VTK collection has 3 different types of quantities
   * Intensity = this is the raw input value (a double) saying what some
   *             quantity of interest is (contention delay, queue depth)
   * Level = this is a discrete quantity that maps the intensity to an integer
   *         level based upon a given set of thresholds
   * Color = this is the value (a double) that is written as a VTK state
   *         at a given timepoint. Depending on configuration,
   *         either intensity or level could be written as the color
  */
  struct port_state {
    int active_ports;
    int congested_ports;
    uint64_t last_collection;
    uint64_t pending_collection_start;
    int current_level;
    double accumulated_color;
    double current_color;
    double active_vtk_color;
    uint64_t last_wait_finished;
    port_state() :
      accumulated_color(0.),
      current_level(0)
    {
    }
  };

  struct compare_events {
    bool operator()(const traffic_event& l, const traffic_event& r){
      if (l.time_ != r.time_) return l.time_ < r.time_;
      if (l.compName_ != r.compName_) return l.compName_ < r.compName_;
      return l.port_ < r.port_;
    }
  };

  std::vector<int> intensity_levels_;
  std::vector<port_state> port_states_;
  uint64_t min_interval_;
  int id_;

  std::vector<std::pair<int,int> > filters_;

  //  display_config display_cfg_;

  std::set<traffic_event, compare_events> sorted_event_list_;

  uint64_t lastTime_;
  std::multimap<uint64_t, traffic_event> traffic_event_map_;
  //  hw::Topology* top_;

  bool active_;
  bool flicker_;

};

}
}

#endif
