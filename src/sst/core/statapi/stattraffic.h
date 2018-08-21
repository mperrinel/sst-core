// Copyright 2009-2018 NTESS. Under the terms
// of Contract DE-NA0003525 with NTESS, the U.S.
// Government retains certain rights in this software.
//
// Copyright (c) 2009-2018, NTESS
// All rights reserved.
//
// This file is part of the SST software package. For license
// information, see the LICENSE file in the top level directory of the
// distribution.


#ifndef _H_SST_CORE_TRAFFIC_STATISTIC_
#define _H_SST_CORE_TRAFFIC_STATISTIC_

#include <cmath>
#include <limits>

#include <sst/core/sst_types.h>
#include <sst/core/warnmacros.h>

#include <sst/core/statapi/statfieldinfo.h>
#include <sst/core/statapi/statoutput.h>
#include <sst/core/statapi/statbase.h>
//#include <sst/core/baseComponent.h>

namespace SST {
namespace Statistics {

// NOTE: When calling base class members of classes derived from 
//       a templated base class.  The user must use "this->" in 
//       order to call base class members (to avoid a compiler 
//       error) because they are "nondependant named" and the 
//       templated base class is a "dependant named".  The 
//       compiler will not look in dependant named base classes 
//       when looking up independent names.
// See: http://www.parashift.com/c++-faq-lite/nondependent-name-lookup-members.html

/**
        \class TrafficStatistic

        Allows the online gathering of statistical information about a single quantity. The basic
        statistics are captured online removing the need to keep a copy of the values of interest.

        @tparam NumberBase A template for the basic numerical type of values
*/

template <typename NumberBase>
class TrafficStatistic : public Statistic<NumberBase>
{
public:

    TrafficStatistic(BaseComponent* comp, const std::string& statName, const std::string& statSubId, Params& statParams)
        : Statistic<NumberBase>(comp, statName, statSubId, statParams)
    {
        std::cout << "TrafficStatistic constructor: " << comp->getName() << std::endl;

        // Set the Name of this Statistic
        this->setStatisticTypeName("Traffic");
    }

    ~TrafficStatistic() {}

protected:    
    /**
        Present a new value to the class to be included in the statistics.
        @param value New value to be presented
    */
    void addData_impl(NumberBase value) override
    {
        // USE uintptr_t as value to remove the dynamical parameters;
        //        Params &traffic_intensity_params = this->getDynamicParams();

        auto tp = std::make_shared<traffic_event>();

        tp->time_ = value.time_;
        tp->id_ = value.id_;
        tp->p_ = value.p_;
        tp->type_ =value.type_;

        traffic_event_map_.insert({value.time_, tp});
    }

public:
    /**
        Provides the sum of the values presented so far.
        @return The sum of values presented to the class so far.
    */
    NumberBase getSum()
    {
        return m_sum;
    }

    void clearStatisticData() override
    {
       // m_sum = 0;
        this->setCollectionCount(0);
    }

    void registerOutputFields(StatisticOutput* statOutput) override
    {
      std::cout << "StatTraffic::registerOutputFields"<< std::endl;
      h_traffic_event = statOutput->registerField<NumberBase>("Traffic_event");
    }

    void outputStatisticData(StatisticOutput* statOutput, bool UNUSED(EndOfSimFlag)) override
    {
      std::cout << "StatTraffic::outputStatisticData"<< std::endl;
      std::cout << "reduce called on switch" << std::endl;
      std::cout << " reduce " << this->traffic_event_map_.size()<<std::endl;

      for (auto it = this->traffic_event_map_.cbegin(); it != this->traffic_event_map_.cend(); ++it){
        auto tp = std::make_shared<traffic_event>();
        tp->time_ = it->first;
        tp->id_ = it->second->id_;
        tp->type_ = it->second->type_;
        tp->p_ = it->second->p_;

        // How to compute intensity ?
        // Let's do it by making a +1/-1 depending of the previous scalar value
        int previousIntensity = !this->traffic_progress_map_.empty() ?
              this->traffic_progress_map_.crbegin()->second->intensity_ : 0;
        tp->intensity_ = it->second->type_ ? previousIntensity - 1 : previousIntensity + 1;

        this->traffic_progress_map_.insert({it->first, tp});
      }

      // need to do something with statouput

      for (auto& pair : this->traffic_progress_map_){
        statOutput->outputField(h_traffic_event, *(pair.second.get()));
      }

    }

    bool isStatModeSupported(StatisticBase::StatMode_t mode) const override
    {
        if (mode == StatisticBase::STAT_MODE_COUNT) {
            return true;
        }
        if (mode == StatisticBase::STAT_MODE_PERIODIC) {
            return true;
        }
        return false;
    }

private:
    NumberBase m_sum;
    std::multimap<uint64_t, std::shared_ptr<traffic_event>> traffic_event_map_;
    std::multimap<uint64_t, std::shared_ptr<traffic_event>> traffic_progress_map_;
    StatisticOutput::fieldHandle_t h_traffic_event;

};

} //namespace Statistics
} //namespace SST

#endif
