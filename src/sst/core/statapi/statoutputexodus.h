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

#ifndef _H_SST_CORE_STATISTICS_OUTPUTEXODUS
#define _H_SST_CORE_STATISTICS_OUTPUTEXODUS

#include "sst/core/sst_types.h"

#include "sst/core/statapi/statoutput.h"

namespace SST {
namespace Statistics {

/**
    \class StatisticOutputEXODUS

  The class for statistics output to a EXODUS formatted file
*/
class StatisticOutputEXODUS : public StatisticOutput
{
public:
    SST_ELI_REGISTER_DERIVED(
      StatisticOutput,
      StatisticOutputEXODUS,
      "macro",
      "exo",
      SST_ELI_ELEMENT_VERSION(1,0,0),
      "writes exodus output");

    /** Construct a StatOutputEXODUS
     * @param outputParameters - Parameters used for this Statistic Output
     */
    StatisticOutputEXODUS(Params& outputParameters);

    void output(StatisticBase* statistic, bool endOfSimFlag);

protected:
    /** Perform a check of provided parameters
     * @return True if all required parameters and options are acceptable
     */
    bool checkOutputParameters() override;

    /** Print out usage for this Statistic Output */
    void printUsage() override;

    /** Indicate to Statistic Output that simulation started.
     *  Statistic output may perform any startup code here as necessary.
     */
    void startOfSimulation() override;

    /** Indicate to Statistic Output that simulation ended.
     *  Statistic output may perform any shutdown code here as necessary.
     */
    void endOfSimulation() override;

private:
    // Start / Stop of register Fields
    virtual void registerStatistic(StatisticBase *stat);

    virtual void startOutputGroup(StatisticGroup* group);
    virtual void stopOutputGroup();

    virtual void startRegisterGroup(StatisticGroup* group);
    virtual void stopRegisterGroup();

protected:
    StatisticOutputEXODUS() {;} // For serialization

private:
    bool openFile();
    void closeFile();

private:
    FILE*                    m_hFile;
    std::string              m_FilePath;
    std::string              m_currentComponentName;
    std::string              m_currentStatisticName;
    std::string              m_currentStatisticSubId;
    std::string              m_currentStatisticType;
    bool                     m_outputSimTime;
    bool                     m_outputRank;
    bool					 m_firstEntry;
    bool					 m_firstField;
    bool 					 m_processedAnyStats;
    int						 m_curIndentLevel;

//    std::vector<SST::Statistics::StatisticGroup *> m_statGroups;
//    std::multimap<uint64_t, traffic_event> traffic_progress_map_;
//    std::function<void (std::multimap<uint64_t, traffic_event>, int, int)> callBackFct;

};

} //namespace Statistics
} //namespace SST

#endif
