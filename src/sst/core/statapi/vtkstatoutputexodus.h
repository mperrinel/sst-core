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

#ifndef _H_SST_CORE_STATISTICS_VTKOUTPUTEXODUS
#define _H_SST_CORE_STATISTICS_VTKOUTPUTEXODUS

#include "sst/core/sst_types.h"

#include "sst/core/statapi/statisticoutputexodus.h"
#include "sst/core/statapi/statintensity.h"

namespace SST {
namespace Statistics {

/**
    \class VTKStatisticOutputEXODUS

  The class for statistics output to a EXODUS formatted file using VTK
*/
class VTKStatisticOutputEXODUS : public StatisticOutputEXODUS
{
public:
    SST_ELI_REGISTER_DERIVED(
      StatisticOutputEXODUS,
      VTKStatisticOutputEXODUS,
      "sst",
      "vtkstatisticoutputexodus",
      SST_ELI_ELEMENT_VERSION(1,0,0),
      "writes vtk exodus output");

    /** Construct a StatOutputEXODUS
     * @param outputParameters - Parameters used for this Statistic Output
     */
    VTKStatisticOutputEXODUS(Params& outputParameters);

    void output(StatisticBase* statistic, bool endOfSimFlag) override;

    void outputExodus(const std::string& fileroot);

    /** True if this StatOutput can handle StatisticGroups */
    virtual bool acceptsGroups() const { return true; }

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
    void registerStatistic(StatisticBase *stat) override;

    void startOutputGroup(StatisticGroup* group) override;
    void stopOutputGroup() override;

    void startRegisterGroup(StatisticGroup* group) override;
    void stopRegisterGroup();

    void outputConsole();

    virtual void writeExodus() = 0;

protected:
    VTKStatisticOutputEXODUS() {;} // For serialization

private:
    bool openFile();
    void closeFile();

private:
    std::string              m_FilePath;
    std::set<intensity_event, compare_events> m_traffic_progress_map;
    std::set<Stat3DViz, compare_stat3dviz> m_stat_3d_viz_list_;

};

} //namespace Statistics
} //namespace SST

#endif
