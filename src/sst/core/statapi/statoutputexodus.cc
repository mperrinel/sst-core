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

#include "sst_config.h"
#include "sst/core/statapi/statoutputexodus.h"

#include "sst/core/simulation.h"
#include "sst/core/stringize.h"

namespace SST {
namespace Statistics {


StatisticOutputEXODUS::StatisticOutputEXODUS(Params& outputParameters)
    : StatisticOutput (outputParameters)
{
    // Announce this output object's name
    Output &out = Simulation::getSimulationOutput();
    out.verbose(CALL_INFO, 1, 0, " : StatisticOutputEXODUS enabled...\n");
    setStatisticOutputName("StatisticOutputEXODUS");

}


void StatisticOutputEXODUS::output(StatisticBase* statistic, bool endOfSimFlag) {
    this->lock();

    if(endOfSimFlag) {
      StatVTK* vtkStat = dynamic_cast<StatVTK *>(statistic);
      for (const auto & eventIte : vtkStat->getEvents()) {
        m_traffic_progress_map.insert(eventIte);
      }

      m_vtk_topology_cube_list_.insert(vtkStat->getTopology());
      m_vtk_link_list_.insert(vtkStat->getLink());

      this->unlock();
    }
}

bool StatisticOutputEXODUS::checkOutputParameters()
{
    bool foundKey;
    // Review the output parameters and make sure they are correct, and
    // also setup internal variables

    // Look for Help Param
    getOutputParameters().find<std::string>("help", "1", foundKey);
    if (true == foundKey) {
        return false;
    }

    // Get the parameters
    m_FilePath = getOutputParameters().find<std::string>("filepath", "./vtkStatisticOutput.e");


    if (0 == m_FilePath.length()) {
        // Filepath is zero length
        return false;
    }

    return true;
}

void StatisticOutputEXODUS::printUsage()
{
    // Display how to use this output object
    Output out("", 0, 0, Output::STDOUT);
    out.output(" : Usage - Sends all statistic output to a Exodus File.\n");
    out.output(" : Parameters:\n");
    out.output(" : help = Force Statistic Output to display usage\n");
    out.output(" : filepath = <Path to .e file> - Default is ./StatisticOutput.e\n");
    out.output(" : outputsimtime = 0 | 1 - Output Simulation Time - Default is 1\n");
    out.output(" : outputrank = 0 | 1 - Output Rank - Default is 1\n");
}

void StatisticOutputEXODUS::startOfSimulation()
{
    // Open the finalized filename
    if ( ! openFile() )
        return;
}

void StatisticOutputEXODUS::endOfSimulation()
{
    std::cout << "StatisticOutputEXODUS::endOfSimulation() "<< std::endl;

    this->outputConsole();
    StatVTK::outputExodus(m_FilePath, std::move(m_traffic_progress_map),
                          std::move(m_vtk_topology_cube_list_),
                          std::move(m_vtk_link_list_)
                          );

    // Close the file
    closeFile();
}

bool StatisticOutputEXODUS::openFile(void)
{
    return true;
}

void StatisticOutputEXODUS::closeFile(void)
{
}

void StatisticOutputEXODUS::registerStatistic(StatisticBase *stat)
{

}

void StatisticOutputEXODUS::startOutputGroup(StatisticGroup *grp)
{
}

void StatisticOutputEXODUS::stopOutputGroup()
{
}

void StatisticOutputEXODUS::startRegisterGroup(StatisticGroup *grp)
{
}

void StatisticOutputEXODUS::stopRegisterGroup()
{
}

void StatisticOutputEXODUS::outputConsole()
{
    std::multimap<std::string, std::multimap<uint64_t, int>> tf_nodes_map;
    for (const auto & eventIte : m_traffic_progress_map){
      auto nodeId = eventIte.second.compName_;
      auto portId = eventIte.second.port_;
      auto nodIdPortIdKey = nodeId +":"+ std::to_string(portId);
      auto resIt = tf_nodes_map.find(nodIdPortIdKey);
      if(resIt == tf_nodes_map.cend()){
        auto map = std::multimap<uint64_t, int>{};
        map.insert({eventIte.first, eventIte.second.color_});

        tf_nodes_map.insert({nodIdPortIdKey, map});
      } else {
        auto &map = resIt->second;
        map.insert({eventIte.first, eventIte.second.color_});
      }
    }

    //  TORM: display the map in the console
    for (const auto & nodeIte : tf_nodes_map){
      auto NodeId = nodeIte.first;
      const auto &map = nodeIte.second;
      std::cout<<NodeId<<"::: ";
      for(auto itMap = map.cbegin(); itMap != map.cend(); ++itMap){
        std::cout << "t:"<< itMap->first<< " color: " <<  itMap->second << "  ";
      }
      std::cout<<std::endl;
    }

}


void StatisticOutputEXODUS::outputExodus(const std::string& fileroot)
{
  // Check if that should be better to fill the exodus file here using VTK or to keep
  // the current behavior on StatVTK class.
}


} //namespace Statistics
} //namespace SST
