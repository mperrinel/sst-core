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
#include "sst/core/statapi/vtk_stats.h"
#include "sst/core/statapi/vtkTrafficSource.h"
#include "sst/core/simulation.h"


#include "vtkExodusIIWriter.h"
#include <vtkIntArray.h>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkHexahedron.h>
#include <vtkUnstructuredGrid.h>

namespace SST {
namespace Statistics {

void
StatVTK::outputExodus(const std::string& fileroot,
    std::multimap<uint64_t, traffic_event>&& traffMap,
    std::set<Stat3DViz, compare_stat3dviz>&& vtkStat3dVizSet)
{
    static constexpr int NUM_POINTS_PER_BOX = 8;
    static constexpr int NUM_POINTS_PER_LINK = 2;

    // Create the vtkPoints
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

    // Compute the number of the points
    int numberOfPoints = 0;
    for (const auto& vtkStat3dViz : vtkStat3dVizSet) {
      Shape3D *shape = vtkStat3dViz.my_shape_;
      switch (vtkStat3dViz.my_shape_->shape) {
      case Shape3D::Box: {
          numberOfPoints += NUM_POINTS_PER_BOX;
          break;
      }
      case Shape3D::Line: {
          numberOfPoints += NUM_POINTS_PER_LINK;
          break;
      }
      default: {
           Simulation::getSimulation()->getSimulationOutput().fatal(CALL_INFO, 1, "Cannot compute the number of points: "                                                                              "Unknown Shape3D type detected\n");
      }
      }
    }

    points->SetNumberOfPoints(numberOfPoints);

    // Create the vtkCellArray
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    std::map<std::string, int> compNameToCellIdMap;

    std::vector<int> cell_types;
    cell_types.reserve(vtkStat3dVizSet.size());

    int i = 0;
    int cellId = 0;
    for (const auto& vtkStat3dViz : vtkStat3dVizSet) {
      Shape3D *shape = vtkStat3dViz.my_shape_;
      switch (vtkStat3dViz.my_shape_->shape) {
      case Shape3D::Box: {
          Box3D * box = static_cast<Box3D*> (shape);
          // Fill the vtkPoints
          points->SetPoint(0 + i, box->x_origin_, box->y_origin_, box->z_origin_);
          points->SetPoint(1 + i, box->x_origin_ + box->x_extent_, box->y_origin_, box->z_origin_);
          points->SetPoint(2 + i, box->x_origin_ + box->x_extent_, box->y_origin_ + box->y_extent_, box->z_origin_);
          points->SetPoint(3 + i, box->x_origin_, box->y_origin_ + box->y_extent_, box->z_origin_);
          points->SetPoint(4 + i, box->x_origin_, box->y_origin_, box->z_origin_ + box->z_extent_);
          points->SetPoint(5 + i, box->x_origin_ + box->x_extent_, box->y_origin_, box->z_origin_ + box->z_extent_);
          points->SetPoint(6 + i, box->x_origin_ + box->x_extent_, box->y_origin_ + box->y_extent_, box->z_origin_ + box->z_extent_);
          points->SetPoint(7 + i, box->x_origin_, box->y_origin_ + box->y_extent_, box->z_origin_ + box->z_extent_);

          // Fill the cells
          vtkSmartPointer<vtkHexahedron> cell = vtkSmartPointer<vtkHexahedron>::New();
          for (int j= 0; j< NUM_POINTS_PER_BOX; ++j) {
              cell->GetPointIds()->SetId(j, i + j);
          }
          cells->InsertNextCell(cell);
          cell_types.push_back(VTK_HEXAHEDRON);

          i += NUM_POINTS_PER_BOX;
          break;
      }
      case Shape3D::Line: {
          Line3D * line = static_cast<Line3D*> (shape);
          // Fill the vtkPoints
          points->SetPoint(0 + i, line->x_first_, line->y_first_, line->z_first_);
          points->SetPoint(1 + i, line->x_second_, line->y_second_, line->z_second_);

          // Fill the cells
          vtkSmartPointer<vtkLine> cell = vtkSmartPointer<vtkLine>::New();
          for (int j= 0; j< NUM_POINTS_PER_LINK; ++j) {
              cell->GetPointIds()->SetId(j, i + j);
          }
          cells->InsertNextCell(cell);
          cell_types.push_back(VTK_LINE);

          i += NUM_POINTS_PER_LINK;
          break;
       }
       default: {
            Simulation::getSimulation()->getSimulationOutput().fatal(CALL_INFO, 1, "Cannot display the geometry: "                                                                                   "Unknown Shape3D type detected\n");
       }
    }

      compNameToCellIdMap.emplace( vtkStat3dViz.name_, cellId);
      cellId += 1;
    }

    // Init traffic array with default 0 traffic value
    vtkSmartPointer<vtkIntArray> traffic = vtkSmartPointer<vtkIntArray>::New();
    traffic->SetNumberOfComponents(1);
    traffic->SetName("MyTraffic");
    traffic->SetNumberOfValues(cells->GetNumberOfCells());

    for (int c = 0; c < cells->GetNumberOfCells(); ++c) {
        traffic->SetValue(c,0);
    }

    vtkSmartPointer<vtkUnstructuredGrid> unstructured_grid =
      vtkSmartPointer<vtkUnstructuredGrid>::New();

    unstructured_grid->SetPoints(points);
    unstructured_grid->SetCells(cell_types.data(), cells);
    unstructured_grid->GetCellData()->AddArray(traffic);

    // Init Time Step
    double current_time = -1;
    double *time_step_value = new double[traffMap.size() + 1];
    time_step_value[0] = 0.;
    int currend_index = 1;
    for (auto it = traffMap.cbegin(); it != traffMap.cend(); ++it){
        if (it->first != current_time){
            current_time = it->first;
            time_step_value[currend_index] = it->first;
            ++currend_index;
        }
    }

    vtkSmartPointer<vtkTrafficSource> trafficSource = vtkSmartPointer<vtkTrafficSource>::New();
    trafficSource->SetCompNameToCellIdMap(std::move(compNameToCellIdMap));
    trafficSource->SetTrafficProgressMap(std::move(traffMap));
    trafficSource->SetTraffics(traffic);
    trafficSource->SetPoints(points);
    trafficSource->SetCells(cells);
    trafficSource->SetSteps(time_step_value);
    trafficSource->SetNumberOfSteps(currend_index);
    trafficSource->SetCellTypes(std::move(cell_types));

    vtkSmartPointer<vtkExodusIIWriter> exodusWriter = vtkSmartPointer<vtkExodusIIWriter>::New();
    std::string fileName = fileroot;
    if(fileroot.find(".e") ==  std::string::npos) {
        fileName = fileroot + ".e";
    }
    exodusWriter->SetFileName(fileName.c_str());
    exodusWriter->SetInputConnection (trafficSource->GetOutputPort());
    exodusWriter->WriteAllTimeStepsOn ();
    exodusWriter->Write();
}


StatVTK::StatVTK(BaseComponent* comp, const std::string& statName,
                 const std::string& statSubId, Params& statParams) :
  MultiStatistic<uint64_t, int, double>(comp, statName, statSubId, statParams), vtk_stat_3d_viz_(statParams)
{
    std::cout<<"StatVTK::StatVTK "<<" "<<statName<< " "<<statSubId << this->getCompName() <<std::endl;
    this->setStatisticTypeName("StatVTK");
    lastTime_ = 0;
    vtk_stat_3d_viz_.setName(this->getCompName());
}

void
StatVTK::registerOutput(StatisticOutput * /*statOutput*/)
{

}

void
StatVTK::registerOutputFields(StatisticFieldsOutput * /*statOutput*/)
{

}

void StatVTK::outputStatisticFields(StatisticFieldsOutput* statOutput, bool UNUSED(EndOfSimFlag))
{

}

void StatVTK::addData_impl(uint64_t time, int port, double intensity) {

    // Update the traffic_event_map with the a new traffic event
    lastTime_ = time;
    auto it = traffic_event_map_.find(time);
    if (it == traffic_event_map_.cend()) {
        traffic_event event(time, port, intensity, this->getCompName());
        traffic_event_map_.emplace(event.time_, event);
    }
    else {
        it->second.color_ = intensity;
    }
}

void StatVTK::addData_impl_Ntimes(uint64_t N, uint64_t time, int port, double intensity) {

}

const std::multimap<uint64_t, traffic_event>& StatVTK::getEvents() const {
    return traffic_event_map_;
}

Stat3DViz StatVTK::geStat3DViz() const {
    return vtk_stat_3d_viz_;
}

}
}
