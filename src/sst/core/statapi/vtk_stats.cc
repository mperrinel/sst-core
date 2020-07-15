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

//#include <sstmac/backends/common/parallel_runtime.h>
//#include <sstmac/hardware/topology/topology.h>
//#include <sprockit/util.h>
//#include <sprockit/keyword_registration.h>

//#include <utility>

//#include <vtkInformation.h>
//#include <vtkStreamingDemandDrivenPipeline.h>
//#include <vtkVersion.h>
//#include <vtkSmartPointer.h>
//#include <vtkPolyVertex.h>
//#include <vtkVertex.h>
//#include <vtkQuad.h>
//#include <vtkLagrangeCurve.h>
//#include <vtkCubicLine.h>
//#include <vtkCellArray.h>
//#include <vtkQuadraticEdge.h>
//#include <vtkHexahedron.h>
//#include <vtkXMLUnstructuredGridReader.h>
//#include <vtkDataSetMapper.h>
//#include <vtkActor.h>
//#include <vtkRenderer.h>
//#include <vtkRenderWindow.h>
//#include <vtkRenderWindowInteractor.h>
//#include <vtkXMLUnstructuredGridWriter.h>
//#include <vtkUnstructuredGrid.h>
//#include <vtkVertexGlyphFilter.h>
//#include "vtkStdString.h"
#include "vtkExodusIIWriter.h"
#include <vtkIntArray.h>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkHexahedron.h>
#include <vtkUnstructuredGrid.h>

//RegisterKeywords(
//{ "intensity_levels", "the port occupancy level that should cause intensity transitions" },
//{ "idle_switch_color", "the color value to use for idle switches" },
//{ "idle_link_color", "the color value to use for idle links" },
//{ "highlight_switch_color", "the color value to use for specially highlighted switches" },
//{ "highlight_link_color", "the color value to use for specially highlighed links" },
//{ "min_interval", "the minimum time interval to paint as a given color" },
//{ "bidirectional_shift", "the displacement in space for each direction of a birectional link" },
//{ "filter", "filter on id" },
//{ "highlight", "a list of switches to fill with special highlight color"},
//{ "collect_delays", "whether to collect packet delays as the intensity"},
//{ "min_face_color", "the minimum color to use for active switches"},
//{ "max_face_color_sum", "the max color to allow for summation coloring of faces" },
//{ "scale_face_color_sum", "" },
//{ "active_face_width", "the width fraction of an active face" },
//{ "field_name", "" }
//);

namespace SST {
namespace Statistics {

//static constexpr double link_midpoint_shift = 2.0;

//void
//outputExodusWithSharedMap(const std::string& fileroot,
//   std::multimap<uint64_t, traffic_event>&& trafficMap,
//   StatVTK::display_config display_cfg,
//   Topology *topo)
//{

//  // The goal:
//  // The idea is to construct the geometry using coordinates of the switch and the coordinates of each ports.
//  // Then, we can apply varying time traffic values on ports and switch points.

//  // What we know:
//  // We assuming the traffic of each port start with 0
//  // for each time step (key) of the traffic map parameter, we know the traffic value to update for the (switch,port) associated.
//  // The switch traffic value itself needs to be recomputed using the mean of all current traffic values of the ports of the switch.

//  int num_switches = topo->numSwitches();
//  std::vector<Topology::VTKSwitchGeometry> geoms; geoms.reserve(num_switches);


//  std::unordered_map<uint32_t,uint64_t> outport_to_link;
//  for (int i=0; i < num_switches; ++i){
//    std::vector<Topology::Connection> conns;
//    topo->connectedOutports(i, conns);
//    for (Topology::Connection& conn : conns){
//      uint16_t id1 = conn.src;
//      uint16_t id2 = conn.dst;
//      uint16_t port1 = conn.src_outport;
//      uint16_t port2 = conn.dst_inport;
//      vtk_link link(id1,port1,id2,port2);
//      vtk_port port(id1,port1);
//      outport_to_link[port.id32()] = link.id64();

//      vtk_link test_link = vtk_link::construct(link.id64());
//      if (   test_link.id1 != link.id1 || test_link.id2 != link.id2
//          || test_link.port1 != link.port1 || test_link.port2 != link.port2){
//        spkt_abort_printf("Bad link bit arithmetic: link(%d,%d,%d,%d) != link(%d,%d,%d,%d)",
//                          int(link.id1), int(link.port1), int(link.id2), int(link.port2),
//                          int(test_link.id1), int(test_link.port1), int(test_link.id2), int(test_link.port2));
//      }
//    }

//    geoms.push_back(topo->getVtkGeometry(i));
//  }

//  int global_link_id = 0;
//  std::unordered_map<uint32_t,int> port_to_vtk_cell_mapping;
//  std::unordered_map<int,uint32_t> link_to_port_mapping;
//  for (auto& pair : trafficMap){
//    traffic_event& e = pair.second;
//    vtk_port p(e.id_, e.port_);
//    vtk_port test_p = vtk_port::construct(p.id32());
//    if (test_p.id != p.id || test_p.port != p.port){
//      spkt_abort_printf("Bad port bit arithmetic: port(%d,%d) != port(%d,%d)",
//                        int(p.id), int(p.port), int(test_p.id), int(test_p.port));
//    }

//    uint32_t port_global_id = p.id32();
//    auto iter = outport_to_link.find(port_global_id);
//    if (iter != outport_to_link.end()){ //not all ports get links
//      auto link_iter = port_to_vtk_cell_mapping.find(port_global_id);
//      if (link_iter == port_to_vtk_cell_mapping.end()){
//        port_to_vtk_cell_mapping[port_global_id] = global_link_id;
//        link_to_port_mapping[global_link_id] = port_global_id;
//        ++global_link_id;
//      }
//    }


//  }
//  std::cout << "vtk_stats : num links=" << port_to_vtk_cell_mapping.size() << std::endl;
//  std::cout << "vtk_stats : num switches=" << num_switches << std::endl;

//  //if a link is never used in the simulation, don't draw it
//  //limit the visualization to only those links that actually matter
//  int num_links_to_paint = port_to_vtk_cell_mapping.size();


//  // Init traffic array with default 0 traffic value
//  vtkSmartPointer<vtkIntArray> traffic = vtkSmartPointer<vtkIntArray>::New();
//  traffic->SetNumberOfComponents(1);
//  traffic->SetName("MyTraffic");

//  static constexpr int NUM_POINTS_PER_LINK = 3;
//  static constexpr int NUM_POINTS_PER_BOX = 8;
//  static constexpr int NUM_BOXES_PER_SWITCH = 7;
//  static constexpr int NUM_POINTS_PER_SWITCH = NUM_BOXES_PER_SWITCH * NUM_POINTS_PER_BOX;

//  int num_link_points = num_links_to_paint*NUM_POINTS_PER_LINK;

//  int num_unique_boxes = 0;
//  for (int swid=0; swid < num_switches; ++swid){
//    auto geom = topo->getVtkGeometry(swid);
//    num_unique_boxes += 1 + geom.ports.size();
//  }

//  int num_switch_points =  num_unique_boxes * NUM_POINTS_PER_BOX;

//  /** Each switch has 8 vertices
//   *  Each link has an additional (optional) midpoint defining it as a parabola */
//  int num_points = num_switch_points + num_link_points;

//  // Create the vtkPoints
//  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//  points->SetNumberOfPoints(num_points);

//  std::cout << "vtk_stats : num points = " << num_points << std::endl;



//  // Create the vtkCellArray
//  // 6 face cell array per switch
//  vtkSmartPointer<vtkCellArray> cell_array = vtkSmartPointer<vtkCellArray>::New();
//  std::vector<int> cell_types; cell_types.reserve(num_unique_boxes + num_links_to_paint);

//  std::vector<int> cell_offsets; cell_offsets.reserve(num_switches + 1);

//  int num_boxes = 0;
//  auto addBox = [&](Topology::VTKBoxGeometry& box_geom){
//    //auto v0 = box_geom.vertex(0); // corner (minus_x, minus_y, minus_z)
//    //auto v1 = box_geom.vertex(1); // corner (minus_x, plus_y, minus_z)
//    //auto v2 = box_geom.vertex(2); // corner (minus_x, minus_y, plus_z)
//    //auto v3 = box_geom.vertex(3); // corner (minus_x, plus_y, plus_z)
//    //auto v4 = box_geom.vertex(4); // corner (plus_x, minus_y, minus_z)
//    //auto v5 = box_geom.vertex(5); // corner (plus_x, plus_y, minus_z)
//    //auto v6 = box_geom.vertex(6); // corner (plus_x, minus_y, plus_z)
//    //auto v7 = box_geom.vertex(7); // corner (plus_x, plus_y, plus_z)

//    auto v0 = box_geom.vertex(0);
//    auto v1 = box_geom.vertex(1);
//    auto v2 = box_geom.vertex(3);
//    auto v3 = box_geom.vertex(2);
//    auto v4 = box_geom.vertex(4);
//    auto v5 = box_geom.vertex(5);
//    auto v6 = box_geom.vertex(7);
//    auto v7 = box_geom.vertex(6);

//    int offset = num_boxes * NUM_POINTS_PER_BOX;

//    points->SetPoint(offset + 0, v0.x, v0.y, v0.z);
//    points->SetPoint(offset + 1, v1.x, v1.y, v1.z);
//    points->SetPoint(offset + 2, v2.x, v2.y, v2.z);
//    points->SetPoint(offset + 3, v3.x, v3.y, v3.z);
//    points->SetPoint(offset + 4, v4.x, v4.y, v4.z);
//    points->SetPoint(offset + 5, v5.x, v5.y, v5.z);
//    points->SetPoint(offset + 6, v6.x, v6.y, v6.z);
//    points->SetPoint(offset + 7, v7.x, v7.y, v7.z);

//    vtkSmartPointer<vtkHexahedron> cell = vtkSmartPointer<vtkHexahedron>::New();
//    cell->GetPointIds()->SetId(0, offset + 0);
//    cell->GetPointIds()->SetId(1, offset + 1);
//    cell->GetPointIds()->SetId(2, offset + 2);
//    cell->GetPointIds()->SetId(3, offset + 3);
//    cell->GetPointIds()->SetId(4, offset + 4);
//    cell->GetPointIds()->SetId(5, offset + 5);
//    cell->GetPointIds()->SetId(6, offset + 6);
//    cell->GetPointIds()->SetId(7, offset + 7);
//    cell_array->InsertNextCell(cell);
//    cell_types.push_back(VTK_HEXAHEDRON);

//    num_boxes++;
//  };


//  // Use the geometry to place them correctly
//  for (int swid = 0; swid < num_switches; ++swid){
//    cell_offsets.push_back(cell_types.size());
//    auto geom = topo->getVtkGeometry(swid);
//    addBox(geom.box);

//    for (int port=0; port < geom.ports.size(); ++port){
//      auto box = geom.get_port_geometry(port);
//      addBox(box);
//    }
//  }
//  cell_offsets.push_back(cell_types.size());

//  if (num_switch_points != num_boxes * NUM_POINTS_PER_BOX){
//    spkt_abort_printf("Expected switch points %d does not match actual %d",
//                      num_switch_points, num_boxes * NUM_POINTS_PER_BOX);
//  }

//  //need to visit links in order
//  for (int link_id=0; link_id < link_to_port_mapping.size(); ++link_id){
//    uint32_t port_hash_id = link_to_port_mapping[link_id];
//    vtk_port vp = vtk_port::construct(port_hash_id);
//    uint64_t link_hash_id = outport_to_link[port_hash_id];
//    vtk_link link = vtk_link::construct(link_hash_id);
//    Topology::VTKSwitchGeometry switch1 = topo->getVtkGeometry(link.id1);//geoms[link.id1];
//    Topology::VTKSwitchGeometry switch2 = topo->getVtkGeometry(link.id2);//geoms[link.id2];
//    Topology::VTKBoxGeometry port1 = switch1.get_port_geometry(link.port1);
//    Topology::VTKBoxGeometry port2 = switch2.get_port_geometry(link.port2);

//    int offset = num_boxes*NUM_POINTS_PER_BOX + link_id * NUM_POINTS_PER_LINK;
//    auto c1 = port1.x_anchor();
//    auto c2 = port2.x_anchor();
//    auto tmp = port1.center();
//    //c3 = c1 + N*(c1-tmp)
//    auto c3 = c1;
//    c3.x += 5.0*(c1.x-tmp.x);
//    c3.y += 5.0*(c1.y-tmp.y);
//    c3.z += 5.0*(c1.z-tmp.z);

//    if (link.id1 != vp.id){
//      spkt_abort_printf("Port thinks switch id is %d, but link thinks switch id is %d",
//                        int(vp.id), int(link.id1));
//    }

//    //links are bidirectional - which means two links will get painted on top of each other
//    //for now (for lack of a better way) shift +/- on the z-axis to avoid conflicts
//    double z_shift = link.id1 > link.id2 ? display_cfg.bidirectional_shift : -display_cfg.bidirectional_shift;
//    c1.z += z_shift;
//    c2.z += z_shift;
//    /** move this quite far out of the plane for clarity */
//    //int link_shift = link.port1 + link.port2;
//    //c3.y += link_shift % 2 ? -link_shift*0.05 : link_shift*0.05;

//    points->SetPoint(offset + 0, c1.x, c1.y, c1.z);
//    points->SetPoint(offset + 1, c2.x, c2.y, c2.z);
//    points->SetPoint(offset + 2, c3.x, c3.y, c3.z);
//    if (topo->isCurvedVtkLink(link.id1, link.port1)){
//      vtkSmartPointer<vtkQuadraticEdge> line = vtkSmartPointer<vtkQuadraticEdge>::New();
//      line->GetPointIds()->SetId(0, offset + 0);
//      line->GetPointIds()->SetId(1, offset + 1);
//      line->GetPointIds()->SetId(2, offset + 2);
//      cell_array->InsertNextCell(line); cell_types.push_back(VTK_QUADRATIC_EDGE);
//    } else {
//      vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
//      line->GetPointIds()->SetId(0, offset + 0);
//      line->GetPointIds()->SetId(1, offset + 1);
//      cell_array->InsertNextCell(line); cell_types.push_back(VTK_LINE);
//    }
//  }

//  if (cell_types.size() != cell_array->GetNumberOfCells()){
//    spkt_abort_printf("Cell types and array size mismatch: %d != %d",
//                      cell_types.size() != cell_array->GetNumberOfCells());
//  }

//  std::cout << "vtk_stats : num cells=" << cell_array->GetNumberOfCells()<<std::endl;
//  std::cout << "vtk_stats : num events=" << trafficMap.size() << std::endl;


//  vtkSmartPointer<vtkUnstructuredGrid> unstructured_grid =
//      vtkSmartPointer<vtkUnstructuredGrid>::New();

//  unstructured_grid->SetPoints(points);
//  unstructured_grid->SetCells(cell_types.data(), cell_array);
//  unstructured_grid->GetCellData()->AddArray(traffic);

//  // Init Time Step
//  double current_time = -1;
//  double *time_step_value = new double[trafficMap.size() + 1];
//  time_step_value[0] = 0.;
//  int currend_index = 1;
//  for (auto it = trafficMap.cbegin(); it != trafficMap.cend(); ++it){
//    if (it->first != current_time){
//      current_time = it->first;
//      time_step_value[currend_index] = it->first;
//      ++currend_index;
//    }
//  }
//  // TOCHECK: time_step_value for trafficMap.size() > values >= currend_index isn't intialized (and shouldn't be used)
//  // time_step_value should be resized ?

//  vtkSmartPointer<vtkTrafficSource> trafficSource = vtkSmartPointer<vtkTrafficSource>::New();

//  trafficSource->SetDisplayParameters(display_cfg);
//  trafficSource->SetNumObjects(num_switches, num_links_to_paint, std::move(cell_offsets));

//  trafficSource->SetSteps(time_step_value);
//  trafficSource->SetNumberOfSteps(currend_index);

//  trafficSource->SetPoints(points);
//  trafficSource->SetGeometries(std::move(geoms));
//  trafficSource->SetCells(cell_array);
//  trafficSource->SetCellTypes(std::move(cell_types));

//  trafficSource->SetTraffics(traffic);
//  trafficSource->SetPortLinkMap(std::move(port_to_vtk_cell_mapping));
//  trafficSource->SetLocalToGlobalLinkMap(std::move(outport_to_link));
//  trafficSource->SetTrafficProgressMap(std::move(trafficMap));

//  vtkSmartPointer<vtkExodusIIWriter> exodusWriter = vtkSmartPointer<vtkExodusIIWriter>::New();
//  std::string fileName = fileroot + ".e";
//  exodusWriter->SetFileName(fileName.c_str());
//  exodusWriter->SetInputConnection (trafficSource->GetOutputPort());
//  exodusWriter->WriteAllTimeStepsOn ();
//  exodusWriter->Write();
//}

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
      if (vtkStat3dViz.my_shape_->isBox()) {
          numberOfPoints += NUM_POINTS_PER_BOX;
      }
      else if (vtkStat3dViz.my_shape_->isLine()) {
          numberOfPoints += NUM_POINTS_PER_LINK;
      }
  }

  points->SetNumberOfPoints(numberOfPoints);
  std::cout << "Number of points : "<<numberOfPoints <<std::endl;

  // Create the vtkCellArray
  // 6 face cell array per switch
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
  std::map<std::string, int> compNameToCellIdMap;

  std::vector<int> cell_types;
  cell_types.reserve(vtkStat3dVizSet.size());

  int i = 0;
  int cellId = 0;
  for (const auto& vtkStat3dViz : vtkStat3dVizSet) {
      Shape3D *shape = vtkStat3dViz.my_shape_;
      if (vtkStat3dViz.my_shape_->isBox()) {
          if (Box3D * box = dynamic_cast<Box3D *> (shape) ) {
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
          }
          else {
              // TODO: LOG ERROR, shape is box but the dynamic_cast fails ? That shouldn't happen
          }
      }    
      else if (vtkStat3dViz.my_shape_->isLine()) {
          if (Line3D * line = dynamic_cast<Line3D *> (shape) ) {
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
          }
          else {
              // TODO: LOG ERROR, shape is box but the dynamic_cast fails ? That shouldn't happen
          }
      }
      else  {
          // TODO: LOG ERROR, shape type unknown ? That shouldn't happen
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

  //  // TOCHECK: time_step_value for trafficMap.size() > values >= currend_index isn't intialized (and shouldn't be used)
  //  // time_step_value should be resized ?

  vtkSmartPointer<vtkTrafficSource> trafficSource = vtkSmartPointer<vtkTrafficSource>::New();
  trafficSource->SetCompNameToCellIdMap(std::move(compNameToCellIdMap));
  trafficSource->SetTrafficProgressMap(std::move(traffMap));
  trafficSource->SetTraffics(traffic);
  trafficSource->SetPoints(points);
  trafficSource->SetCells(cells);
  trafficSource->SetSteps(time_step_value);
  trafficSource->SetNumberOfSteps(currend_index);
  trafficSource->SetCellTypes(std::move(cell_types));
  //  trafficSource->SetDisplayParameters(display_cfg);
  //  trafficSource->SetNumObjects(num_switches, num_links_to_paint, std::move(cell_offsets));
  //  trafficSource->SetGeometries(std::move(geoms));
  //  trafficSource->SetPortLinkMap(std::move(port_to_vtk_cell_mapping));
  //  trafficSource->SetLocalToGlobalLinkMap(std::move(outport_to_link));

  vtkSmartPointer<vtkExodusIIWriter> exodusWriter = vtkSmartPointer<vtkExodusIIWriter>::New();
  std::string fileName = fileroot;
  if(fileroot.find(".e") ==  std::string::npos){
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
  if(it == traffic_event_map_.cend()){
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
