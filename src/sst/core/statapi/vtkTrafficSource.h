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
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTrafficSource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkTimeSource
 * @brief   creates a simple time varying data set.
 *
 * Creates a small easily understood time varying data set for testing.
 * The output is a vtkUntructuredGrid in which the point and cell values vary
 * over time in a sin wave. The analytic ivar controls whether the output
 * corresponds to a step function over time or is continuous.
 * The X and Y Amplitude ivars make the output move in the X and Y directions
 * over time. The Growing ivar makes the number of cells in the output grow
 * and then shrink over time.
*/

#ifndef vtkTrafficSource_h
#define vtkTrafficSource_h

#include "vtkUnstructuredGridAlgorithm.h"

#include "vtkCellArray.h"
#include "vtkIntArray.h"
#include "vtkPoints.h"
#include "vtkSmartPointer.h"
#include "statintensity.h"

using namespace SST::Statistics;

class vtkTrafficSource : public vtkUnstructuredGridAlgorithm
{
public:
  static vtkTrafficSource *New();
  vtkTypeMacro(vtkTrafficSource,vtkUnstructuredGridAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  void SetNumberOfSteps(double count);
  void SetSteps(double *steps);

  // Topology
  void SetPoints(vtkSmartPointer<vtkPoints> points);
  void SetCells(vtkSmartPointer<vtkCellArray> cells);

  template <class Vec> //allow move or copy
  void SetCellTypes(Vec&& types){
      CellTypes = std::forward<Vec>(types);
  }

  // Traffic
  void SetTrafficProgressMap(std::multimap<uint64_t, sorted_intensity_event>&& trafficProgressMap){
      traffic_progress_map_ = std::move(trafficProgressMap);
  }

  void SetTraffics(vtkSmartPointer<vtkIntArray> traffics);


  static void vtkOutputExodus(const std::string& fileroot,
        std::multimap<uint64_t, sorted_intensity_event>&& traffMap,
        std::set<Stat3DViz, compare_stat3dviz>&& stat3dVizSet);


protected:
  vtkTrafficSource();
  ~vtkTrafficSource() override;

  int RequestInformation(vtkInformation*,
                         vtkInformationVector**,
                         vtkInformationVector*) override;

  int RequestData(vtkInformation*,
                  vtkInformationVector**,
                  vtkInformationVector*) override;

  int NumSteps_;
  double *Steps_;
  std::multimap<uint64_t, sorted_intensity_event> traffic_progress_map_;
  vtkSmartPointer<vtkIntArray> Traffics;
  vtkSmartPointer<vtkPoints> Points;
  vtkSmartPointer<vtkCellArray> Cells;
  std::vector<int> CellTypes;

private:
  vtkTrafficSource(const vtkTrafficSource&) = delete;
  void operator=(const vtkTrafficSource&) = delete;
};

#endif

