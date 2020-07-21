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
  Module:    vtkTrafficSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkTrafficSource.h"

#include "vtkObjectFactory.h"
#include "vtkAlgorithm.h"
#include "vtkAlgorithmOutput.h"
#include "vtkCellData.h"
#include "vtkCellTypes.h"
#include "vtkDoubleArray.h"
#include "vtkIdTypeArray.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkMath.h"
#include "vtkPointData.h"
#include "vtkPoints.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkUnstructuredGrid.h"
#include <vector>

vtkStandardNewMacro(vtkTrafficSource);

//----------------------------------------------------------------------------
vtkTrafficSource::vtkTrafficSource()
{
    this->SetNumberOfInputPorts(0);
}

//----------------------------------------------------------------------------
vtkTrafficSource::~vtkTrafficSource()
{
}

void vtkTrafficSource::SetNumberOfSteps(double count)
{
    this->NumSteps_ = count;
}
void vtkTrafficSource::SetSteps(double *steps)
{
    this->Steps_ = steps;
}

// Topology
void vtkTrafficSource::SetPoints(vtkSmartPointer<vtkPoints> points)
{
    this->Points = points;
}

void vtkTrafficSource::SetCells(vtkSmartPointer<vtkCellArray> cells)
{
    this->Cells = cells;
}

// Traffic
void vtkTrafficSource::SetTraffics(vtkSmartPointer<vtkIntArray> traffics)
{
    this->Traffics = traffics;
}

//----------------------------------------------------------------------------
int vtkTrafficSource::RequestInformation(
  vtkInformation* reqInfo,
  vtkInformationVector** inVector,
  vtkInformationVector* outVector
  )
{
    if(!this->Superclass::RequestInformation(reqInfo,inVector,outVector))
    {
      return 0;
    }

    vtkInformation *info=outVector->GetInformationObject(0);

    //tell the caller that I can provide time varying data and
    //tell it what range of times I can deal with
    double tRange[2];
    tRange[0] = this->Steps_[0];
    tRange[1] = this->Steps_[this->NumSteps_-1];
    info->Set(
    vtkStreamingDemandDrivenPipeline::TIME_RANGE(),
    tRange,
    2);

    //tell the caller if this filter can provide values ONLY at discrete times
    //or anywhere within the time range

    info->Set(
    vtkStreamingDemandDrivenPipeline::TIME_STEPS(),
    this->Steps_,
    this->NumSteps_);


    info->Set(CAN_HANDLE_PIECE_REQUEST(), 1);

    return 1;
}

//----------------------------------------------------------------------------
int vtkTrafficSource::RequestData(
  vtkInformation* vtkNotUsed(reqInfo),
  vtkInformationVector** vtkNotUsed(inVector),
  vtkInformationVector* outVector
  )
{
    static int timestep = 0;

    vtkInformation *outInfo = outVector->GetInformationObject(0);
    vtkUnstructuredGrid *output= vtkUnstructuredGrid::SafeDownCast(
    outInfo->Get(vtkDataObject::DATA_OBJECT()));
    if (!output)
    {
      return 0;
    }

    uint64_t reqTS(0);
    if (outInfo->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
    {
        double requested = outInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
        reqTS = llround(requested);
    }

    //if analytic compute the value at that time
    //TODO: check if it's necessary to look up the nearest time and value from the table
    output->Initialize();
    output->GetInformation()->Set(vtkDataObject::DATA_TIME_STEP(), reqTS);

    //Updade and Send traffic to output
    auto currentIntensities =  traffic_progress_map_.equal_range(reqTS);

    for(auto it = currentIntensities.first; it != currentIntensities.second; ++it){
        traffic_event& event = it->second;
        int cell = this->compName_to_cellId_map.find(event.compName_)->second; //
        this->Traffics->SetValue(cell, event.color_);
    }
    ++timestep;

    output->GetCellData()->AddArray(this->Traffics);

    // Send Topology to output
    output->SetPoints(this->Points);
    output->SetCells(this->CellTypes.data(), this->Cells);

    return 1;
}

//----------------------------------------------------------------------------
void vtkTrafficSource::PrintSelf(ostream& os, vtkIndent indent)
{
    this->Superclass::PrintSelf(os, indent);
}
