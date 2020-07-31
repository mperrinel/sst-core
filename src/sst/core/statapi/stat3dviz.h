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

#ifndef _H_SST_CORE_STAT_3D_VIZ
#define _H_SST_CORE_STAT_3D_VIZ

#include "sst/core/sst_types.h"
#include "sst/core/warnmacros.h"
#include "sst/core/simulation.h"

#include <string>

static const std::string cubeKey = "cube";
static const std::string lineKey = "line";

namespace SST {
namespace Statistics {

// use common base class for the topology/geometry
struct Shape3D {
   enum Shape {
     Box,
     Line
   };
   Shape shape;

   Shape3D(Shape s) :
     shape(s)
   {
   }
};
struct Box3D : Shape3D {
    Box3D(double x_origin, double y_origin, double z_origin,
          double x_extent, double y_extent, double z_extent) :
      Shape3D(Shape::Box),
      x_origin_(x_origin), y_origin_(y_origin), z_origin_(z_origin),
      x_extent_(x_extent), y_extent_(y_extent), z_extent_(z_extent)
    {
    }

    double x_origin_;
    double y_origin_;
    double z_origin_;
    double x_extent_;
    double y_extent_;
    double z_extent_;
};

struct Line3D : Shape3D {
    Line3D(double x_first, double y_first, double z_first,
           double x_second, double y_second, double z_second) :
        Shape3D(Shape::Line),
        x_first_(x_first), y_first_(y_first), z_first_(z_first),
        x_second_(x_second), y_second_(y_second), z_second_(z_second)
    {
    }

    double x_first_;
    double y_first_;
    double z_first_;
    double x_second_;
    double y_second_;
    double z_second_;

};

struct Stat3DViz {

    Stat3DViz(Params& params){
        std::string shape = params.find<std::string>("shape", "");
        if (shape == cubeKey) {
            double x_origin = params.find<double>("x_origin", "");
            double y_origin = params.find<double>("y_origin", "");
            double z_origin = params.find<double>("z_origin", "");
            double x_extent = params.find<double>("x_extent", "");
            double y_extent = params.find<double>("y_extent", "");
            double z_extent = params.find<double>("z_extent", "");
            my_shape_ = new Box3D(x_origin, y_origin, z_origin, x_extent, y_extent, z_extent);
        }
        else if (shape == lineKey) {
            double x_first = params.find<double>("x_first", "");
            double y_first = params.find<double>("y_first", "");
            double z_first = params.find<double>("z_first", "");
            double x_second = params.find<double>("x_second", "");
            double y_second = params.find<double>("y_second", "");
            double z_second = params.find<double>("z_second", "");
            my_shape_ = new Line3D(x_first, y_first, z_first, x_second, y_second, z_second);

        }
        else {
            Simulation::getSimulation()->getSimulationOutput().fatal(CALL_INFO, 1, "Cannot create a correct Shape3D: "
                                                                                   "Unknown %s type detected\n", shape.c_str());
        }
    }

    void setId(uint64_t id) {
        id_ = id;
    }

    uint64_t id_;
    Shape3D* my_shape_;
};

struct compare_stat3dviz {
  bool operator()(const Stat3DViz& l, const Stat3DViz& r){
    return l.my_shape_ < r.my_shape_;
  }
};


}
}

#endif
