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

   virtual bool isBox() const = 0;
   virtual bool isLine() const = 0;
};
struct Box3D : Shape3D {
    Box3D(double x_origin, double y_origin, double z_origin,
          double x_extent, double y_extent, double z_extent) :
      Shape3D(Shape::Box),
      x_origin_(x_origin), y_origin_(y_origin), z_origin_(z_origin),
      x_extent_(x_extent), y_extent_(y_extent), z_extent_(z_extent)
    {
    }
    bool isLine() const override {
        return false;
    }

    bool isBox() const override {
        return true;
    }

    double x_origin_;
    double y_origin_;
    double z_origin_;
    double x_extent_;
    double y_extent_;
    double z_extent_;
};

struct Line3D : Shape3D {
    Line3D(double x_origin, double y_origin, double z_origin,
           double x_end, double y_end, double z_end) :
        Shape3D(Shape::Line)
    {
    }

    bool isLine() const override {
        return true;
    }

    bool isBox() const override {
        return false;
    }

};

struct Stat3DViz {

    Stat3DViz(Params& params){
        std::string shape = params.find<std::string>("shape", "");
        std::cout << " Stat3DViz creation of shape : " << shape << std::endl;
        if (shape == cubeKey) {
            double x_origin = params.find<double>("x_origin", "");
            double y_origin = params.find<double>("y_origin", "");
            double z_origin = params.find<double>("z_origin", "");
            double x_extent = params.find<double>("x_extent", "");
            double y_extent = params.find<double>("y_extent", "");
            double z_extent = params.find<double>("z_extent", "");
            std::cout << " origin : " << x_origin << " "<< y_origin << " "<< z_origin << std::endl;
            std::cout << " extent : " << x_extent << " "<< y_extent << " "<< z_extent << std::endl;
            my_shape_ = new Box3D(x_origin, y_origin, z_origin, x_extent, y_extent, z_extent);
        }
        else if (shape == lineKey) {
//            my_shape_ = new Line3D();

        }
        else {
           // TODO Error : type unrecognized
           std::cout << " Stat3DViz isn't recognized " <<  std::endl;
        }
    }

    void setName(std::string name) {
        name_ = name;
    }

    std::string name_;
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
