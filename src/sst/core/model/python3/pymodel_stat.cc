// -*- c++ -*-

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
#include "sst/core/warnmacros.h"

DISABLE_WARN_DEPRECATED_REGISTER
#include <Python.h>
REENABLE_WARNING

#include <string.h>

#include "sst/core/model/python3/pymodel.h"
#include "sst/core/model/python3/pymodel_stat.h"

#include "sst/core/sst_types.h"
#include "sst/core/configGraph.h"

using namespace SST::Core;
extern SST::Core::SSTPythonModelDefinition *gModel;


extern "C" {

StatisticId_t PyStatistic::getID()
{
    return id;
}

std::string PyStatistic::getName() {
    return getStat()->name;
}

ConfigStatistic* PyStatistic::getStat() {
    return gModel->getGraph()->findStatistic(id);
}

int PyStatistic::compare(PyStatistic *other) {
    if (id < other->id) return -1;
    else if (id > other->id ) return 1;
    else return 0;
}

static int statInit(StatisticPy_t *self, PyObject *args, PyObject *UNUSED(kwds))
{
    StatisticId_t id;
    PyObject *parent;
    // if ( !PyArg_ParseTuple(args, "Ossii", &parent, &name, &type, &slot, &id) )
    if ( !PyArg_ParseTuple(args, "Ok", &parent, &id) )
        return -1;

    PyStatistic *obj = new PyStatistic(self,id);
    self->obj = obj;

    gModel->getOutput()->verbose(CALL_INFO, 3, 0, "Creating statistic [%s]]\n", getStat((PyObject*)self)->name.c_str());

    return 0;
}


static void statDealloc(StatisticPy_t *self)
{
    if ( self->obj ) delete self->obj;
    Py_TYPE(self)->tp_free((PyObject*)self);
}



static PyObject* statAddParam(PyObject *self, PyObject *args)
{
    char* param = nullptr;
    PyObject *value = nullptr;
    if ( !PyArg_ParseTuple(args, "sO", &param, &value) )
        return nullptr;

    ConfigStatistic *c = getStat(self);
    if ( nullptr == c ) return nullptr;

    // Get the string-ized value by calling __str__ function of the
    // value object
    PyObject *vstr = PyObject_CallMethod(value, (char*)"__str__", nullptr);
    c->addParameter(param, PyUnicode_AsUTF8(vstr), true);
    Py_XDECREF(vstr);

    return PyLong_FromLong(0);
}


static PyObject* statAddParams(PyObject *self, PyObject *args)
{

    ConfigStatistic *c = getStat(self);
    if ( nullptr == c ) return nullptr;

    if ( !PyDict_Check(args) ) {
        return nullptr;
    }

    Py_ssize_t pos = 0;
    PyObject *key, *val;
    long count = 0;

    while ( PyDict_Next(args, &pos, &key, &val) ) {
        PyObject *kstr = PyObject_CallMethod(key, (char*)"__str__", nullptr);
        PyObject *vstr = PyObject_CallMethod(val, (char*)"__str__", nullptr);
        c->addParameter(PyUnicode_AsUTF8(kstr), PyUnicode_AsUTF8(vstr), true);
        Py_XDECREF(kstr);
        Py_XDECREF(vstr);
        count++;
    }
    return PyLong_FromLong(count);
}

static PyMethodDef statisticMethods[] = {
    {   "addParam",
        statAddParam, METH_VARARGS,
        "Adds a parameter(name, value)"},
    {   "addParams",
        statAddParams, METH_O,
        "Adds Multiple Parameters from a dict"},
     {   nullptr, nullptr, 0, nullptr }
};


PyTypeObject PyModel_StatType = {
    PyVarObject_HEAD_INIT(nullptr, 0)
    "sst.Statistic",           /* tp_name */
    sizeof(StatisticPy_t),     /* tp_basicsize */
    0,                         /* tp_itemsize */
    (destructor)statDealloc,/* tp_dealloc */
    0,                         /* tp_vectorcall_offset */
    nullptr,                   /* tp_getattr */
    nullptr,                   /* tp_setattr */
    nullptr,                   /* tp_as_async */
    nullptr,                   /* tp_repr */
    nullptr,                   /* tp_as_number */
    nullptr,                   /* tp_as_sequence */
    nullptr,                   /* tp_as_mapping */
    nullptr,                   /* tp_hash */
    nullptr,                   /* tp_call */
    nullptr,                   /* tp_str */
    nullptr,                   /* tp_getattro */
    nullptr,                   /* tp_setattro */
    nullptr,                   /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,        /* tp_flags */
    "SST Statistic",           /* tp_doc */
    nullptr,                   /* tp_traverse */
    nullptr,                   /* tp_clear */
    nullptr,                   /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    nullptr,                   /* tp_iter */
    nullptr,                   /* tp_iternext */
    statisticMethods,          /* tp_methods */
    nullptr,                   /* tp_members */
    nullptr,                   /* tp_getset */
    nullptr,                   /* tp_base */
    nullptr,                   /* tp_dict */
    nullptr,                   /* tp_descr_get */
    nullptr,                   /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)statInit,     /* tp_init */
    nullptr,                   /* tp_alloc */
    nullptr,                   /* tp_new */
    nullptr,                   /* tp_free */
    nullptr,                   /* tp_is_gc */
    nullptr,                   /* tp_bases */
    nullptr,                   /* tp_mro */
    nullptr,                   /* tp_cache */
    nullptr,                   /* tp_subclasses */
    nullptr,                   /* tp_weaklist */
    nullptr,                   /* tp_del */
    0,                         /* tp_version_tag */
    nullptr,                   /* tp_finalize */
};



}  /* extern C */


