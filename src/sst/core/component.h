// Copyright 2009-2016 Sandia Corporation. Under the terms
// of Contract DE-AC04-94AL85000 with Sandia Corporation, the U.S.
// Government retains certain rights in this software.
//
// Copyright (c) 2009-2016, Sandia Corporation
// All rights reserved.
//
// This file is part of the SST software package. For license
// information, see the LICENSE file in the top level directory of the
// distribution.

#ifndef SST_CORE_COMPONENT_H
#define SST_CORE_COMPONENT_H

#include <sst/core/sst_types.h>

#include <map>
#include <string>

#include <sst/core/baseComponent.h>


using namespace SST::Statistics;

namespace SST {
class SubComponent;

/**
 * Main component object for the simulation.
 *  All models inherit from this.
 */
class Component: public BaseComponent {
public:

    /** Constructor. Generally only called by the factory class.
        @param id Unique component ID
    */
    Component( ComponentId_t id );
    virtual ~Component();

    /** Returns unique component ID */
    inline ComponentId_t getId() const { return id; }


    /** Register that the simulation should not end until this
        component says it is OK to. Calling this function (generally
        done in Component::setup() or in component constructor)
        increments a global counter. Calls to
        Component::unregisterExit() decrements the counter. The
        simulation cannot end unless this counter reaches zero, or the
        simulation time limit is reached. This counter is synchonized
        periodically with the other nodes.

        @sa Component::unregisterExit()
    */
    bool registerExit();

    /** Indicate permission for the simulation to end. This function is
        the mirror of Component::registerExit(). It decrements the
        global counter, which, upon reaching zero, indicates that the
        simulation can terminate. @sa Component::registerExit() */
    bool unregisterExit();

    /** Register as a primary component, which allows the component to
        specifiy when it is and is not OK to end simulation.  The
        simulator will not end simulation natuarally (through use of
        the Exit object) while any primary component has specified
        primaryComponentDoNotEndSim().  However, it is still possible
        for Actions other than Exit to end simulation.  Once all
        primary components have specified
        primaryComponentOKToEndSim(), the Exit object will trigger and
        end simulation.

	This must be called during simulation wireup (i.e during the
	constructor for the component).  By default, the state of the
	primary component is set to OKToEndSim.

	If no component registers as a primary component, then the
	Exit object will not be used for that simulation and
	simulation termination must be accomplished through some other
	mechanism (e.g. --stopAt flag, or some other Action object).

        @sa Component::primaryComponentDoNotEndSim()
        @sa Component::primaryComponentOKToEndSim()
    */
    void registerAsPrimaryComponent();

    /** Tells the simulation that it should not exit.  The component
	will remain in this state until a call to
	primaryComponentOKToEndSim().

        @sa Component::registerAsPrimaryComponent()
        @sa Component::primaryComponentOKToEndSim()
    */
    void primaryComponentDoNotEndSim();

    /** Tells the simulation that it is now OK to end simulation.
	Simulation will not end until all primary components have
	called this function.

        @sa Component::registerAsPrimaryComponent()
        @sa Component::primaryComponentDoNotEndSim()
    */
    void primaryComponentOKToEndSim();


protected:
    friend class SubComponent;
    Component() {} // Unused, but previously available

    Component* getTrueComponent() final { return this; }


    // Does the statisticName exist in the ElementInfoStatistic
    virtual bool doesComponentInfoStatisticExist(const std::string &statisticName) final;
    // Return the EnableLevel for the statisticName from the ElementInfoStatistic
    virtual uint8_t getComponentInfoStatisticEnableLevel(const std::string &statisticName) final;
    // Return the Units for the statisticName from the ElementInfoStatistic
    virtual std::string getComponentInfoStatisticUnits(const std::string &statisticName) final;

private:

    /** Unique ID */
    ComponentId_t   id;

};

} //namespace SST

#endif // SST_CORE_COMPONENT_H
