// Copyright (C) Geom Software e.U, Bernhard Kornberger, Graz/Austria
//
// This file is part of the Fade2D library. The student license is free
// of charge and covers personal non-commercial research. Licensees
// holding a commercial license may use this file in accordance with
// the Commercial License Agreement.
//
// This software is provided AS IS with NO WARRANTY OF ANY KIND,
// INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE.
//
// Please contact the author if any conditions of this licensing are
// not clear to you.
//
// Author: Bernhard Kornberger, bkorn (at) geom.at
// http://www.geom.at

/// @file MsgBase.h
#pragma once
#include "common.h"


#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

enum MsgType
{
	MSG_PROGRESS,
	MSG_WARNING
};

/**  \brief MsgBase, a base class for message subscriber classes
*
* MsgBase is a base class from which message subscriber classes (for
* example widgets, progress bars, ...) can be derived which then
* receive messages (progress, warnings, ...) from Fade.
*
* \sa http://www.geom.at/progress-bar/
*
*/
class CLASS_DECLSPEC MsgBase
{
public:
	MsgBase(){};
	virtual ~MsgBase(){}


/** \brief update
*
* This method must be defined in derived classes. It is automatically
* called everytime Fade has a message of type \p msgType.
*
*/
	virtual void update(MsgType msgType,const char* s,double d)=0;
};


} // Namespace

