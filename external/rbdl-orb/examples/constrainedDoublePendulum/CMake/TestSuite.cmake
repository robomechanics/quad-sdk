# File:   TestSuite.cmake
# Author: Christian Hoffmann
# Date:   2009
#
# This file is part of the MUSCOD suite. The MUSCOD suite is proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
#
# Copyright (C) 200*
#
####################################################################################################
#
# CTest test cases
#
####################################################################################################

####################################################################################################
#### DEFINE TEST PROBLEMS
####################################################################################################
SET( MUSCOD_TEST_COMMAND ${MUSCOD_EXECUTABLE} )
SET( MUSCOD_TEST_ARGS "" )

SET( MUSCOD_TEST_PROBLEMS
	academy
	soccer
	batchdist
	batchdistrob
	batchdistRobEx
	batchdistUT
	brac
	brgr1
	brgr2
	ccbat
	ccrane
	chain1d
	chain
	container_bridge
	cstr
	cstr_est
	dcbat1
	dcbat2
	dcbat3
	eason
	eocar1
	eocar2
	extrosen
	fedbat1
	fedbat1m
	fedbat2
	fedbat2m
	fedbat3
	fedbat3m
	fedbat4
	freuden
	freudenstein
	ftlos1
	ftlos2
	ftlos2_nlp
	ftlos3
	ftlos4
	hang
	helical
	hydroscal
	kite
	lbat
	macro1
	macro2
	macro3
	macro4
	macro5
	maratos
	nlbat
	nlp1
	nlp2
	nlp3
	nlp4
	nmpc1
	ocean3a
	ocean3b
	optcar_lego
	orbit
	pbreac
	powerkite
	qlin
	reactdiff
	reentry
	rob2link
	rob2link_flex
	rob3link
	rob3link_flex
	rosen
	singular
	skeleton
	smb
	stp1
	stp1_gn
	stp2a
	stp2b
	stp2c
	stp2d
	stp2e
	stp3
	stp3_gn
	stp4a
	stp4a_gn
	stp4b
	swtball
	tocar1
	tocar2
	tocar3
	tolin1
	tolin2
	tolos1
	tolos2
	tolos3
	tolos4
	twobat1
	twobat2
	unload1
	unload2
	vdpol
	watson
	wood
	oven
	energy_f77
	energy_lsq_f77
	inventory_f77
	rocket_f77
	stp3_f77
)


####################################################################################################
#### RUN TESTS
####################################################################################################
SET (regexp3 "convergence achieved")

FOREACH( MUSCOD_TEST_PROBLEM ${MUSCOD_TEST_PROBLEMS} )
	SET( TEST_NAME "${MUSCOD_TEST_PROBLEM}_test" )
	ADD_TEST( ${TEST_NAME} ${MUSCOD_TEST_COMMAND} ${MUSCOD_TEST_ARGS} ${MUSCOD_TEST_PROBLEM} )
# 	LOG( "ADDED TEST: ${TEST_NAME}" )
	SET_TESTS_PROPERTIES( ${TEST_NAME} PROPERTIES PASS_REGULAR_EXPRESSION "${regexp3}" )
ENDFOREACH( MUSCOD_TEST_PROBLEM )
