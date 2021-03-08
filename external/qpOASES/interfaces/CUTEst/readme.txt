CUTEst interface for qpOASES
by Dennis Janka <dennis.janka@iwr.uni.heidelberg.de>

====================================================

0.) Files included:

    - qpoasesCutest.cpp
    - Makefile
    - makeprob
    - readme.txt

1.) Download and install CUTEst from

    http://ccpforge.cse.rl.ac.uk/gf/project/cutest/wiki/

    Make sure all environment variables are set as instructed.
    (in particular $CUTEST, $MYARCH and $MASTSIF)

2.) To decode and compile a problem, type:

    ./makeprob <problemname>

    For problems that come in different sizes (e.g. NCVXQP[1-9]) the size
    parameter ( N=<size> ) may be passed as a second argument:

    ./makeprob <problemname> <size>

    This calls "sifdecoder" to decode the .sif file and creates a shared
    library libprob.so in prob/ that will be linked against the
    qpOASES CUTEst interface.

3.) Finally, call "make" to compile and link qpoasesCutest.
    You may have to set the correct paths for the qpOASES library and
    header files in the makefile.

4.) The command ./qpoasesCutest runs qpOASES with the latest compiled
    .sif problem. For solving further problems, only the appropriate
    "./makeprob" call is required.
