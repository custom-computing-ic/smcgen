####################################################
#       Makefile ------- Thomas Chau 2013          #
#       Tested for MaxCompiler 2013.2.2            #
####################################################
# 1)    For SLiC interface only
#       If you are using MaxCompilerRT,
#       use maxfilecompile instead of sliccompile
#       and change LFLAGS -lslic to -lmaxcompilerrt
# 2)    This makefile uses common settings and
#       compilation targets defined in the root folder
# 3)    Usage
#       -----
#       Build and run simulation:  make runsim
#       Kill failed simulation:    make stopsim
#       Debug simulation:          make debug
#       Build DFE:                 make build
#       Run DFE:                   make run
#       Clean binaries:            make clean
#       Clean all generated files: make distclean


# ---- Project settings ----

WHEREISROOT=../../


# Assuming Kernel and Manager class names follow the pattern below:
PRJ=Smc
KERNEL=$(PRJ)Kernel
MANAGER=$(PRJ)Manager

# Application specific compile flags
OTHERCFLAGS=-I../src/dSFMT-src-2.2.1 -DDSFMT_MEXP=521 -DHAVE_SSE2=1 

# Runtime arguments
ARGS=../src/sensor.txt ../src/control.txt 


# Using known hardware settings
include $(WHEREISROOT)/Makefile.Max3.hardware


# Using common compillation targets and compiler settings
include $(WHEREISROOT)/Makefile.common
