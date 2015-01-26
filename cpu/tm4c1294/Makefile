# define the module that is build
MODULE = cpu

# add a list of subdirectories, that should also be build
DIRS = periph $(CORTEX_M4_COMMON)

PART=TM4C1294NCPDT
CFLAGS +=-DTARGET_IS_BLIZZARD_RB1 -DPART_${PART}

include $(RIOTBASE)/Makefile.base
