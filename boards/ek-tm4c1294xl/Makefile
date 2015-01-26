# tell the Makefile.base which module to build
MODULE = $(BOARD)_base

PART=TM4C1294NCPDT
CFLAGS +=-DTARGET_IS_BLIZZARD_RB1 -DPART_${PART}

include $(RIOTBASE)/Makefile.base
