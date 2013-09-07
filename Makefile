# Target specific macros
TARGET = robolt

TARGET_SOURCES = \
    balancer_param.c \
    hook.c \
    main.c \
    main_table.c \
    background.c \
    tasksync_table.c \
    linetrace.c \
    lookup.c \
    lookup_table.c \

TOPPERS_OSEK_OIL_SOURCE = ./robolt.oil

USER_INC_PATH= ../../ecrobot/nxtway_gs_balancer
USER_LIB = nxtway_gs_balancer

# Don't modify below part
O_PATH ?= build
include ../../ecrobot/ecrobot.mak
