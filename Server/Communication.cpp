#include"Communication.h"

char ERGS_NAMES[ERGS_NUM][6]
{
    "SNULL",

    "SPOFF",
    "SPOED",

    "SSTOP",
    "SSTPD",

    "SENBL",
    "SENBD",

    "SRUNN",

    "SHME1",
    "SHM1D",

    "SHME2",
    "SHM2D",

    "SH2S1",
    "SH2S2",

    "SRNST",

    "SGFWD",
    "SGBWD",
    "SGLFT",
    "SGRGT",

    "SGEX1",
    "SGEX2",

};


char ERG_NAMES[ERG_NUM][6]
{
    "GNULL",
    "GPWOF",
    "GSTOP",
    "GENBL",

    "GRUNN",

    "GHME1",// LF RM LR homing   RF LM RR maintain state
    "GHME2",// RF LM RR homing   LF RM LR maintain state
    "GH2S1",
    "GH2S2",

    "GRNST",// All homed and state will automatically switched to this one

    "GFRWD",
    "GBKWD",

    "GLEFT",
    "GRIGH",

    "GEXP1",
    "GEXP2",
};


char EMT_NAMES[EMT_NUM][6]
{
    "GCMD",
    "GSTA",
    "DATA",
    "SYSC",
    "DIMU",
    "CUCO",
};
