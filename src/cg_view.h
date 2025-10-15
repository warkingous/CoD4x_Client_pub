#pragma once

#include "cg_shared.h"  // cg_t, trace_t, math helpers, etc.
#include "q_shared.h"   // cvar_t

// Third-person camera with optional spectator free-look orbit.
void __cdecl CG_OffsetThirdPersonView(cg_t* cgameGlob);