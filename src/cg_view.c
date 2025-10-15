// cg_view.c
#include "cg_view.h"
#include "q_shared.h"
#include "client.h"
#include <math.h>

#ifndef TP_CONTENT_MASK
// Content mask used for third-person camera collision traces.
// You can extend this if you want the camera to collide with more/less things.
#define TP_CONTENT_MASK 0x811
#endif

// Engine trace prototype (matches the game's internal CG_Trace signature).
// We call this via a fixed address (s_CG_Trace), so make sure the address is correct for your build.
typedef void (__cdecl *CG_Trace_t)(
    trace_t* results,
    float* start, float* mins, float* maxs, float* end,
    int32_t passEntityNum, int32_t contentMask,
    int /*locational*/, int /*staticModels*/
);
static CG_Trace_t s_CG_Trace = (CG_Trace_t)0x00459EF0;

// Small capsule extents for the camera trace to avoid clipping through tight geometry.
static const float TP_MINS[3] = { -4.0f, -4.0f, -4.0f };
static const float TP_MAXS[3] = {  4.0f,  4.0f,  4.0f };

// --- DVars we read for TP tuning ---
static cvar_t *dvar_tpAngle = NULL; // cg_thirdPersonAngle (offset around the player)
static cvar_t *dvar_tpRange = NULL; // cg_thirdPersonRange (distance behind the player)

// Local clamp: returns v clamped to [lo, hi].
static ID_INLINE float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Late-bind the dvars so we don't assume registration order.
static ID_INLINE void TP_LazyInit(void)
{
    if (!dvar_tpAngle) dvar_tpAngle = Cvar_FindVar("cg_thirdPersonAngle");
    if (!dvar_tpRange) dvar_tpRange = Cvar_FindVar("cg_thirdPersonRange");
    // If not found (e.g., mod didn’t register them), fall back to a malleable container.
    if (!dvar_tpAngle) dvar_tpAngle = Cvar_FindMalleable("cg_thirdPersonAngle");
    if (!dvar_tpRange) dvar_tpRange = Cvar_FindMalleable("cg_thirdPersonRange");
}

// Simple 3D linear interpolation helper.
static ID_INLINE void V3_Lerp(const float* a, const float* b, float t, float* o)
{
    o[0] = a[0] + t * (b[0] - a[0]);
    o[1] = a[1] + t * (b[1] - a[1]);
    o[2] = a[2] + t * (b[2] - a[2]);
}

// Read third-person yaw offset from dvar. Fallback to 30 only if the dvar truly doesn't exist.
static ID_INLINE float TP_Angle(void)
{
    TP_LazyInit();
    if (dvar_tpAngle) {
        if (dvar_tpAngle->modified) {
            Com_Printf(CON_CHANNEL_CLIENT, "^3TP:^7 angle=%.1f\n", dvar_tpAngle->value);
            dvar_tpAngle->modified = qfalse;
        }
        return dvar_tpAngle->value;
    }
    return 0.0f; // emergency fallback only
}

// Read third-person range from dvar. If user sets <= 0, fall back to the dvar’s reset value, then to 64.
static ID_INLINE float TP_Range(void)
{
    TP_LazyInit();
    if (dvar_tpRange) {
        if (dvar_tpRange->modified) {
            Com_Printf(CON_CHANNEL_CLIENT, "^3TP:^7 range=%.1f\n", dvar_tpRange->value);
            dvar_tpRange->modified = qfalse;
        }
        float v = dvar_tpRange->value;
        if (v <= 0.0f) {
            float defv = dvar_tpRange->resetFloatval;
            if (defv <= 0.0f) defv = 64.0f;
            return defv;
        }
        return v;
    }
    return 128.0f; // emergency fallback only
}

// Thin wrapper over the engine's CG_Trace to do a capsule sweep for the camera.
static ID_INLINE void CG_TraceCapsule(
    trace_t *results,
    const float *start,
    const float *mins,
    const float *maxs,
    const float *end,
    int32_t passEntityNum,
    int32_t contentMask)
{
    if (!results || !start || !mins || !maxs || !end) {
        if (results) Com_Memset(results, 0, sizeof(*results));
        return;
    }

    s_CG_Trace(
        results,
        (float*)start, (float*)mins, (float*)maxs, (float*)end,
        passEntityNum,
        contentMask,
        0, /* locational: off */
        0  /* staticModels: off */
    );
}

// --- Access to current usercmd for mouse deltas (orbit accumulation) ---
static ID_INLINE int GetCurrentCmdNumber(void) { return cl.cmdNumber; }
static ID_INLINE qboolean GetUserCmd(int cmdNumber, usercmd_t *out)
{
    if (cmdNumber > cl.cmdNumber)        return qfalse;
    if (cmdNumber <= cl.cmdNumber - 128) return qfalse; // ring buffer range check
    *out = cl.cmds[cmdNumber & 127];
    return qtrue;
}

// ---------- Third-person camera with optional spectator free-look orbit ----------
void __cdecl CG_OffsetThirdPersonView(cg_t *cgameGlob)
{
    float  scale, v2, v3, v4, focusDist;
    vec3_t right, view, forward, focusAngles, up, viewAngles, focusPoint;

    // Persistent state for orbit freelook (survives between frames).
    static int      s_init = 0;
    static int32_t  s_lastPitch32 = 0, s_lastYaw32 = 0;
    static float    s_orbitPitchDeg = 0.0f, s_orbitYawDeg = 0.0f;
    static int      s_lastSpectatedClient = -1;
    static float    s_seedYawDeg = 0.0f;        // base yaw when freelook uses CS-style lock
    static float    s_seedBasePitchDeg = 0.0f;  // base (half) pitch when freelook uses CS-style lock

    // Move camera origin to eye height (matches vanilla behavior).
    cgameGlob->refdef.vieworg[2] += cgameGlob->predictedPlayerState.viewHeightCurrent;

    // Copy current view angles into locals we can modify.
    VectorCopy(cgameGlob->refdefViewAngles, viewAngles);
    VectorCopy(cgameGlob->refdefViewAngles, focusAngles);

    // During death/killcam, vanilla locks yaw to a stored value.
    if (cgameGlob->predictedPlayerState.pm_type >= PM_DEAD) {
        const float deadYaw = (float)cgameGlob->predictedPlayerState.stats[1];
        focusAngles[YAW] = deadYaw;
        viewAngles[YAW]  = deadYaw;
    }

    // Focus pitch cap (vanilla does this before computing focus point).
    if (focusAngles[PITCH] > 45.0f) focusAngles[PITCH] = 45.0f;

    // Focus point used by vanilla to compute pitch aiming later.
    AngleVectors(focusAngles, forward, NULL, NULL);
    VectorMA(cgameGlob->refdef.vieworg, 512.0f, forward, focusPoint);

    // Start from current view origin and lift slightly so camera sits a bit above.
    VectorCopy(cgameGlob->refdef.vieworg, view);
    view[2] += 8.0f;

    // “CS-style”: when true, we keep the camera's base angles locked (don’t follow the player yaw/pitch).
    // Dvar semantics we use here:
    //   cg_thirdPersonFreeLookCS = 0  => follow player (baseYaw/basePitch come from player)
    //   cg_thirdPersonFreeLookCS = 1  => lock base to seeds (baseYaw/basePitch come from s_seed*)
    const qboolean followPlayer = !Cvar_GetBool("cg_thirdPersonFreeLookCS");

    // Vanilla applies half pitch for third-person transform (lower tilt to avoid over-the-shoulder extremes).
    float basePitch = viewAngles[PITCH] * 0.5f;
    float baseYaw   = viewAngles[YAW];

    // Snapshot info for “who we’re spectating” — used to reset orbit when target changes.
    const playerState_t *ps  = &cgameGlob->predictedPlayerState;
    const snapshot_t    *nxt = cgameGlob->nextSnap;
    const int meIdx  = ps ? ps->clientNum : -1;
    const int tgtIdx = nxt ? nxt->ps.clientNum : meIdx;

    // Master toggle: if true, we accumulate mouse deltas and orbit the camera around the base angles.
    qboolean freelook = Cvar_GetBool("cg_thirdPersonFreeLook") ? qtrue : qfalse;

    // Disable freelook when you are dead (and not spectating someone else) or when view is hard-locked.
    const qboolean selfDead        = (ps && ps->pm_type >= PM_DEAD) ? qtrue : qfalse;
    const qboolean spectatingOther = (nxt && (tgtIdx != meIdx)) ? qtrue : qfalse;
    const qboolean viewIsLocked    = (ps && ps->viewlocked != PLAYERVIEWLOCK_NONE) ? qtrue : qfalse;

    if ((selfDead && !spectatingOther) || viewIsLocked) {
        if (freelook) { s_orbitPitchDeg = 0.0f; s_orbitYawDeg = 0.0f; }
        freelook = qfalse;
    }

    // When spectated client changes, reset the orbit state and seed the CS-style base.
    if (!s_init || s_lastSpectatedClient != tgtIdx) {
        const int  currNum = GetCurrentCmdNumber();
        usercmd_t  ucmd; Com_Memset(&ucmd, 0, sizeof(ucmd));
        GetUserCmd(currNum, &ucmd);

        s_lastPitch32 = ucmd.angles[PITCH];
        s_lastYaw32   = ucmd.angles[YAW];
        s_orbitPitchDeg = 0.0f;
        s_orbitYawDeg   = 0.0f;
        s_lastSpectatedClient = tgtIdx;
        s_init = 1;

        // Seed base for CS-style mode so the orbit starts from the current camera orientation.
        s_seedYawDeg       = viewAngles[YAW];
        s_seedBasePitchDeg = (viewAngles[PITCH] * 0.5f);
    }

    if (freelook) {
        // CS-style: keep the base fixed to the seed instead of following the spectated player's current angles.
        if (!followPlayer) {
            baseYaw   = s_seedYawDeg;
            basePitch = s_seedBasePitchDeg;
        }

        // Accumulate mouse deltas from usercmd ring buffer (already includes invert/m_pitch handling at input).
        const int curr = GetCurrentCmdNumber();
        usercmd_t u0; Com_Memset(&u0, 0, sizeof(u0));
        usercmd_t u1; Com_Memset(&u1, 0, sizeof(u1));
        const qboolean ok1 = GetUserCmd(curr,   &u1);
        const qboolean ok0 = GetUserCmd(curr-1, &u0);

        int dPitch16 = (ok1 && ok0) ? ((u1.angles[PITCH]-u0.angles[PITCH]) & 0xFFFF) : 0;
        int dYaw16   = (ok1 && ok0) ? ((u1.angles[YAW]  -u0.angles[YAW])   & 0xFFFF) : 0;
        if (dPitch16 > 32767) dPitch16 -= 65536;
        if (dYaw16   > 32767) dYaw16   -= 65536;

        s_orbitPitchDeg += (float)dPitch16 * (360.0f/65536.0f);
        s_orbitYawDeg   += (float)dYaw16   * (360.0f/65536.0f);

        // Keep orbit within sane limits.
        s_orbitPitchDeg = clampf(s_orbitPitchDeg, -80.0f, 60.0f);
        if (s_orbitYawDeg > 180.0f)  s_orbitYawDeg -= 360.0f;
        if (s_orbitYawDeg < -180.0f) s_orbitYawDeg += 360.0f;

        // Final camera orientation from base + orbit deltas.
        viewAngles[PITCH] = basePitch + s_orbitPitchDeg;
        viewAngles[YAW]   = baseYaw   + s_orbitYawDeg;

        // Compute the desired third-person camera position.
        AngleVectors(viewAngles, forward, right, up);
        scale = -TP_Range();
        vec3_t desired;
        VectorMA(view, scale, forward, desired);

        // Resolve collisions between current origin and desired target.
        ThirdPersonViewTrace(cgameGlob, cgameGlob->refdef.vieworg, desired,
                             TP_CONTENT_MASK, cgameGlob->refdef.vieworg);

        // Apply final orientation.
        VectorCopy(viewAngles, cgameGlob->refdefViewAngles);
        AnglesToAxis(viewAngles, cgameGlob->refdef.viewaxis);
        return;
    }

    // ----- Vanilla (no freelook): camera sits behind the player at a fixed angle and range. -----
    viewAngles[PITCH] = basePitch;
    viewAngles[YAW]   = baseYaw - TP_Angle();

    AngleVectors(viewAngles, forward, right, up);
    scale = -TP_Range();
    vec3_t desired;
    VectorMA(view, scale, forward, desired);

    // Resolve collisions for vanilla camera too.
    ThirdPersonViewTrace(cgameGlob, cgameGlob->refdef.vieworg, desired, TP_CONTENT_MASK, cgameGlob->refdef.vieworg);

    // Recompute pitch so we look at the focus point (matches vanilla behavior).
    VectorSubtract(focusPoint, cgameGlob->refdef.vieworg, focusPoint);
    v4 = focusPoint[1]*focusPoint[1] + focusPoint[0]*focusPoint[0];
    v3 = sqrtf(v4);
    focusDist = (v3 < 1.0f) ? 1.0f : v3;
    v2 = atan2f(focusPoint[2], focusDist);
    viewAngles[PITCH] = -RAD2DEG(v2);

    // Apply final orientation.
    AnglesToAxis(viewAngles, cgameGlob->refdef.viewaxis);
}
