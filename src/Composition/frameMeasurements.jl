"""
    R = measFrameRotation(frameMeas::Object3D; frameOrig::Object3D)

Return relative rotation matrix `R` from frame `frameOrig` into frame `frameMeas`.

If `frameOrig` is omitted `R` represents the absolute rotation of `frameMeas`.
"""
function measFrameRotation(frameMeas::Object3D{F}; frameOrig::Union{Object3D{F}, Nothing}=nothing) where F <: Modia3D.VarFloatType
    R_MeasOrig = copy(frameMeas.R_abs)  # R_MeasOrig := R_MeasWorld
    if !isnothing(frameOrig)
        R_MeasOrig = R_MeasOrig * frameOrig.R_abs'  # R_MeasOrig := R_MeasOrig * R_OrigWorld^T
    end
    return SMatrix{3,3,F,9}(R_MeasOrig)
end


"""
    r = measFramePosition(frameMeas::Object3D; frameOrig::Object3D, frameCoord::Object3D)

Return relative position vector `r` from frame `frameOrig` to frame `frameMeas` resolved in frame `frameCoord`.

If `frameOrig` is omitted `r` represents the absolute position of `frameMeas`.

If `frameCoord` is omitted `r` is resolved in absolute coordinates.
"""
function measFramePosition(frameMeas::Object3D{F}; frameOrig::Union{Object3D{F}, Nothing}=nothing, frameCoord::Union{Object3D{F}, Nothing}=nothing) where F <: Modia3D.VarFloatType
    r_OrigMeas = copy(frameMeas.r_abs)  # World_r_OrigMeas := World_r_WorldMeas
    if !isnothing(frameOrig)
        r_OrigMeas = r_OrigMeas - frameOrig.r_abs  # World_r_OrigMeas := World_r_OrigMeas - World_r_WorldOrig
    end
    if !isnothing(frameCoord)
        r_OrigMeas = frameCoord.R_abs * r_OrigMeas  # Coord_r_OrigMeas := R_CoordWorld * World_r_OrigMeas
    end
    return SVector{3,F}(r_OrigMeas)
end


"""
    (d, n) = measFrameDistance(frameMeas::Object3D; frameOrig::Object3D)

Return distance `d` and normalized direction vector `n` from origin of frame `frameOrig` to origin of frame `frameMeas`.

If `frameOrig` is omitted `d` and  `n` represent the distance from world frame to `frameMeas`.
"""
function measFrameDistance(frameMeas::Object3D{F}; frameOrig::Union{Object3D{F}, Nothing}=nothing) where F <: Modia3D.VarFloatType
    r_OrigMeas = measFramePosition(frameMeas; frameOrig=frameOrig)
    d_OrigMeas = norm(r_OrigMeas)
    if d_OrigMeas > 1.0e-32
        n_OrigMeas = r_OrigMeas / d_OrigMeas
    else
        n_OrigMeas = Modia3D.ZeroVector3D(F)
    end
    return F(d_OrigMeas), SVector{3,F}(n_OrigMeas)
end


"""
    w = measFrameRotVelocity(frameMeas::Object3D; frameOrig::Object3D, frameCoord::Object3D)

Return relative rotational velocity vector `w` from frame `frameOrig` to frame `frameMeas` resolved in frame `frameCoord`.

If `frameOrig` is omitted `w` represents the absolute rotational velocity of `frameMeas`.

If `frameCoord` is omitted `w` is resolved in absolute coordinates.
"""
function measFrameRotVelocity(frameMeas::Object3D{F}; frameOrig::Union{Object3D{F}, Nothing}=nothing, frameCoord::Union{Object3D{F}, Nothing}=nothing) where F <: Modia3D.VarFloatType
    w_OrigMeas = frameMeas.R_abs' * copy(frameMeas.w)  # World_w_WorldMeas := R_MeasWorld^T * Meas_w_WorldMeas
    if !isnothing(frameOrig)
        w_OrigMeas = w_OrigMeas - (frameOrig.R_abs' * frameOrig.w)  # World_w_OrigMeas := World_w_WorldMeas - R_OrigWorld^T * Orig_w_WorldOrig
    end
    if !isnothing(frameCoord)
        w_OrigMeas = frameCoord.R_abs * w_OrigMeas  # Coord_w_OrigMeas := R_CoordWorld * World_w_OrigMeas
    end
    return SVector{3,F}(w_OrigMeas)
end


"""
    v = measFrameTransVelocity(frameMeas::Object3D; frameOrig::Object3D, frameCoord::Object3D, frameObsrv::Object3D)

Return relative translational velocity vector `v` from frame `frameOrig` to frame `frameMeas` resolved in frame `frameCoord` and observed in frame `frameObsrv`.

If `frameOrig` is omitted `v` represents the absolute translational velocity of `frameMeas`.

If `frameCoord` is omitted `v` is resolved in absolute coordinates.

If `frameObsrv` is omitted `v` is observed in world frame.
"""
function measFrameTransVelocity(frameMeas::Object3D{F}; frameOrig::Union{Object3D{F}, Nothing}=nothing, frameCoord::Union{Object3D{F}, Nothing}=nothing, frameObsrv::Union{Object3D{F}, Nothing}=nothing) where F <: Modia3D.VarFloatType
    v_OrigMeas = copy(frameMeas.v0)  # World_v_OrigMeas := World_v_WorldMeas
    if !isnothing(frameOrig)
        v_OrigMeas = v_OrigMeas - frameOrig.v0  # World_v_OrigMeas := World_v_WorldMeas - World_v_WorldOrig
    end
    if !isnothing(frameObsrv)
        r_OrigMeas = measFramePosition(frameMeas; frameOrig=frameOrig)
        w_WorldObsrv = measFrameRotVelocity(frameObsrv)
        v_OrigMeas = v_OrigMeas - cross(w_WorldObsrv, r_OrigMeas)  # World_v_OrigMeas := World_v_OrigMeas - World_w_WorldObsrv x World_r_OrigMeas
    end
    if !isnothing(frameCoord)
        v_OrigMeas = frameCoord.R_abs * v_OrigMeas  # Coord_v_OrigMeas := R_CoordWorld * World_v_OrigMeas
    end
    return SVector{3,F}(v_OrigMeas)
end


"""
    dd = measFrameDistVelocity(frameMeas::Object3D; frameOrig::Object3D)

Return distance velocity `dd` between origin of frame `frameOrig` and origin of frame `frameMeas`.

If `frameOrig` is omitted `dd` represents the distance velocity between world frame and `frameMeas`.
"""
function measFrameDistVelocity(frameMeas::Object3D{F}; frameOrig::Union{Object3D{F}, Nothing}=nothing) where F <: Modia3D.VarFloatType
    (d_OrigMeas, n_OrigMeas) = measFrameDistance(frameMeas; frameOrig=frameOrig)
    v_OrigMeas = measFrameTransVelocity(frameMeas; frameOrig=frameOrig)
    dd_OrigMeas = dot(n_OrigMeas, v_OrigMeas)
    return F(dd_OrigMeas)
end


"""
    wd = measFrameRotAcceleration(frameMeas::Object3D; frameOrig::Object3D, frameCoord::Object3D, frameObsrv::Object3D)

Return relative rotational acceleration vector `wd` from frame `frameOrig` to frame `frameMeas` resolved in frame `frameCoord` and observed in frame `frameObsrv`.

If `frameOrig` is omitted `wd` represents the absolute rotational acceleration of `frameMeas`.

If `frameCoord` is omitted `wd` is resolved in absolute coordinates.

If `frameObsrv` is omitted `wd` is observed in world frame.
"""
function measFrameRotAcceleration(frameMeas::Object3D{F}; frameOrig::Union{Object3D{F}, Nothing}=nothing, frameCoord::Union{Object3D{F}, Nothing}=nothing, frameObsrv::Union{Object3D{F}, Nothing}=nothing) where F <: Modia3D.VarFloatType
    wd_OrigMeas = frameMeas.R_abs' * copy(frameMeas.z)  # World_wd_WorldMeas := R_MeasWorld^T * Meas_wd_WorldMeas
    if !isnothing(frameOrig)
        wd_OrigMeas = wd_OrigMeas - (frame.Orig.R_abs' * frameOrig.z)  # World_wd_OrigMeas := World_wd_WorldMeas - R_OrigWorld^T * Orig_wd_WorldOrig
    end
    if !isnothing(frameObsrv)
        w_OrigMeas = measFrameRotVelocity(frameMeas; frameOrig=frameOrig)
        w_WorldObsrv = measFrameRotVelocity(frameObsrv)
        wd_OrigMeas = wd_OrigMeas - cross(w_WorldObsrv, w_OrigMeas)  # World_wd_OrigMeas := World_wd_OrigMeas - World_w_WorldObsrv x World_w_OrigMeas
    end
    if !isnothing(frameCoord)
        wd_OrigMeas = frameCoord.R_abs * wd_OrigMeas  # Coord_wd_OrigMeas := R_CoordWorld * World_wd_OrigMeas
    end
    return SVector{3,F}(wd_OrigMeas)
end


"""
    a = measFrameTransAcceleration(frameMeas::Object3D; frameOrig::Object3D, frameCoord::Object3D, frameObsrv::Object3D)

Return relative translational acceleration vector `a` from frame `frameOrig` to frame `frameMeas` resolved in frame `frameCoord` and observed in frame `frameObsrv`.

If `frameOrig` is omitted `a` represents the absolute translational velocity of `frameMeas`.

If `frameCoord` is omitted `a` is resolved in absolute coordinates.

If `frameObsrv` is omitted `a` is observed in world frame.
"""
function measFrameTransAcceleration(frameMeas::Object3D{F}; frameOrig::Union{Object3D{F}, Nothing}=nothing, frameCoord::Union{Object3D{F}, Nothing}=nothing, frameObsrv::Union{Object3D{F}, Nothing}=nothing) where F <: Modia3D.VarFloatType
    a_OrigMeas = copy(frameMeas.a0)  # World_a_OrigMeas := World_a_WorldMeas
    if !isnothing(frameOrig)
        a_OrigMeas = a_OrigMeas - frameOrig.a0  # World_a_OrigMeas := World_a_WorldMeas - World_a_WorldOrig
    end
    if !isnothing(frameObsrv)
        r_OrigMeas = measFramePosition(frameMeas; frameOrig=frameOrig)
        v_OrigMeas = measFrameTransVelocity(frameMeas; frameOrig=frameOrig)
        w_WorldObsrv = measFrameRotVelocity(frameObsrv)
        wd_WorldObsrv = measFrameRotAcceleration(frameObsrv)
        a_OrigMeas = a_OrigMeas                                              # World_a_OrigMeas := World_a_OrigMeas
                     - 2.0*cross(w_WorldObsrv, v_OrigMeas)                   # - 2*World_w_WorldObsrv x World_v_OrigMeas
                     + cross(w_WorldObsrv, cross(w_WorldObsrv, r_OrigMeas))  # + World_w_WorldObsrv x (World_w_WorldObsrv x World_r_OrigMeas)
                     - cross(wd_WorldObsrv, r_OrigMeas)                      # - World_wd_WorldObsrv x World_r_OrigMeas
    end
    if !isnothing(frameCoord)
        a_OrigMeas = frameCoord.R_abs * a_OrigMeas  # Coord_a_OrigMeas := R_CoordWorld * World_a_OrigMeas
    end
    return SVector{3,F}(a_OrigMeas)
end


"""
    ddd = measFrameDistAcceleration(frameMeas::Object3D; frameOrig::Object3D)

Return distance acceleration `ddd` between origin of frame `frameOrig` and origin of frame `frameMeas`.

If `frameOrig` is omitted `ddd` represents the distance acceleration between world frame and `frameMeas`.
"""
function measFrameDistAcceleration(frameMeas::Object3D{F}; frameOrig::Union{Object3D{F}, Nothing}=nothing) where F <: Modia3D.VarFloatType
    # (n * v)d = n*vd + nd*v
    #          = n*a + [r/d]d*v
    #          = n*a + [(v*d - r*dd)/(d*d)]*v
    #          = n*a + v*v/d - (r*v)*(n*v)/(d*d)
    #          = n*a + v*v/d - (n*v)*(n*v)/d
    #          = n*a + (v*v - dd*dd)/d
    (d_OrigMeas, n_OrigMeas) = measFrameDistance(frameMeas; frameOrig=frameOrig)
    v_OrigMeas = measFrameTransVelocity(frameMeas; frameOrig=frameOrig)
    a_OrigMeas = measFrameTransAcceleration(frameMeas; frameOrig=frameOrig)
    dd_OrigMeas = dot(n_OrigMeas, v_OrigMeas)
    ddd_OrigMeas = dot(n_OrigMeas, a_OrigMeas)
    if d_OrigMeas > 1.0e-32
        ddd_OrigMeas += (dot(v_OrigMeas, v_OrigMeas) - dd_OrigMeas*dd_OrigMeas) / d_OrigMeas
    end
    return F(ddd_OrigMeas)
end
