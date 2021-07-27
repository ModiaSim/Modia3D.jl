"""
    applyFrameTorque!(frameApply::Object3D, torque::SVector{3,Float64}; frameCoord::Object3D)

Apply torque vector `torque` resolved in frame `frameCoord` at frame `frameApply`.

If `frameCoord` is omitted `torque` is resolved in absolute coordinates.
"""
function applyFrameTorque!(frameApply::Object3D, torque::SVector{3,Float64}; frameCoord::Object3D)
    if !isnothing(frameCoord)
        torque_abs = frameCoord.R_abs' * torque  # World_torque := R_CoordWorld^T * Coord_torque
    else
        torque_abs = torque
    end
    frameApply.t += frameApply.R_abs * torque_abs  # Apply_t := R_ApplyWorld * World_torque
    return nothing
end


"""
    applyFrameForce!(frameApply::Object3D, force::SVector{3,Float64}; frameCoord::Object3D)

Apply force vector `force` resolved in frame `frameCoord` at origin of frame `frameApply`.

If `frameCoord` is omitted `force` is resolved in absolute coordinates.
"""
function applyFrameForce!(frameApply::Object3D, force::SVector{3,Float64}; frameCoord::Object3D)
    if !isnothing(frameCoord)
        force_abs = frameCoord.R_abs' * force  # World_force := R_CoordWorld^T * Coord_force
    else
        force_abs = force
    end
    frameApply.f += frameApply.R_abs * force_abs  # Apply_f := R_ApplyWorld * World_force
    return nothing
end


"""
    applyFrameForceTorque!(frameApply::Object3D, force::SVector{3,Float64}, torque::SVector{3,Float64}; frameCoord::Object3D)

Apply force and torque vectors `force` and `torque` resolved in frame `frameCoord` at origin of frame `frameApply`.

If `frameCoord` is omitted `force` and `torque` are resolved in absolute coordinates.
"""
function applyFrameForceTorque!(frameApply::Object3D, force::SVector{3,Float64}, torque::SVector{3,Float64}; frameCoord::Object3D)
    applyFrameForce!(frameApply, force; frameCoord)
    applyFrameTorque!(frameApply, torque; frameCoord)
    return nothing
end


"""
    applyFrameTorquePair!(frameMeas::Object3D, frameOrig::Object3D, torque::SVector{3,Float64}; frameCoord::Object3D)

Apply torque vector `torque` resolved in frame `frameCoord` at origins of frames `frameMeas` (negative) and `frameOrig` (positive).

If `frameCoord` is omitted torque` is resolved in absolute coordinates.
"""
function applyFrameTorquePair!(frameMeas::Object3D, frameOrig::Object3D, torque::SVector{3,Float64}; frameCoord::Object3D)
    applyFrameTorque!(frameMeas, -torque; frameCoord)
    applyFrameTorque!(frameOrig,  torque; frameCoord)
    return nothing
end


"""
    applyFrameForcePair!(frameMeas::Object3D, frameOrig::Object3D, force::SVector{3,Float64}; frameCoord::Object3D)

Apply force vector `force` resolved in frame `frameCoord` at origins of frames `frameMeas` (negative) and `frameOrig` (positive).
In addition a compensation torque is applied at frame `frameOrig` to satisfy torque balance.

If `frameCoord` is omitted `torque` is resolved in absolute coordinates.
"""
function applyFrameForcePair!(frameMeas::Object3D, frameOrig::Object3D, force::SVector{3,Float64}; frameCoord::Object3D)
    applyFrameForce!(frameMeas, -force; frameCoord)
    applyFrameForce!(frameOrig,  force; frameCoord)
    r_OrigMeas = measFramePosition(frameMeas; frameOrig, frameCoord)
    applyFrameTorque!(frameOrig, cross(r_OrigMeas, force); frameCoord)  # Coord_t := Coord_r_OrigMeas x Coord_force
    return nothing
end


"""
    applyFrameForceTorquePair!(frameMeas::Object3D, frameOrig::Object3D, force::SVector{3,Float64}, torque::SVector{3,Float64}; frameCoord::Object3D)

Apply force vector `force` and torque vector `torque` resolved in frame `frameCoord` at origins of frames `frameMeas` (negative) and `frameOrig` (positive).
In addition a compensation torque is applied at frame `frameOrig` to satisfy torque balance.

If `frameCoord` is omitted `force` and `torque` are resolved in absolute coordinates.
"""
function applyFrameForceTorquePair!(frameMeas::Object3D, frameOrig::Object3D, force::SVector{3,Float64}; frameCoord::Object3D)
    applyFrameTorquePair!(frameMeas, frameOrig, torque; frameCoord)
    applyFrameForcePair!(frameMeas, frameOrig, force; frameCoord)
    return nothing
end
