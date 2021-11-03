
"""
    force = Bushing(; obj1, obj2,
        nominalForce = Modia3D.ZeroVector3D,
        stiffness    = Modia3D.ZeroVector3D,
        damping      = Modia3D.ZeroVector3D )

Return a `force` acting as bushing between `obj1::`[`Object3D`](@ref) and
`obj2::`[`Object3D`](@ref). Vectors `stiffness`, `damping` and
`nominalForce` define the stiffness, damping and nominal force values in
x, y and z direction of `obj1`. The orientation of `obj2` does not
influence the resulting forces.
"""
mutable struct Bushing <: Modia3D.AbstractForceElement

    obj1::Object3D
    obj2::Object3D

    nominalForce::SVector{3,Float64}
    stiffness::SVector{3,Float64}
    damping::SVector{3,Float64}

    function Bushing(; obj1::Object3D,
                       obj2::Object3D,
                       nominalForce::AbstractVector = Modia3D.ZeroVector3D,
                       stiffness::AbstractVector = Modia3D.ZeroVector3D,
                       damping::AbstractVector = Modia3D.ZeroVector3D)

        nomForce = Modia3D.convertAndStripUnit(SVector{3,Float64}, u"N"    , nominalForce)
        stiff    = Modia3D.convertAndStripUnit(SVector{3,Float64}, u"N/m"  , stiffness)
        damp     = Modia3D.convertAndStripUnit(SVector{3,Float64}, u"N*s/m", damping)

        return new(obj1, obj2, nomForce, stiff, damp)

    end
end


function initializeForceElement(force::Bushing)
    force.obj1.hasForceElement = true
    force.obj2.hasForceElement = true
    return nothing
end

function evaluateForceElement(force::Bushing)
    r12 = measFramePosition(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1)
    v12 = measFrameTransVelocity(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1, frameObsrv=force.obj1)

    f12 = force.stiffness .* r12 + force.damping .* v12 + force.nominalForce

    applyFrameForcePair!(force.obj2, force.obj1, f12; frameCoord=force.obj1)
    return nothing
end

function terminateForceElement(force::Bushing)
    return nothing
end
