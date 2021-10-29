
"""
force = Bushing(; obj1, obj2, stiffness, damping)

Return a `force` acting as bushing between `obj1::`[`Object3D`](@ref) and
`obj2::`[`Object3D`](@ref). Vectors `stiffness` and `damping` define the
stiffness resp. damping values in x, y and z direction of `obj1`. The
orientation of `obj2` does no influence the resulting forces.
"""
mutable struct Bushing <: Modia3D.AbstractForceElement

    obj1::Modia3D.AbstractObject3D
    obj2::Modia3D.AbstractObject3D

    stiffness::SVector{3,Float64}
    damping::SVector{3,Float64}

    function Bushing(; obj1::Modia3D.AbstractObject3D,
                       obj2::Modia3D.AbstractObject3D,
                       stiffness::AbstractVector = Modia3D.ZeroVector3D,
                       damping::AbstractVector   = Modia3D.ZeroVector3D)

        stiff = Modia3D.convertAndStripUnit(SVector{3,Float64}, u"N/m" , stiffness)
        damp  = Modia3D.convertAndStripUnit(SVector{3,Float64}, u"N*s/m", damping)

        return new(obj1, obj2, stiff, damp)

    end
end


function initializeForceElement(force::Bushing)
    return nothing
end

function evaluateForceElement(time, force::Bushing)
    r12 = measFramePosition(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1)
    v12 = measFrameTransVelocity(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1, frameObsrv=force.obj1)
    f12 = force.stiffness .* r12 + force.damping .* v12
    applyFrameForcePair!(force.obj2, force.obj1, f12; frameCoord=force.obj1)
    return nothing
end

function terminateForceElement(force::Bushing)
    return nothing
end
