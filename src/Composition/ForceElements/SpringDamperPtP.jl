
"""
force = SpringDamperPtP(; obj1, obj2, stiffness, damping)

Return a `force` acting as point-to-point parallel spring-damper between
`obj1::`[`Object3D`](@ref) and `obj2::`[`Object3D`](@ref). Values
`stiffness` and `damping` define the stiffness resp. damping coefficient.
"""
mutable struct SpringDamperPtP <: Modia3D.AbstractForceElement

    obj1::Modia3D.AbstractObject3D
    obj2::Modia3D.AbstractObject3D

    stiffness::Float64
    damping::Float64

    function SpringDamperPtP(; obj1::Modia3D.AbstractObject3D,
                               obj2::Modia3D.AbstractObject3D,
                               stiffness::Float64 = 0.0,
                               damping::Float64 = 0.0)

        stiff = Modia3D.convertAndStripUnit(Float64, u"N/m"  , stiffness)
        damp  = Modia3D.convertAndStripUnit(Float64, u"N*s/m", damping)

        return new(obj1, obj2, stiff, damp)

    end
end


function initializeForceElement(force::SpringDamperPtP)
    return nothing
end

function evaluateForceElement(time, force::SpringDamperPtP)
    (pos, norm) = measFrameDistance(force.obj2; frameOrig=force.obj1)
    vel = measFrameDistVelocity(force.obj2; frameOrig=force.obj1)
    frc = force.stiffness * pos + force.damping * vel
    f12 = frc * norm
    applyFrameForcePair!(force.obj2, force.obj1, f12; frameCoord=force.obj1)
end

function terminateForceElement(force::SpringDamperPtP)
    return nothing
end
