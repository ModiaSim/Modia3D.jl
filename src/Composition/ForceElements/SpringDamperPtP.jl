
"""
    force = SpringDamperPtP(; obj1, obj2,
        nominalLength          = 0.0,
        nominalForce           = 0.0,
        stiffness              = 0.0,
        stiffnessForceFunction = nothing,
        damping                = 0.0,
        dampingForceFunction   = nothing )

Return a `force` acting as point-to-point parallel spring-damper between
`obj1::`[`Object3D`](@ref) and `obj2::`[`Object3D`](@ref).

# Arguments

- `nominalLength`: Nominal length, i.e. distance between `obj1` and `obj2`
  where deflection of stiffness force is zero.
- `nominalForce`: Nominal force, i.e. force that acts when stiffness and
  damping forces are zero. Positive values represent tensil forces.
- `stiffness`: Linear stiffness coefficient of spring. Ignored if
  `stiffnessForceFunction` is defined.
- `stiffnessForceFunction`: Univariate function which computes the
  stiffness force dependent of the stiffness force deflection.
- `damping`: Linear damping coefficient of damper. Ignored if
  `dampingForceFunction` is defined.
- `dampingForceFunction`: Univariate function which computes the
  damping force dependent of the damper velocity.
"""
mutable struct SpringDamperPtP <: Modia3D.AbstractForceElement

    obj1::Modia3D.AbstractObject3D
    obj2::Modia3D.AbstractObject3D

    nominalLength::Float64
    nominalForce::Float64

    stiffness::Float64
    stiffnessForceFunction::Union{Function, Nothing}

    damping::Float64
    dampingForceFunction::Union{Function, Nothing}

    function SpringDamperPtP(; obj1::Modia3D.AbstractObject3D,
                               obj2::Modia3D.AbstractObject3D,
                               nominalLength::Float64 = 0.0,
                               nominalForce::Float64 = 0.0,
                               stiffness::Float64 = 0.0,
                               stiffnessForceFunction::Union{Function, Nothing} = nothing,
                               damping::Float64 = 0.0,
                               dampingForceFunction::Union{Function, Nothing} = nothing)

        nomLength = Modia3D.convertAndStripUnit(Float64, u"m"    , nominalLength)
        nomForce  = Modia3D.convertAndStripUnit(Float64, u"N"    , nominalForce)
        stiff     = Modia3D.convertAndStripUnit(Float64, u"N/m"  , stiffness)
        damp      = Modia3D.convertAndStripUnit(Float64, u"N*s/m", damping)

        return new(obj1, obj2, nomLength, nomForce, stiff, stiffnessForceFunction, damp, dampingForceFunction)

    end
end


function initializeForceElement(force::SpringDamperPtP)
    force.obj1.hasForceElement = true
    force.obj2.hasForceElement = true
    return nothing
end

function evaluateForceElement(force::SpringDamperPtP)
    (pos, norm) = measFrameDistance(force.obj2; frameOrig=force.obj1)
    vel = measFrameDistVelocity(force.obj2; frameOrig=force.obj1)

    defl = pos - force.nominalLength
    if isnothing(force.stiffnessForceFunction)
        fc = force.stiffness * defl
    else
        fc = force.stiffnessForceFunction(defl)
    end
    if isnothing(force.dampingForceFunction)
        fd = force.damping * vel
    else
        fd = force.dampingForceFunction(vel)
    end
    f12 = (fc + fd + force.nominalForce) * norm

    applyFrameForcePair!(force.obj2, force.obj1, f12; frameCoord=force.obj1)
end

function terminateForceElement(force::SpringDamperPtP)
    return nothing
end
