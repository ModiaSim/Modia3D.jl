
"""
    force = Bushing(; obj1, obj2,
        nominalForce      = [0.0, 0.0, 0.0],
        springForceLaw    = [0.0, 0.0, 0.0],
        damperForceLaw    = [0.0, 0.0, 0.0],
        nominalTorque     = [0.0, 0.0, 0.0],
        rotSpringForceLaw = [0.0, 0.0, 0.0],
        rotDamperForceLaw = [0.0, 0.0, 0.0],
        largeAngles       = false )

Return a `force` acting as bushing between `obj1::`[`Object3D`](@ref) and
`obj2::`[`Object3D`](@ref). The force directions are defined by `obj1`,
i.e. the orientation of `obj2` does not influence the resulting forces.

# Arguments

- `nominalForce` defines the nominal force vector, i.e. the force that
  acts when spring and damper forces are zero. Positive values act in
  positive axis directions at `obj1` and in opposite directions at `obj2`.
- `springForceLaw` defines the force law of the spring in x-, y- and
  z-direction:
  - A `Real` number represents a linear stiffness coefficient.
  - An univariate `Function` is used to compute the spring force
    dependent of its deflection.
- `damperForceLaw` defines the force law of the damper in x-, y- and
  z-direction:
  - A `Real` number represents a linear damping coefficient.
  - An univariate `Function` is used to compute the damper force
    dependent of its deflection velocity.
- `nominalTorque` defines nominal torques about alpha, beta and gamma
  directions.
- `rotSpringForceLaw` defines the force law of the rotational spring
  about alpha-, beta- and gamma-direction:
  - A `Real` number represents a linear damping coefficient.
  - An univariate `Function` is used to compute the spring force
    dependent of its deflection.
- `rotDamperForceLaw` defines the force law of the rotational damper
  about alpha-, beta- and gamma-direction:
  - A `Real` number represents a linear damping coefficient.
  - An univariate `Function` is used to compute the damper force
    dependent of its deflection velocity.
- `largeAngles` can be used to enable large angle mode.
  - When disabled, small deformation angles (< 10°) are assumed. This
    option deals equally with rotations [alpha, beta gamma] about the
    axes [x, y, z] of `obj1`, but causes approximation errors for
    larger angles.
  - When enabled, the deformation angles and torque directions are
    calculated as [Cardan (Tait–Bryan) angles](https://en.wikipedia.org/wiki/Euler_angles#Chained_rotations_equivalence)
    (rotation sequence x-y-z from `obj1` to `obj2`). This option
    supports angles up to nearly 90°, but introduces local rotation
    directions [alpha, beta gamma] which differ from the axes [x, y, z]
    of `obj1` and increases computation effort.
"""
mutable struct Bushing{F <: AbstractFloat} <: Modia3D.AbstractForceElement

    obj1::Object3D{F}
    obj2::Object3D{F}

    nominalForce::SVector{3,F}
    springForceFunction::SVector{3,Function}
    damperForceFunction::SVector{3,Function}
    nominalTorque::SVector{3,F}
    rotSpringForceFunction::SVector{3,Function}
    rotDamperForceFunction::SVector{3,Function}
    largeAngles::Bool

    function Bushing{F}(; obj1::Object3D{F},
                          obj2::Object3D{F},
                          nominalForce::AbstractVector = Modia3D.ZeroVector3D(F),
                          springForceLaw::AbstractVector = Modia3D.ZeroVector3D(F),
                          damperForceLaw::AbstractVector = Modia3D.ZeroVector3D(F),
                          nominalTorque::AbstractVector = Modia3D.ZeroVector3D(F),
                          rotSpringForceLaw::AbstractVector = Modia3D.ZeroVector3D(F),
                          rotDamperForceLaw::AbstractVector = Modia3D.ZeroVector3D(F),
                          largeAngles::Bool = false ) where F <: AbstractFloat
        nomForce  = Modia3D.convertAndStripUnit(SVector{3,F}, u"N"  , nominalForce)
        nomTorque = Modia3D.convertAndStripUnit(SVector{3,F}, u"N*m", nominalTorque)
        springForceFunction = Vector{Function}(undef, 3)
        damperForceFunction = Vector{Function}(undef, 3)
        rotSpringForceFunction = Vector{Function}(undef, 3)
        rotDamperForceFunction = Vector{Function}(undef, 3)
        irand = rand(Int)
        for dir in 1:3
            if (isa(springForceLaw[dir], Function))
                springForceFunction[dir] = springForceLaw[dir]
            else
                stiffness = Modia3D.convertAndStripUnit(F, u"N/m", springForceLaw[dir])
                fsymb = Symbol("fc", dir, "_", irand)  # todo: replace irand by force.path
                springForceFunction[dir] = eval(:($fsymb(pos) = $stiffness * pos))
            end
            if (isa(damperForceLaw[dir], Function))
                damperForceFunction[dir] = damperForceLaw[dir]
            else
                damping = Modia3D.convertAndStripUnit(F, u"N*s/m", damperForceLaw[dir])
                fsymb = Symbol("fd", dir, "_", irand)  # todo: replace irand by force.path
                damperForceFunction[dir] = eval(:($fsymb(vel) = $damping * vel))
            end
            if (isa(rotSpringForceLaw[dir], Function))
                rotSpringForceFunction[dir] = rotSpringForceLaw[dir]
            else
                stiffness = Modia3D.convertAndStripUnit(F, u"N*m/rad", rotSpringForceLaw[dir])
                fsymb = Symbol("mc", dir, "_", irand)  # todo: replace irand by force.path
                rotSpringForceFunction[dir] = eval(:($fsymb(ang) = $stiffness * ang))
            end
            if (isa(rotDamperForceLaw[dir], Function))
                rotDamperForceFunction[dir] = rotDamperForceLaw[dir]
            else
                damping = Modia3D.convertAndStripUnit(F, u"N*m*s/rad", rotDamperForceLaw[dir])
                fsymb = Symbol("md", dir, "_", irand)  # todo: replace irand by force.path
                rotDamperForceFunction[dir] = eval(:($fsymb(angd) = $damping * angd))
            end
        end

        return new(obj1, obj2, nomForce, springForceFunction, damperForceFunction, nomTorque, rotSpringForceFunction, rotDamperForceFunction, largeAngles)
    end
end
Bushing(; kwargs...) = Bushing{Float64}(; kwargs...)


# Compute deformation angles from rotation matrix
function anglesFromRotation(largeAngles::Bool, R12::SMatrix{3,3,F}, w12::SVector{3,F}) where F <: AbstractFloat
    if largeAngles
        sbe = clamp(R12[3,1], F(-1.0), F(1.0))
        cbe = sqrt(F(1.0) - sbe*sbe)
        if (cbe > 1e-12)
            sal = -R12[3,2]/cbe
            cal =  R12[3,3]/cbe
            al = atan(-R12[3,2], R12[3,3])
            be = asin(sbe)
            ga = atan(-R12[2,1], R12[1,1])
            ald = w12[1] + (sal*w12[2] - cal*w12[3])*sbe/cbe
            bed = cal*w12[2] + sal*w12[3]
            gad = (-sal*w12[2] + cal*w12[3])/cbe
            return (SVector{3,F}(al, be, ga), SVector{3,F}(ald, bed, gad), SMatrix{2,2,F}(sal, cal, sbe, cbe))
        else
            @error("Gimbal lock of Bushing transformation.")
            return (SVector{3,F}(0.0, 0.0, 0.0), SVector{3,F}(0.0, 0.0, 0.0), SMatrix{2,2,F}(0.0, 0.0, 0.0, 0.0))
        end
    else
        al = R12[2,3]
        be = R12[3,1]
        ga = R12[1,2]
        ald = w12[1]
        bed = w12[2]
        gad = w12[3]
        if (max(abs(al), abs(be), abs(ga)) > 0.174)
            @warn("Bushing angle exceeds 10 deg.")
        end
        return (SVector{3,F}(al, be, ga), SVector{3,F}(ald, bed, gad), SMatrix{2,2,F}(0.0, 0.0, 0.0, 0.0))
    end
end

# Compute torque vector from force law moments
function torqueFromMoments(largeAngles::Bool, moments::SVector{3,F}, sico::SMatrix{2,2,F}) where F <: AbstractFloat
    if largeAngles
        tx = moments[1] + sico[1,2]*moments[3]
        ty = sico[2,1]*moments[2] - sico[1,1]*sico[2,2]*moments[3]
        tz = sico[1,1]*moments[2] + sico[2,1]*sico[2,2]*moments[3]
        return SVector{3,F}(tx, ty, tz)
    else
        return moments
    end
end


function initializeForceElement(force::Bushing{F}) where F <: AbstractFloat
    force.obj1.hasForceElement = true
    force.obj2.hasForceElement = true
    return nothing
end

function evaluateForceElement(force::Bushing{F}) where F <: AbstractFloat
    R12 = measFrameRotation(force.obj2; frameOrig=force.obj1)
    r12 = measFramePosition(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1)
    w12 = measFrameRotVelocity(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1)
    v12 = measFrameTransVelocity(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1, frameObsrv=force.obj1)
    (ang, angd, sico) = anglesFromRotation(force.largeAngles, R12, w12)

    f12 = Vector{F}(undef, 3)
    mom = Vector{F}(undef, 3)
    for dir in 1:3
        f12[dir] = force.springForceFunction[dir](r12[dir]) + force.damperForceFunction[dir](v12[dir]) + force.nominalForce[dir]
        mom[dir] = force.rotSpringForceFunction[dir](ang[dir]) + force.rotDamperForceFunction[dir](angd[dir]) + force.nominalTorque[dir]
    end
    t12 = torqueFromMoments(force.largeAngles, SVector{3,F}(mom), sico)

    applyFrameForcePair!(force.obj2, force.obj1, SVector{3,F}(f12); frameCoord=force.obj1)
    applyFrameTorquePair!(force.obj2, force.obj1, t12; frameCoord=force.obj1)
    return nothing
end

function terminateForceElement(force::Bushing{F}) where F <: AbstractFloat
    return nothing
end
