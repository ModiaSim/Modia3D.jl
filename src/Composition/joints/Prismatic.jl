# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#



get_eAxis(::Type{F}, axis::Int) where F <: Modia3D.VarFloatType = axis==  1 ? SVector{3,F}(  1.0,  0.0,  0.0) :
                       axis==  2 ? SVector{3,F}(  0.0,  1.0,  0.0) :
                       axis==  3 ? SVector{3,F}(  0.0,  0.0,  1.0) :
                       axis== -1 ? SVector{3,F}( -1.0,  0.0,  0.0) :
                       axis== -2 ? SVector{3,F}(  0.0, -1.0,  0.0) :
                       axis== -3 ? SVector{3,F}(  0.0,  0.0, -1.0) :
                       error("Modia3D.Composition.Prismatic: axis = ", axis, " but must be 1, 2, 3, -1, -2, or -3.")

"""
    joint = Prismatic(; obj1, obj2, path="", axis=1, s=0, v=0, canCollide=true)

Return a `joint` that translates `obj2::`[`Object3D`](@ref) with respect to
`obj1::`[`Object3D`](@ref) along coordinate axis `axis` (`axis = 1,2,3,-1,-2,-3`)
of `obj1`. The initial position is `s` and the initial velocity is `v`. Negative
`axis` values describe axes in negative axis directions, i.e. the signs of the values
of `s` and `v` are reversed. If `canCollide=false`, no collision detection will occur
between `obj1` and `obj2` (and `Object3D`s that are directly or indirectly rigidly
fixed to `obj1` or `obj2`).

It is currently not supported that a `Prismatic` joint *closes a kinematic loop*.
"""
mutable struct Prismatic{F <: Modia3D.VarFloatType} <: Modia3D.AbstractJoint
    path::String

    obj1::Object3D{F}
    obj2::Object3D{F}

    posAxis::Int             # = 1,2,3
    posMovement::Bool        # = true, if movement along posAxis, otherwise in negative posAxis
    eAxis::SVector{3,F}   # Unit vector in direction of axis
    ndof::Int
    canCollide::Bool         # = false, if no collision between obj1 and obj2

    s::F
    v::F
    a::F

    function Prismatic{F}(; obj1::Object3D{F},
                         obj2::Object3D{F},
                         path::String="",
                         axis::Int = 1,
                         s::Real   = F(0.0),
                         v::Real   = F(0.0),
                         canCollide::Bool = true) where F <: Modia3D.VarFloatType

        (parent,obj,cutJoint) = attach(obj1, obj2, name = "Prismatic joint")  # an error is triggered if cutJoint=true

        if !(1 <= abs(axis) <= 3)
            error("\nError from Prismatic joint connecting ", Modia3D.fullName(obj1), " with ", Modia3D.fullName(obj2), ":\n",
                "    axis = $axis, but must be 1,2,3 or -1,-2-3.!")
        end

        if !(typeof(s) <: AbstractFloat) || !(typeof(v) <: AbstractFloat)
            error("\nError from Prismatic joint connecting ", Modia3D.fullName(obj1), " with ", Modia3D.fullName(obj2), ":\n",
                "    s and/or v are not <: AbstractFloat!")
        end

        axis  = obj === obj2 ? axis : -axis
        eAxis = get_eAxis(F, axis)

        s = Modia3D.convertAndStripUnit(F, u"m"  , s)
        v = Modia3D.convertAndStripUnit(F, u"m/s", v)

        posAxis     = abs(axis)
        posMovement = axis > 0

        obj.joint      = new(path, parent, obj, posAxis, posMovement, eAxis, 1, canCollide, s, v, F(0.0) )
        obj.jointKind  = PrismaticKind
        obj.jointIndex = 0
        obj.ndof       = 1
        obj.canCollide = canCollide
        obj.r_rel      = eAxis*s
        obj.R_rel      = Modia3D.NullRotation(F)

        parent.hasChildJoint = true
        return obj.joint
    end
end
