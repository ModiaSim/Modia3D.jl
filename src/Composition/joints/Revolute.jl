# License for this file: MIT (expat)
# Copyright 2017-2021, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
import Modia3D.Frames


"""
    joint = Revolute(;obj1, obj2, path="", axis=3, phi=0, w=0, canCollide=false)

Return a Revolute `joint` that rotates `obj1::`[`Object3D`](@ref) into
`obj2::`[`Object3D`](@ref) along the axis `axis` of `obj1` (`axis = 1,2,3,-1,-2,-3`).
The initial start angle is `phi` and the initial angular velocity
is `w`. Negative `axis` values describe axes in negative axis directions, i.e.
the signs of the values of `phi` and `w` are reversed. If `canCollide=false`, no
collision detection will occur between `obj1` and `obj2` (and `Object3D`s that are
directly or indirectly rigidly fixed to `obj1` or `obj2`).

It is currently not supported that a `Revolute` joint *closes a kinematic loop*.
"""
mutable struct Revolute{F} <: Modia3D.AbstractJoint
    path::String

    obj1::Object3D
    obj2::Object3D

    posAxis::Int      # = 1,2,3
    posMovement::Bool # = true, if movement along posAxis, otherwise in negative posAxis
    ndof::Int
    canCollide::Bool  # = false, if no collision between obj1 and obj2

    phi::Float64
    w::Float64
    a::Float64
    tau::Float64
    residue::Float64

    function Revolute{F}(; obj1::Object3D,
                        obj2::Object3D,
                        path::String="",
                        axis::Int = 3,
                        phi::Real = 0.0,
                        w::Real   = 0.0,
                        canCollide::Bool = false) where {F}

        (parent,obj,cutJoint) = attach(obj1, obj2)
        if cutJoint
            error("\nError from Revolute joint connecting ", Modia3D.fullName(obj1), " with ", Modia3D.fullName(obj2), ":\n",
                "    This joint is a cut-joint which is currently not supported.!")
        end

        if !(1 <= abs(axis) <= 3)
            error("\nError from Revolute joint connecting ", Modia3D.fullName(obj1), " with ", Modia3D.fullName(obj2), ":\n",
                "    axis = $axis, but must be 1,2,3 or -1,-2-3.!")
        end

        if !(typeof(phi) <: AbstractFloat) || !(typeof(w) <: AbstractFloat)
            error("\nError from Revolute joint connecting ", Modia3D.fullName(obj1), " with ", Modia3D.fullName(obj2), ":\n",
                "    phi and/or w are not <: AbstractFloat!")
        end

        axis = obj === obj2 ? axis : -axis
        phi = Modia3D.convertAndStripUnit(Float64, u"rad"  , phi)
        w   = Modia3D.convertAndStripUnit(Float64, u"rad/s", w)

        posAxis     = abs(axis)
        posMovement = axis > 0

        obj.joint = new(path, parent, obj, posAxis, posMovement, 1, canCollide, phi, w, 0.0, 0.0, 0.0)
        obj.jointKind  = RevoluteKind
        obj.jointIndex = 0
        obj.ndof       = 1
        obj.canCollide = canCollide
        obj.r_rel      = Modia3D.ZeroVector3D(Float64)
        obj.R_rel      = Frames.rotAxis(posAxis, posMovement, phi)

        parent.hasChildJoint = true
        return obj.joint
    end
end
