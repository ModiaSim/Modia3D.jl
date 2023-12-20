# License for this file: MIT (expat)
# Copyright 2017-2021, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#

"""
    joint = Revolute(;obj1, obj2, axis=3, phi=0, w=0, canCollide=false)

Return a Revolute `joint` that rotates `obj1::`[`Object3D`](@ref) into
`obj2::`[`Object3D`](@ref) along the axis `axis` of `obj1` (`axis = 1,2,3,-1,-2,-3`) with angle `phi`.
Optionally, `axis` can be a vector, such as `axis = [1.0, 2.0, 3.0]`, defining the direction of the
axis of rotation in `obj1`. The initial start angle is `phi` and the initial angular velocity
is `w`.

If `canCollide=false`, no collision detection will occur between `obj1` and `obj2` (and `Object3D`s that are
directly or indirectly rigidly fixed to `obj1` or `obj2`). Note, if both `obj1` and `obj2` have
solids defined that are overlapping around the axis of rotation, then collision will be permanently
occuring and `canCollide=false` is the only meaningful value.

It is currently not supported that a `Revolute` joint *closes a kinematic loop*.
"""
mutable struct Revolute{F <: Modia3D.VarFloatType} <: Modia3D.AbstractJoint
    path::String

    obj1::Object3D{F}
    obj2::Object3D{F}

    eAxis::SVector{3,F}   # Rotation axis with norm(eAxis) = 1
    canCollide::Bool      # = false, if no collision between obj1 and obj2

    phi::F
    w::F
    a::F

    function Revolute{F}(; obj1::Object3D{F},
                           obj2::Object3D{F},
                           path::String="",
                           axis::Union{Int, AbstractVector} = 3,
                           phi::Real = F(0.0),
                           w::Real   = F(0.0),
                           canCollide::Bool = false) where F <: Modia3D.VarFloatType

        (parent,obj,cutJoint) = attach(obj1, obj2, name = "Revolute joint")  # an error is triggered if cutJoint=true

        if typeof(axis) <: Int
            eAxis =  abs(axis) == 1 ? SVector{3,F}(1, 0, 0) :
                    (abs(axis) == 2 ? SVector{3,F}(0, 1, 0) :
                    (abs(axis) == 3 ? SVector{3,F}(0, 0, 1) :
                        error("\nError from $path = Revolute(obj1 = :($(obj1.path)), obj2 = :($(obj2.path)), axis = $axis, ...):\n",
                              "    axis = must be 1,2,3 or -1,-2,-3 or a vector with 3 elements!")))
            if axis < 0
                eAxis = -eAxis
            end
        else
            eAxis = normalize(SVector{3,F}(ustrip.(axis)))
            if !all(isfinite,eAxis)
                error("\nError from $path = Revolute(obj1 = :($(obj1.path)), obj2 = :($(obj2.path)), axis = $axis, ...):\n",
                      "    normalize(axis) results in vector $eAxis that has non-finite elements!")
            end
        end

        eAxis = obj === obj2 ? eAxis : -eAxis
        phi = Modia3D.convertAndStripUnit(F, u"rad"  , phi)
        w   = Modia3D.convertAndStripUnit(F, u"rad/s", w)

        obj.joint = new(path, parent, obj, eAxis, canCollide, phi, w, F(0.0))
        obj.jointKind  = RevoluteKind
        obj.jointIndex = 0
        obj.ndof       = 1
        obj.canCollide = canCollide
        obj.r_rel      = Modia3D.ZeroVector3D(F)
        obj.R_rel      = Modia3D.rot_e(eAxis, phi)
        parent.hasChildJoint = true
        return obj.joint
    end
end
Revolute(; kwargs...) = Revolute{Float64}(; kwargs...)


function revertRevoluteKind!(oldChild::Object3D{F}, newChild::Object3D{F}) where F <: Modia3D.VarFloatType
    newChild.joint.obj1, newChild.joint.obj2 = newChild.joint.obj2, newChild.joint.obj1
    newChild.joint.eAxis = -newChild.joint.eAxis
    return nothing
end
