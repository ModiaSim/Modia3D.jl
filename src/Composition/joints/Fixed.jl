# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#

mutable struct TreeJointFixed <: Modia3D.AbstractJoint
   _internal::ModiaMath.ComponentInternal  # Data common to assembly component classes
   obj1::Object3D                          # obj2 is fixed to obj1
   obj2::Object3D
end


function Base.show(io::IO, joint::TreeJointFixed)
   print(io,"Fixed(", ModiaMath.fullName(joint.obj1),
                   ", ", ModiaMath.fullName(joint.obj2))
end



"""
    joint = Modia3D.Fixed(obj1, obj2; r=zeros(3), R=nothing, q=nothing)

Return a Fixed `joint` that fixes `obj2::`[`Object3D`](@ref) relatively to
`obj1::`[`Object3D`](@ref) via the relative position vector from `obj1` to `obj2`
(resolved in `obj1`) and the relative transformation matrix `R` from `obj1` to `obj2`
or alternatively the relative quaternion `q`.

# Arguments
- `obj1::Object3D`: Parent object.

- `obj2::Object3D`: Object fixed relative to `obj1`.

- `r::AbstractVector`: Relative position vector from `obj1` to `obj2`
   resolved in `obj1`.

- `R::Union{ModiaMath.RotationMatrix,Nothing}`: Rotation matrix defining the rotation
   from `obj1` to `obj`. If both `R = nothing` and `q = nothing`,
   a null rotation is defined.

- `q::Union{ModiaMath.Quaternion,Nothing}`: Quaternion defining the rotation
   from `obj1` to `obj2`. If both `R = nothing` and `q = nothing`,
   a null rotation is defined.
"""
function Fixed(obj1::Object3D, obj2::Object3D;
               r::AbstractVector = ModiaMath.ZeroVector3D,
               R::Union{ModiaMath.RotationMatrix,NOTHING} = nothing,
               q::Union{ModiaMath.Quaternion,NOTHING} = nothing)::TreeJointFixed

    connect(obj1, obj2, r=r, R=R, q=q)
    if obj2.parent ≡ obj1
		obj2.joint = TreeJointFixed(ModiaMath.ComponentInternal(), obj1, obj2)
        return obj2.joint
    elseif obj1.parent ≡ obj2
		obj1.joint = TreeJointFixed(ModiaMath.ComponentInternal(), obj2, obj1)
        return obj1.joint
    else
        error("Fixed: unexpected error. obj1 = ", ModiaMath.fullName(obj1),
              ", obj2 = ", ModiaMath.fullName(obj2))
    end
end


computeKinematics!(joint::TreeJointFixed, obj::Object3D, analysis::ModiaMath.AnalysisType, time::Float64) = nothing
computeForceTorqueAndResidue!(joint::TreeJointFixed, obj::Object3D, analysis::ModiaMath.AnalysisType, time::Float64) = nothing
