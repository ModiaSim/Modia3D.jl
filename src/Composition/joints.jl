# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#


# Utility function that should not be directly called (only to be called from attach(..)
function attachAndReverseParents(newParent::Object3D, obj::Object3D)::NOTHING
   @assert(!(newParent ≡ obj))

   # Save elements of obj
   r_rel  = obj.r_rel
   R_rel  = obj.R_rel
   parent = obj.parent

   # Reverse obj, so that newParent is the new parent
   obj.parent = newParent
   push!(newParent.children, obj)

   # Reverse all parents of obj
   obj1::Object3D = obj
   obj2::Object3D = parent
   while !(parent === obj1)
      parent_r_rel = parent.r_rel
      parent_R_rel = parent.R_rel

      if r_rel ≡ ModiaMath.ZeroVector3D
         parent.r_rel = ModiaMath.ZeroVector3D
         parent.r_abs = obj1.r_abs
      else
         parent.r_rel = -R_rel*r_rel
      end

      if R_rel ≡ ModiaMath.NullRotation
         parent.R_rel = ModiaMath.NullRotation
         parent.R_abs = obj1.R_abs
      else
         parent.R_rel = R_rel'
      end

      obj2 = parent
      parent = obj2.parent
      r_rel  = parent_r_rel
      R_rel  = parent_R_rel
      obj2.parent = obj1
      removeChild!(obj2, obj1)
      push!(obj1.children, obj2)
      obj1 = obj2
   end

   # If scene != nothing, visit all objs and set scene
#=
   if typeof(scene) != NOTHING
      stack = scene.stack
      empty!(stack)
      append!(stack, obj.children)
      while length(stack) > 0
         obj1 = pop!(stack)
         obj1.scene = scene
         append!(stack, obj1.children)
      end
   end
=#
   return nothing
end


"""
    (obj1, obj2, cutJoint) = attach(frame_a, frame_b)

If frame_a and frame_b do not have the same root, they are attached to each other
and cutJoint = false is returned.

If they have the same root, the tree is not modified and cutJoint=true is returned.
"""
function attach(obj1::Object3D, obj2::Object3D)
   root1 = rootObject3D(obj1)
   root2 = rootObject3D(obj2)
   #println("attach: obj1 = ", ModiaMath.fullName(obj1), ", root = ", ModiaMath.fullName(root1))
   #println("attach: obj2 = ", ModiaMath.fullName(obj2), ", root = ", ModiaMath.fullName(root2))

   if root1 ≡ root2
      # Compute absolute positions
      updatePosition!(root1)
      return (obj1,obj2,true)
   end

   if isWorld(root2)
      attachAndReverseParents(obj2, obj1)
      return (obj2,obj1,false)
   else
      attachAndReverseParents(obj1, obj2)
      return (obj1,obj2,false)
   end
end


function connect(obj1::Object3D, obj2::Object3D;
                 r::AbstractVector = ModiaMath.ZeroVector3D,
                 R::Union{ModiaMath.RotationMatrix,NOTHING} = nothing,
                 q::Union{ModiaMath.Quaternion,NOTHING} = nothing,
                 fixed::Bool = true)::Nothing
   if typeof(R) != NOTHING && typeof(q) != NOTHING
      error("Modia3D.connect(...): either R or q must be nothing but both have a value.")
   end
   if typeof(R) != NOTHING
      ModiaMath.assertRotationMatrix(R)
   elseif typeof(q) != NOTHING
      ModiaMath.assertQuaternion(q)
   end


   (parentObject3D, obj, cutJoint) = attach(obj1, obj2)
  # println("... connect, fixed = ", fixed, ", obj1=",ModiaMath.fullName(obj1), ", obj2 = ", ModiaMath.fullName(obj2), ", obj = ", ModiaMath.fullName(obj))


   if cutJoint
      error("Error from Modia3D.Composition.connect(", obj1.name, ",", obj2.name, "):\n",
            "Not yet supported to rigidly connect two objs that have the same root.")
   end

   r_rel = SVector{3,Float64}(r)
   R_rel = typeof(R) != NOTHING ? R                   :
           typeof(q) != NOTHING ? ModiaMath.from_q(q) : ModiaMath.NullRotation

   obj.r_rel = obj===obj2 ? r_rel : -R_rel*r_rel
   obj.R_rel = obj===obj2 ? R_rel :  R_rel'

   #r_abs = parent.r_abs + r_rel
   #R_abs = R_rel*parent.R_abs


   if fixed
      obj.joint = fixedJoint
   else
      obj1.hasChildJoint = true
      q_start = typeof(R) != NOTHING ? ModiaMath.from_R(R) :
                typeof(q) != NOTHING ? q                   : ModiaMath.NullQuaternion
      q_start = obj===obj2 ? q_start : ModiaMath.inverseRotation(q_start)

      obj.joint = FreeMotion(obj, r_start = obj.r_rel, q_start = q_start)
   end
   return nothing
end


updatePosition!(assembly::Modia3D.AbstractAssembly) = updatePosition!(assembly._internal.referenceObject3D)
function updatePosition!(frame::Object3D)::NOTHING
   stack = Object3D[]
   # Push initial children on stack
   append!(stack, frame.children)
   while length(stack) > 0
      frame = pop!(stack)

      computeKinematics!(frame.joint, frame, ModiaMath.KinematicAnalysis, 0.0)

#=
      parent = frame.parent

      if frame.r_rel ≡ ModiaMath.ZeroVector3D
         frame.r_abs = parent.r_abs
      else
         frame.r_abs = parent.r_abs + parent.R_abs'*frame.r_rel
      end

      if frame.R_rel ≡ ModiaMath.NullRotation
         frame.R_abs = parent.R_abs
      else
         frame.R_abs = frame.R_rel*parent.R_abs
      end
=#
      if typeof(frame.visualizationFrame) != NOTHING
         frame.visualizationFrame.r_abs = frame.r_abs
         frame.visualizationFrame.R_abs = frame.R_abs
      end

      append!(stack, frame.children)
   end
   return nothing
end
