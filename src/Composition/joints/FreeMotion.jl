# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#

# mutable struct of FreeMotion is defined in Modia3D/Composition/object3d.jl,
# since Object3D references FreeMotion and FreeMotion references Object3D

function driveJoint!(joint::FreeMotion)
   joint.isDriven = true
   joint.r.numericType  = ModiaMath.WR
   joint.v.numericType  = ModiaMath.WR
   joint.a.numericType  = ModiaMath.WR
   joint.q.numericType    = ModiaMath.WR
   joint.derq.numericType = ModiaMath.WR
   joint.w.numericType  = ModiaMath.WR
   joint.z.numericType  = ModiaMath.WR
end


function computeKinematics!(joint::FreeMotion, obj::Object3D, analysis::ModiaMath.AnalysisType, time::Float64)::NOTHING
   parent::Object3D = obj.parent
   absolute::Bool   = parent.r_abs ≡ ModiaMath.ZeroVector3D && parent.R_abs ≡ ModiaMath.NullRotation

   obj.r_rel = joint.r.value
   obj.R_rel = ModiaMath.from_q( joint.q.value )

   if absolute
      obj.r_abs = obj.r_rel
      obj.R_abs = obj.R_rel
   else
      obj.r_abs = parent.r_abs + parent.R_abs'*obj.r_rel
      obj.R_abs = obj.R_rel*parent.R_abs
   end

   q = joint.q.value
   joint.residue_q.value = dot(q, q) - 1.0

   if analysis == ModiaMath.DynamicAnalysis
      dynamics::Object3Ddynamics       = obj.dynamics
      parentDynamics::Object3Ddynamics = parent.dynamics

      joint.residue_w.value = joint.w.value - 2.0*( @SMatrix( [ q[4]  q[3] -q[2] -q[1];
                                                               -q[3]  q[4]  q[1] -q[2];
                                                                q[2] -q[1]  q[4] -q[3]] )*joint.derq.value)

      if absolute
         dynamics.v0 = joint.v.value
         dynamics.a0 = joint.a.value
         dynamics.w  = joint.w.value
         dynamics.z  = joint.z.value
      else
         dynamics.v0 = parentDynamics.v0 + parent.R_abs'*(joint.v.value + cross(parentDynamics.w, obj.r_rel))
         dynamics.a0 = parentDynamics.a0 + parent.R_abs'*(joint.a.value + cross(parentDynamics.z, obj.r_rel) +
                                                          cross(parentDynamics.w, cross(parentDynamics.w, obj.r_rel)))
         dynamics.w  = parent.R_abs*parentDynamics.w + joint.w.value
         dynamics.z  = parent.R_abs*parentDynamics.z + joint.z.value
      end
   end
   return nothing
end


function computeForceTorqueAndResidue!(joint::FreeMotion, obj::Object3D, analysis::ModiaMath.AnalysisType, time::Float64)::NOTHING
   joint.residue_f.value = obj.dynamics.f
   joint.residue_t.value = obj.dynamics.t
   return nothing
end



# Sets the relative position vector
function set_r!(obj::Object3D, r::ModiaMath.Vector3D)::NOTHING
   if isNotFree(obj)
      error("Not allowed to call set_r!(..) on Object3D ", ModiaMath.fullName(obj), " because not freely moving")
   end
   obj.joint.r.value = r
   obj.r_rel         = r
   return nothing
end
set_r!(obj::Object3D, r::AbstractVector)::NOTHING = set_r!(obj, ModiaMath.Vector3D(r))


# Sets the absolute position vector
function set_r_abs!(obj::Object3D, r::ModiaMath.Vector3D)::NOTHING
   if isNotFree(obj)
      error("Not allowed to call set_r!(..) on Object3D ", ModiaMath.fullName(obj), " because not freely moving")
   end
   obj.joint.r.value = r
   obj.r_abs         = r
   return nothing
end
set_r_abs!(obj::Object3D, r::AbstractVector)::NOTHING = set_r_abs!(obj, ModiaMath.Vector3D(r))



# Sets the relative quaternion
function set_q!(obj::Object3D, q::ModiaMath.Quaternion)::NOTHING
   if isNotFree(obj)
      error("Not allowed to call set_q!(..) on Object3D ", ModiaMath.fullName(obj), " because not freely moving")
   end
   obj.joint.q.value = q
   obj.R_rel         = ModiaMath.from_q(q)
   return nothing
end
set_q!(obj::Object3D, q::AbstractVector)::NOTHING = set_q!(obj, ModiaMath.Quaternion(q))
