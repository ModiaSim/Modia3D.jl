# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#

# mutable struct of FreeMotion is defined in Modia3D/Composition/object3d.jl, 
# since Object3D references FreeMotion and FreeMotion references Object3D


function computeKinematics!(joint::FixedJoint, obj::Object3D, analysis::ModiaMath.AnalysisType, time::Float64)::NOTHING
   parent::Object3D    = obj.parent
   noTranslation::Bool = obj.r_rel ≡ ModiaMath.ZeroVector3D
   noRotation::Bool    = obj.R_rel ≡ ModiaMath.NullRotation

   obj.r_abs = noTranslation ? parent.r_abs : parent.r_abs + parent.R_abs'*obj.r_rel
   obj.R_abs = noRotation    ? parent.R_abs : obj.R_rel*parent.R_abs

   if analysis == ModiaMath.DynamicAnalysis
      dynamics::Object3Ddynamics       = obj.dynamics
      parentDynamics::Object3Ddynamics = parent.dynamics

      if noTranslation
         dynamics.v0 = parentDynamics.v0
         dynamics.a0 = parentDynamics.a0 
      else
         dynamics.v0 = parentDynamics.v0 + parent.R_abs'*cross(parentDynamics.w, obj.r_rel)
         dynamics.a0 = parentDynamics.a0 + parent.R_abs'*(cross(parentDynamics.z, obj.r_rel) +
                                                          cross(parentDynamics.w, cross(parentDynamics.w, obj.r_rel)))
      end

      if noRotation
         dynamics.w = parentDynamics.w
         dynamics.z = parentDynamics.z
      else
         dynamics.w = obj.R_rel*parentDynamics.w
         dynamics.z = obj.R_rel*parentDynamics.z 
      end
   end
   return nothing
end


function computeForceTorqueAndResidue!(joint::FixedJoint, obj::Object3D, analysis::ModiaMath.AnalysisType, time::Float64)::NOTHING
   parent::Object3D = obj.parent

   if obj.R_rel ≡ ModiaMath.NullRotation
      # Fixed translation
      parent.dynamics.f += obj.dynamics.f
      parent.dynamics.t += obj.dynamics.t + cross(obj.r_rel, obj.dynamics.f)
   else
      # Fixed translation and rotation
      parent.dynamics.f += obj.R_rel'*obj.dynamics.f
      parent.dynamics.t += obj.R_rel'*obj.dynamics.t - cross(obj.r_rel, parent.dynamics.f)
   end
   return nothing
end

