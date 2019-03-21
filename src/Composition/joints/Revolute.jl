# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
@static if VERSION >= v"0.7.0-DEV.2005"
    using LinearAlgebra
    EYE3() = Matrix(1.0I,3,3)
else
    EYE3() = eye(3)
end

mutable struct TreeJointRevolute <: Modia3D.AbstractRevolute
   _internal::ModiaMath.ComponentInternal  # Data common to assembly component classes
   obj1::Object3D                        # Rotation of obj1 to obj2 along z-axis of obj1
   obj2::Object3D
   canCollide::Bool                        # = true, if collision detection is enabled between the two Object3Ds connected by the Revolute joint (default = false)
   isDriven::Bool                          # = true, if setAngle!(..) can be called on the joint

   # Definitions special to Revolute joints
   axis::Int            # Rotation axis (axis = 1,2,3 or -1,-2,-3)
   # phi_ref::Float64   # Value of angle phi in reference configuration
   # phi::Float64       # Actual value of relative rotation angle (obj1 and obj2 coincide, if phi = 0)

   phi::ModiaMath.RealScalar
   w::ModiaMath.RealScalar
   a::ModiaMath.RealScalar
   tau::ModiaMath.RealScalar
   residue::ModiaMath.RealScalar

   # The following variables are only provided for checking (should be removed, once all tests successful)
   #   P_derx::ModiaMath.RealScalar  # Total power flowing in obj1, obj2, axis computed with derx-vector (must be zero; = computed with error control)
   #   P_w::ModiaMath.RealScalar     # Total power flowing in obj1, obj2, axis computed at communication points (must be zero; = computed with interpolated values of x,derx)
   #   residue_P_derx::ModiaMath.RealScalar # Residue of P_derx

   flange::RevoluteFlange

   TreeJointRevolute(_internal, obj1, obj2, canCollide, isDriven, axis) = new(_internal, obj1, obj2, canCollide, isDriven, axis)
end


function Base.show(io::IO, joint::TreeJointRevolute)
   print(io,"Revolute(", ModiaMath.fullName(joint.obj1),
                   ", ", ModiaMath.fullName(joint.obj2),
                      ", axis = ", joint.axis,
                      ", phi_start = ", Basics.radToDeg*joint.phi.start, " deg)")
end

#=
function JSON.show_json(io::JSON.StructuralContext,
                         s::JSON.CommonSerialization, joint::TreeJointRevolute)
   JSON.begin_object(io)
      JSON.show_pair(io, s, "obj_a"  , joint.obj1.name)
      JSON.show_pair(io, s, "obj_b"  , joint.obj2.name)
      JSON.show_pair(io, s, "driven"   , joint.driven)
      JSON.show_pair(io, s, "cutJoint" , joint.cutJoint)
      JSON.show_pair(io, s, "axis"     , joint.axis)
      JSON.show_pair(io, s, "phi_start", joint.phi.start)
   JSON.end_object(io)
end
=#


mutable struct CutJointRevolute <: Modia3D.AbstractRevolute
   _internal::ModiaMath.ComponentInternal  # Data common to assembly component classes
   obj1::Object3D                                   # Rotation of obj1 to obj2 along z-axis of obj1
   obj2::Object3D
   # driven::Bool                                    # = true, if setAngle!(..) can be called on the joint
   visited::Bool                                   # = true, if already visited when traversing the spanning tree

   # Definitions special to Revolute joints
   axis::Int            # Rotation axis (axis = 1,2,3 or -1,-2,-3)
   # phi_ref::Float64   # Value of angle phi in reference configuration
   # phi::Float64       # Actual value of relative rotation angle (obj1 and obj2 coincide, if phi = 0)

   lambda_x::ModiaMath.RealScalar
   lambda_y::ModiaMath.RealScalar
   lambda_z::ModiaMath.RealScalar
   mue_x::ModiaMath.RealScalar
   mue_y::ModiaMath.RealScalar
   mue_z::ModiaMath.RealScalar
   residue_x::Union{ModiaMath.RealScalar, NOTHING}
   residue_y::Union{ModiaMath.RealScalar, NOTHING}
   residue_z::Union{ModiaMath.RealScalar, NOTHING}
   residue_vx::Union{ModiaMath.RealScalar, NOTHING}
   residue_vy::Union{ModiaMath.RealScalar, NOTHING}
   residue_vz::Union{ModiaMath.RealScalar, NOTHING}

   CutJointRevolute(_internal, obj1, obj2, visited, axis) = new(_internal, obj1, obj2, visited, axis)
end


"""
    joint = Modia3D.Revolute(obj1, obj2; axis=3, phi_start=0, w_start=0, canCollide=false)

Return a Revolute `joint` that rotates `obj1::`[`Object3D`](@ref) into
`obj2::`[`Object3D`](@ref) along the z-axis of `obj1`.
The initial start angle is `phi_start` and the initial angular velocity
is `w_start`. If `canCollide=false`, no collision detection will occur between `obj1` and `obj2`
(and `Object3D`s that are directly or indirectly rigidly fixed to `obj1` or `obj2`).

If a `Revolute` joint *closes a kinematic loop*, then the already present objects must be consistent
to the `Revolute` joint that is the frames of `obj1` and `obj2` must be *parallel* to each other and
rotation of `obj1` along its axis `axis` with `phi_start` results in `obj2`. If `phi_start=NaN`,
its value is computed in this case.

# Examples
```julia
using Modia3D
import ModiaMath

@assembly Pendulum(;L=1.0) begin
   world  = Modia3D.Object3D()
   obj1 = Modia3D.Object3D()
   rev    = Modia3D.Revolute(world, obj1)
   sphere = Modia3D.Object3D(obj1, Modia3D.Solid(Modia3D.SolidSphere(0.1), "Aluminium"),
                             r = [L,0,0] )
end

simulationModel = Modia3D.SimulationModel( Pendulum(L=0.8), stopTime=5.0 )
result          = ModiaMath.simulate!(simulationModel)
ModiaMath.plot(result, ("rev.phi", "rev.w"))
```
"""
function Revolute(obj1::Object3D, obj2::Object3D;
                  phi_start::Number = 0.0,
                  w_start::Number   = 0.0,
                  canCollide::Bool  = false,
                  axis::Int = 3)
   rphi_start = Float64(phi_start)
   rw_start   = Float64(w_start)
   (obj_a,obj_b,cutJoint) = attach(obj1, obj2)
   @assert(1 <= axis <= 3)
   axis  = obj_b===obj2 ? axis : -axis
   println("axis = $axis")
   if cutJoint
      println("... Revolute joint connecting ", ModiaMath.fullName(obj1), " with ", ModiaMath.fullName(obj2), " is a cut-joint")

      # Check that rotating from obj_a and obj_b have the same origin and
      # rotating from obj_a along z-axis with angle phi gives obj_b (or compute rphi_start if phi_start = NaN)
      diff_r_abs = norm(obj_b.r_abs - obj_a.r_abs)
      if diff_r_abs > Basics.neps
         @static if VERSION >= v"0.7.0-DEV.2005"
             @warn begin
                 name_obj1 = ModiaMath.fullName(obj1)
                 name_obj2 = ModiaMath.fullName(obj2)
                 "Warning from Modia3D.Revolute($name_obj1, $name_obj2, ....):\n" *
                 "The origins of the provided frames do not coincide.\n" *
                 "( norm(obj2.r_abs - obj1.r_abs) = $diff_r_abs > $(Basics.neps) )"
             end
         else
             warn("\nWarning from Modia3D.Revolute(", ModiaMath.fullName(obj1), ", ", ModiaMath.fullName(obj2), ", ...):\n",
                  "The origins of the provided frames do not coincide.\n",
                  "( norm(obj2.r_abs - obj1.r_abs) = ", string(diff_r_abs), " > ", Basics.neps, ")")
         end
      end
      if isnan(rphi_start)
         # Compute angle between the two z-axes
         @error("brrrr!!!!!!!!!!!!!!!!!!")
         angle = acos( dot(obj_a.R_abs[abs(axis),:], obj_b.R_abs[abs(axis),:]) )
         if abs(angle) > Basics.neps
             @static if VERSION >= v"0.7.0-DEV.2005"
                 @warn begin
                     name_obj1 = ModiaMath.fullName(obj1)
                     name_obj2 = ModiaMath.fullName(obj2)
                     if abs(axis) == 3
                        "Warning from Modia3D.Revolute($name_obj1, $name_obj2, ....):\n" *
                        "The z-axes of the two frames do not coincide.\n" *
                        "( angle(zaxis(obj1), zaxis(obj2)) = $zaxes_angle > $(Basics.neps) )"
                     elseif abs(axis) == 2
                        "Warning from Modia3D.Revolute($name_obj1, $name_obj2, ....):\n" *
                        "The y-axes of the two frames do not coincide.\n" *
                        "( angle(yaxis(obj1), yaxis(obj2)) = $yaxes_angle > $(Basics.neps) )"
                     elseif abs(axis) == 1
                        "Warning from Modia3D.Revolute($name_obj1, $name_obj2, ....):\n" *
                        "The x-axes of the two frames do not coincide.\n" *
                        "( angle(xaxis(obj1), xaxis(obj2)) = $xaxes_angle > $(Basics.neps) )"
                     end
                 end
             else

                      if abs(axis) == 3
                        warn("\nWarning from Modia3D.Revolute(", ModiaMath.fullName(obj1), ", ", ModiaMath.fullName(obj2), ", ...):\n",
                             "The z-axes of the two frames do not coincide.\n",
                             "( angle(zaxis(obj1), zaxis(obj2)) = ", string(), " > ", Basics.neps, ")")
                     elseif abs(axis) == 2
                        warn("\nWarning from Modia3D.Revolute(", ModiaMath.fullName(obj1), ", ", ModiaMath.fullName(obj2), ", ...):\n",
                            "The y-axes of the two frames do not coincide.\n",
                            "( angle(yaxis(obj1), yaxis(obj2)) = ", string(), " > ", Basics.neps, ")")
                     elseif abs(axis) == 1
                        warn("\nWarning from Modia3D.Revolute(", ModiaMath.fullName(obj1), ", ", ModiaMath.fullName(obj2), ", ...):\n",
                            "The x-axes of the two frames do not coincide.\n",
                            "( angle(xaxis(obj1), xaxis(obj2)) = ", string(), " > ", Basics.neps, ")")
                     end
             end
         end
      else
         R2_abs     = rotationMatrixAxis(axis, axis > 0 ? rphi_start : -rphi_start)*obj_a.R_abs
         diff_R_abs = norm(R2_abs*obj_b.R_abs' - ModiaMath.NullRotation)
         if diff_R_abs > Basics.neps
             @static if VERSION >= v"0.7.0-DEV.2005"
                 @warn begin
                     name_obj1 = ModiaMath.fullName(obj1)
                     name_obj2 = ModiaMath.fullName(obj2)
                     "Warning from Modia3D.Revolute($name_obj1, $name_obj2, ....):\n" *
                     "The orientations of the provided frames do not coincide.\n" *
                     "( norm(R2_abs*obj2.R_abs' - EYE3()) = $diff_R_abs > $(Basics.neps) )"
                 end
             else
                 warn("Warning from Modia3D.Revolute(", ModiaMath.fullName(obj1), ", ", ModiaMath.fullName(obj2), ", ...):\n",
                      "The orientations of the provided frames do not coincide.\n",
                      "( norm(R2_abs*obj2.R_abs' - EYE3()) = ", string(diff_R_abs), " > ", Basics.neps,")")
             end
         end
      end

      # Define variables of cut-joint
      joint      = CutJointRevolute(ModiaMath.ComponentInternal(), obj_a, obj_b, false, axis)
      lambda_x   = ModiaMath.RealScalar("lambda_x"  , joint, unit="N", info="Constraint force in x-direction"   , numericType=ModiaMath.LAMBDA, analysis=ModiaMath.QuasiStaticAndDynamicAnalysis)
      lambda_y   = ModiaMath.RealScalar("lambda_y"  , joint, unit="N", info="Constraint force in y-direction"   , numericType=ModiaMath.LAMBDA, analysis=ModiaMath.QuasiStaticAndDynamicAnalysis)
      lambda_z   = ModiaMath.RealScalar("lambda_z"  , joint, unit="N", info="Constraint force in z-direction"   , numericType=ModiaMath.LAMBDA, analysis=ModiaMath.QuasiStaticAndDynamicAnalysis)
      mue_x      = ModiaMath.RealScalar("mue_x"     , joint,           info="Stabilizing mue in x-direction"    , numericType=ModiaMath.MUE   , analysis=ModiaMath.OnlyDynamicAnalysis)
      mue_y      = ModiaMath.RealScalar("mue_y"     , joint,           info="Stabilizing mue in y-direction"    , numericType=ModiaMath.MUE   , analysis=ModiaMath.OnlyDynamicAnalysis)
      mue_z      = ModiaMath.RealScalar("mue_z"     , joint,           info="Stabilizing mue in z-direction"    , numericType=ModiaMath.MUE   , analysis=ModiaMath.OnlyDynamicAnalysis)
      if abs(axis) != 1
         residue_x  = ModiaMath.RealScalar("residue_x" , joint,           info="Position constraint in x-direction", numericType=ModiaMath.FC)
         residue_vx = ModiaMath.RealScalar("residue_vx", joint,           info="Velocity constraint in x-direction", numericType=ModiaMath.FC    , analysis=ModiaMath.OnlyDynamicAnalysis)
      elseif abs(axis) != 2
         residue_y  = ModiaMath.RealScalar("residue_y" , joint,           info="Position constraint in y-direction", numericType=ModiaMath.FC)
         residue_vy = ModiaMath.RealScalar("residue_vy", joint,           info="Velocity constraint in y-direction", numericType=ModiaMath.FC    , analysis=ModiaMath.OnlyDynamicAnalysis)
      elseif abs(axis) != 3
         residue_z  = ModiaMath.RealScalar("residue_z" , joint,           info="Position constraint in z-direction", numericType=ModiaMath.FC)
         residue_vz = ModiaMath.RealScalar("residue_vz", joint,           info="Velocity constraint in z-direction", numericType=ModiaMath.FC    , analysis=ModiaMath.OnlyDynamicAnalysis)
      end
      # Add joint to the frames
      push!(obj_a.twoObject3Dobject, joint)
      push!(obj_b.twoObject3Dobject, joint)
      obj_a.hasCutJoint = true
      obj_b.hasCutJoint = true
   else
      # Joint in the spanning tree
      if isnan(rphi_start)
         error("\nerror from Modia3D.Revolute(", ModiaMath.fullName(obj1), ", ", ModiaMath.fullName(obj2), ", phi_start=NaN):\n",
              "Argument phi_start = NaN (Not-a-Number). However, joint is not allowed if the joint is in the spanning tree")
      end
      joint   = TreeJointRevolute(ModiaMath.ComponentInternal(), obj_a, obj_b, canCollide, false, axis)
      phi     = ModiaMath.RealScalar("phi"    , joint, unit="rad"    , start=rphi_start, fixed=true, info="Relative rotation angle"      , numericType=ModiaMath.XD_EXP)
      w       = ModiaMath.RealScalar("w"      , joint, unit="rad/s"  , start=rw_start  , fixed=true, info="Relative angular velocity"    , numericType=ModiaMath.XD_IMP    , integral=phi, analysis=ModiaMath.OnlyDynamicAnalysis)
      a       = ModiaMath.RealScalar("a"      , joint, unit="rad/s^2", start=0.0       ,             info="Relative angular acceleration", numericType=ModiaMath.DER_XD_IMP, integral=w  , analysis=ModiaMath.OnlyDynamicAnalysis)
      tau     = ModiaMath.RealScalar("tau"    , joint, unit="Nm"     , start=0.0       ,             info="Driving torque"               , numericType=ModiaMath.WR                      , analysis=ModiaMath.QuasiStaticAndDynamicAnalysis)
      residue = ModiaMath.RealScalar("residue", joint,                 start=0.0       ,             info="Revolute residue"             , numericType=ModiaMath.FD_IMP                  , analysis=ModiaMath.OnlyDynamicAnalysis)
      #P_w     = ModiaMath.RealScalar("P_w"    , joint, unit="J", info="Total power computed with interpolated x,derx, values (should be zero)", numericType=ModiaMath.WC, analysis=ModiaMath.OnlyDynamicAnalysis)

      joint.flange = RevoluteFlange()

      obj_a.hasChildJoint = true
      obj_b.joint = joint
      obj_b.r_rel = ModiaMath.ZeroVector3D
      obj_b.r_abs = obj_a.r_abs
      obj_b.R_rel = rotationMatrixAxis(axis, axis > 0 ? rphi_start : -rphi_start)
   end
   return joint
end


function rotationMatrixAxis(axis::Int, phi::Number)
   if abs(axis) == 1
      return ModiaMath.rot1(phi)
   elseif abs(axis) == 2
      return ModiaMath.rot2(phi)
   elseif abs(axis) == 3
      return ModiaMath.rot3(phi)
   end
end


function driveJoint!(joint::TreeJointRevolute)
   joint.isDriven = true
   joint.phi.numericType = ModiaMath.WR
   joint.w.numericType   = ModiaMath.WR
   joint.a.numericType   = ModiaMath.WR
end


function setAngle!(joint::TreeJointRevolute, phi::Float64)
   #@assert( joint.isDriven )
   joint.phi.value = phi
   joint.obj2.R_rel = rotationMatrixAxis(joint.axis, joint.axis > 0 ? phi : -phi)
   return joint
end


function computeKinematics!(joint::TreeJointRevolute, obj::Object3D, analysis::ModiaMath.AnalysisType, time::Float64)::NOTHING
   parent::Object3D = obj.parent

   obj.r_abs = parent.r_abs
   obj.R_rel = rotationMatrixAxis(joint.axis, joint.axis > 0 ? joint.phi.value : -joint.phi.value)
   obj.R_abs = obj.R_rel*parent.R_abs

   if analysis == ModiaMath.DynamicAnalysis
      dynamics::Object3Ddynamics       = obj.dynamics
      parentDynamics::Object3Ddynamics = parent.dynamics

      dynamics.v0 = parentDynamics.v0
      dynamics.a0 = parentDynamics.a0

      w_rel = MVector{3,Float64}(0.0, 0.0, 0.0)
      z_rel = MVector{3,Float64}(0.0, 0.0, 0.0)

      w_rel[abs(joint.axis)] = joint.axis > 0 ? joint.w.value : -joint.w.value
      z_rel[abs(joint.axis)] = joint.axis > 0 ? joint.a.value : -joint.a.value

      dynamics.w = obj.R_rel*(parentDynamics.w + w_rel)
      dynamics.z = obj.R_rel*(parentDynamics.z + z_rel + cross(parentDynamics.w, w_rel))
   end
   return nothing
end


function computeForceTorqueAndResidue!(joint::TreeJointRevolute, obj::Object3D, analysis::ModiaMath.AnalysisType, time::Float64)::NOTHING
   parent::Object3D                 = obj.parent
   dynamics::Object3Ddynamics       = obj.dynamics
   parentDynamics::Object3Ddynamics = parent.dynamics
   parentDynamics.f    += obj.R_rel'*dynamics.f
   parentDynamics.t    += obj.R_rel'*dynamics.t
   dynamics.f          = -dynamics.f
   dynamics.t          = -dynamics.t

   joint.residue.value = -joint.tau.value + (joint.axis > 0 ? dynamics.t[abs(joint.axis)] : -dynamics.t[abs(joint.axis)])

   # For checking: compute total power
   # P = dot(parentDynamics.f, joint.obj1.R_abs*parentDynamics.v0) +
   #    dot(dynamics.f      , joint.obj2.R_abs*dynamics.v0) +
   #    dot(parentDynamics.t, parentDynamics.w) +
   #    dot(dynamics.t      , dynamics.w) -
   #    joint.tau.value*joint.w.value
   #
   # joint.P_w.value = P

   return nothing
end


function computePositionResidues!(joint::CutJointRevolute, time::Float64)
   r_rel12 = joint.obj1.R_abs*(joint.obj2.r_abs - joint.obj1.r_abs)
   if abs(joint.axis) == 3
      joint.residue_x.value = r_rel12[1]
      joint.residue_y.value = r_rel12[2]
   elseif abs(joint.axis) == 2
      joint.residue_x.value = r_rel12[1]
      joint.residue_z.value = r_rel12[3]
   elseif abs(joint.axis) == 1
      joint.residue_y.value = r_rel12[2]
      joint.residue_z.value = r_rel12[3]
   end
   return nothing
end

function computeVelocityResidues!(joint::CutJointRevolute, time::Float64)
   v_rel12 = joint.obj1.R_abs*(joint.obj2.dynamics.v0 - joint.obj1.dynamics.v0)
   if abs(joint.axis) == 3
      joint.residue_vx.value = v_rel12[1]
      joint.residue_vy.value = v_rel12[2]
   elseif abs(joint.axis) == 2
      joint.residue_vx.value = v_rel12[1]
      joint.residue_vz.value = v_rel12[3]
   elseif abs(joint.axis) == 1
      joint.residue_vy.value = v_rel12[2]
      joint.residue_vz.value = v_rel12[3]
   end
   return nothing
end

function computeCutForcesAndToques!(joint::CutJointRevolute, time::Float64)
   if abs(joint.axis) == 3
      dynamics1.f += SVector{3,Float64}(joint.lambda_x, joint.lambda_y, 0.0)
   elseif abs(joint.axis) == 2
      dynamics1.f += SVector{3,Float64}(joint.lambda_x, 0.0, joint.lambda_z)
   elseif abs(joint.axis) == 1
      dynamics1.f += SVector{3,Float64}(0.0, joint.lambda_y, joint.lambda_z)
   end
   dynamics2.f += joint.obj2.R_abs*(joint.obj1.R_abs'*dynamics1.f)
end


function addToFlowOrPotentialArray(variable::ModiaMath.RealScalar, assembly::Modia3D.Composition.AssemblyInternal)
  if variable.flow
    if variable.causality == ModiaMath.Input
      push!(assembly.flowVarInput, variable)
    elseif variable.causality == ModiaMath.Output
      push!(assembly.flowVarOutput, variable)
    end
  else
    if variable.causality == ModiaMath.Input
      push!(assembly.potentialVarInput, variable)
    elseif variable.causality == ModiaMath.Output
      push!(assembly.potentialVarOutput, variable)
    end
  end
end


function checkLengthArrays(assembly::Modia3D.Composition.AssemblyInternal)
  @assert(length(assembly.potentialVarOutput) == length(assembly.potentialVarInput))
  @assert(length(assembly.flowVarOutput) == length(assembly.flowVarInput))
end


function addVariableToArray(flange_val::ModiaMath.RealScalar, joint_val::ModiaMath.RealScalar, assembly::Modia3D.Composition.AssemblyInternal)
  # println("flange_val.analysis = ", flange_val.analysis, " revFlange_val.analysis = ", revFlange_val.analysis, " joint_val.analysis = ", joint_val.analysis)
  if flange_val.causality != ModiaMath.Local && joint_val.causality != ModiaMath.Local
    addToFlowOrPotentialArray(flange_val, assembly)
    addToFlowOrPotentialArray(joint_val, assembly)
    checkLengthArrays(assembly)
  end
end


function assignRevoluteFlange(flange::Flange, revFlange::RevoluteFlange, joint::Modia3D.AbstractRevolute, assembly::Modia3D.Composition.AssemblyInternal)
  names = fieldnames(typeof(flange))
  for val in names
    flange_val    = getfield(flange, val)
    revFlange_val = getfield(revFlange,val)
    joint_val     = getfield(joint,val)
    if causalitiesAreOk(flange_val, joint_val)
        setFlangeVariable!(flange_val, revFlange_val)
        setJointVariable!(joint_val, revFlange_val)
        addVariableToArray(flange_val, joint_val, assembly)
    else
      return false
    end
  end
  return true
end


function addTorqueObjToJoint(torqueObj::Modia3D.AbstractForceTorque, SymbFlangeTorque::Symbol, joint::Modia3D.AbstractRevolute, SymbFlangeJoint::Symbol, assembly::Modia3D.AbstractAssembly)
  flangeTorque  = getfield(torqueObj, SymbFlangeTorque)
  flangeJoint   = getfield(joint, SymbFlangeJoint)
  if !assignRevoluteFlange(flangeTorque, flangeJoint, joint, assembly)
    error("The causalities are different. Please, check all flanges which are connected with joint = ", joint ,".")
  end
  if !isPhiInput(flangeJoint) && isTauInput(flangeJoint)
    addRevoluteTorqueObject(joint, torqueObj, SymbFlangeTorque)
    joint.isDriven = false
  elseif isPhiInput(flangeJoint) && !isTauInput(flangeJoint)
    driveJoint!(joint)
  end
end


function connect(torqueObj::Modia3D.AbstractForceAdaptor, joint::Modia3D.AbstractRevolute)
  actualAssembly = torqueObj._internal.within._internal

  flangeTorque  = torqueObj.flange
  flangeJoint   = joint.flange

  if !assignRevoluteFlange(flangeTorque, flangeJoint, joint, actualAssembly) # !assignRevoluteFlangeAndSignal(signal, flangeSig, joint, flangeJoint, assembly)
    error("The causalities are different. Please, check all flanges which are connected with joint = ", joint ,".")
  end
  if !in(torqueObj.forceElement, actualAssembly.uniqueForceTorques)
    push!(actualAssembly.uniqueForceTorques, torqueObj.forceElement)
  end
  if !isPhiInput(flangeJoint) && isTauInput(flangeJoint)
    joint.isDriven = false
  elseif isPhiInput(flangeJoint) && !isTauInput(flangeJoint)
      driveJoint!(joint)
  end
end
connect(joint::Modia3D.AbstractRevolute, torqueObj::Modia3D.AbstractForceAdaptor) = connect(torqueObj::Modia3D.AbstractForceAdaptor, joint::Modia3D.AbstractRevolute)

computeTorque(obj::Any, phi::Float64, w::Float64, time::Float64) = error("Modia3D.Revolute: Missing implementation for function Modia3D.computeTorque")


function connect(signal::Modia3D.AbstractSignalAdaptor, joint::Modia3D.AbstractRevolute)
    actualAssembly = signal._internal.within._internal

    flangeSig   = signal.flange
    flangeJoint = joint.flange

    if !assignRevoluteFlange(flangeSig, flangeJoint, joint, actualAssembly) # !assignRevoluteFlangeAndSignal(signal, flangeSig, joint, flangeJoint, assembly)
      error("The causalities are different. Please, check all flanges which are connected with joint = ", joint ,".")
    end
    if !in(signal.signal, actualAssembly.uniqueSignals)
      push!(actualAssembly.uniqueSignals, signal.signal)
    end
    if !isPhiInput(flangeJoint) && isTauInput(flangeJoint)
      joint.isDriven = false
    elseif isPhiInput(flangeJoint) && !isTauInput(flangeJoint)
        driveJoint!(joint)
    end
end
connect(joint::Modia3D.AbstractRevolute, signal::Modia3D.AbstractSignalAdaptor) = connect(signal::Modia3D.AbstractSignalAdaptor, joint::Modia3D.AbstractRevolute)

computeSignal(obj::Any, time::Float64, tStart::Float64) = error("Modia3D.Revolute: Missing implementation for function Modia3D.computeSignal.")
