# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Solids (Modia3D/Solids/_module.jl)
#
# Content:
#
# Properties of contact materials



mutable struct ContactMaterialElastic <: Modia3D.AbstractContactMaterial
   c::Float64      # [N/m]       Spring constant of contact in normal direction (normalForce = c*relPosition)
   d::Float64      # [N*s/m]     Damping constant of contact in normal direction (normalForce = d*relVelocity)
   #d_w::Float64   # [N*m*s/rad] Rotational damping constant in normal direction (frictionTorque = d_w*relAngularVelocity)
   #mu0::Float64   #             Friction force in tangential direction (frictionForce = (mu0+mu1*relVelocity)*normalForce)
   mu1::Float64    #             frictionForce = (mu1*normalForce)*relTangentialVelocity
   #v_min::Float64 # [m/s]       Tangential velocity at which sliding friction force starts
   #w_min::Float64 # [rad/s]     Rel. angular velocity at which sliding friction torque starts

   function ContactMaterialElastic(; c = 1e9, d = 100.0, mu1=0.0)
      @assert(c > 0.0)
      @assert(d >= 0.0)
      #@assert(d_w >= 0.0)
      #@assert(mu0 >= 0.0)
      @assert(mu1 >= 0.0)
      #@assert(v_min > 0.0)
      #@assert(w_min > 0.0)
      new(c, d, mu1)
   end
end

defaultContactMaterial() = ContactMaterialElastic()
