# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#


function responseCalculation(chpairs::ContactPairs,
                             cM1::Solids.ContactMaterialElastic,
                             cM2::Solids.ContactMaterialElastic,
                             obj1::Object3D,
                             obj2::Object3D,
                             s::Float64,
                             rContact::ModiaMath.Vector3D,
                             e_n::ModiaMath.Vector3D,
                             delta_dot_initial::Union{Float64, NOTHING},
                             time,
                             file)::Tuple{ModiaMath.Vector3D,ModiaMath.Vector3D,ModiaMath.Vector3D,ModiaMath.Vector3D}
   # Resultant spring constant (series connection of two springs)
   c = cM1.c*cM2.c/(cM1.c + cM2.c)

   # Other resultant contact data (as mean values)
   d     = (cM1.d     + cM2.d)/2
   #d_w   = (cM1.d_w   + cM2.d_w)/2
   #mu0   = (cM1.mu0   + cM2.mu0)/2
   mu1   = (cM1.mu1   + cM2.mu1)/2
   #v_min = (cM1.v_min + cM2.v_min)/2
   #w_min = (cM1.w_min + cM2.w_min)/2

   # Contact points and distances to local part frame (in world frame)
   r_rel1 = rContact - obj1.r_abs
   r_rel2 = rContact - obj2.r_abs

   # Velocities and angular velocities of contact frames in world frame
   dynamics1 = obj1.dynamics
   dynamics2 = obj2.dynamics
   w1 = obj1.R_abs'*dynamics1.w
   w2 = obj2.R_abs'*dynamics2.w
   v1 = dynamics1.v0 + cross(w1,r_rel1)
   v2 = dynamics2.v0 + cross(w2,r_rel2)

   # Velocities and angular velocities in normal and tangential direction
   v_rel       = v2 - v1
   v_rel_n     = dot(v_rel,e_n)
   v_rel_t     = v_rel - v_rel_n*e_n
   #abs_v_rel_t = norm(v_rel_t)
   #friction    = abs_v_rel_t > v_min
   #e_t         = v_rel_t/(friction ? abs_v_rel_t : v_min)
   w_rel_n     = dot(w2 - w1,e_n)

   # Contact forces/torques
   f_c  = c*s
   f_d  = d*v_rel_n
   f_cd = f_c + f_d
   f_n  = f_cd >= 0.0 ? 0.0 : f_cd
   abs_f_n = abs(f_n)
   f_t = (mu1*abs_f_n)*v_rel_t
   #println(file,"... time = ", time, ", abs_f_n = ", abs_f_n, ", v_rel = ", v_rel, ", v_rel_t = ", v_rel_t, ", f_t1 = ", f_t1)
   #t_n  = (w_rel_n < -w_min ? -1.0 : (w_rel_n >  w_min ? +1.0 : w_rel_n/w_min))*(-d_w)*abs(f_n)
   f1   = f_n*e_n + f_t
   f2   = -f1
   #ta   = t_n*e_n
   t1   = cross(r_rel1,f1)
   t2   = cross(r_rel2,f2)
   #t2 = zeros(3)
   #println("r_rel2 = ",r_rel2)
   #println("f1 = ",f1)
   #println("t1 = ",t1)
   #println("t2 = ",t2)
   # println("... 1: ", ModiaMath.instanceName(obj1), ", 2: ", ModiaMath.instanceName(obj2), ", s = ", s, ", e_n = ", e_n, ", e_t = ", e_t, ", rContact = ", rContact, ", f_n = ", f_n, ", f_t = ", f_t)


   return (f1,f2,t1,t2)
end



# response calculation with new contact force law
function responseCalculation(chpairs::ContactPairs,
                             cM1::Solids.ElasticContactMaterial,
                             cM2::Solids.ElasticContactMaterial,
                             obj1::Object3D,
                             obj2::Object3D,
                             s::Float64,
                             rContact::ModiaMath.Vector3D,
                             e_n::ModiaMath.Vector3D,
                             delta_dot_initial::Union{Float64, NOTHING},
                             time,
                             file)::Tuple{ModiaMath.Vector3D,ModiaMath.Vector3D,ModiaMath.Vector3D,ModiaMath.Vector3D}

### computing constans out of material definition ###
  # c_mean ... elastic material constant in normal direction
  # d_mean ... damping material constant in normal direction
  # mu_mean ... sliding friction coefficient in tangential direction
  # v_min ... absolute value of tangential velocity at which sliding friction force starts
  c_mean = cM1.c*cM2.c/(cM1.c + cM2.c)
  d_mean = (cM1.d + cM2.d)/2
  mu_mean = min(cM1.mu_k, cM2.mu_k)
  v_min = (cM1.v_min + cM2.v_min)/2

### signed velocity and relative velocity ####
  # Contact points and distances to local part frame (in world frame)
  r_rel1 = rContact - obj1.r_abs
  r_rel2 = rContact - obj2.r_abs

  # Velocities and angular velocities of contact frames in world frame
  dynamics1 = obj1.dynamics
  dynamics2 = obj2.dynamics
  w1 = obj1.R_abs'*dynamics1.w
  w2 = obj2.R_abs'*dynamics2.w
  v1 = dynamics1.v0 + cross(w1,r_rel1)
  v2 = dynamics2.v0 + cross(w2,r_rel2)

  # Velocities and angular velocities in normal and tangential direction
  v_rel = v2 - v1
  # delta_dot ... signed relative velocity in normal direction
  delta_dot = dot(v_rel,e_n)

  # v_t ... relative velocity vector in tangential direction
  v_t = v_rel - delta_dot*e_n


  # delta ... signed distance
  delta = abs(s)
  fn = normalForce(obj1, obj2, c_mean, d_mean, delta, delta_dot, delta_dot_initial)
  ft = tangentialForce(fn, mu_mean, v_t, v_min)
#  println("e_n = ", e_n)
#  println("v_t = ", v_t)
  #ft = tangentialForce(fn, mu_mean, v_t)

  f1 = fn*e_n + ft
  f2 = -f1
  println("f1 = $f1")
  t1 = cross(r_rel1,f1)
  t2 = cross(r_rel2,f2)
  return (f1,f2,t1,t2)
end


normalForce(obj1, obj2, c_mean, d_mean, delta, delta_dot::Float64, delta_dot_initial::NOTHING) =  error("delta_dot_initial is not defined!!!")

function normalForce(obj1, obj2, c_mean, d_mean, delta, delta_dot::Float64, delta_dot_initial::Float64)
  if typeof(obj1.data.geo) != Modia3D.Solids.SolidSphere && typeof(obj2.data.geo) != Modia3D.Solids.SolidSphere
    fn = max(0.0, c_mean*delta*(1+d_mean*delta_dot/delta_dot_initial) )
  else
    if typeof(obj1.data.geo) == Modia3D.Solids.SolidSphere && typeof(obj2.data.geo) == Modia3D.Solids.SolidSphere
      r1 = obj1.data.geo.Dx*0.5
      r2 = obj2.data.geo.Dx*0.5
      r = r1*r2/(r1 + r2)
    elseif typeof(obj1.data.geo) == Modia3D.Solids.SolidSphere && typeof(obj2.data.geo) != Modia3D.Solids.SolidSphere
      r1 = obj1.data.geo.Dx*0.5
      r = r1
    elseif typeof(obj1.data.geo) != Modia3D.Solids.SolidSphere && typeof(obj2.data.geo) == Modia3D.Solids.SolidSphere
      r2 = obj2.data.geo.Dx*0.5
      r = r2
    end
    fn =  max(0.0, c_mean*(4/3*delta*sqrt(r*delta))*(1+d_mean*delta_dot/delta_dot_initial) )
  end
  return -fn
end


function tangentialForce(fn, mu_mean, v_t, v_min)
  norm_v_t = norm(v_t)
  k = -log(0.01)/v_min   # is natural logramithm ln(..)
  ft = mu_mean*fn*v_t/(norm_v_t + v_min*MathConstants.e^(-norm_v_t*k) )

  # println("ft = ", ft)
  return -ft
end

function tangentialForce(fn, mu_mean, v_t)
  ft = mu_mean*fn*v_t/norm(v_t)
  return ft
end


function computeDeltaDotInitial(obj1::Object3D, obj2::Object3D,
                             rContact::ModiaMath.Vector3D, e_n::ModiaMath.Vector3D)
  r_rel1 = rContact - obj1.r_abs
  r_rel2 = rContact - obj2.r_abs
  dynamics1 = obj1.dynamics
  dynamics2 = obj2.dynamics
  w1 = obj1.R_abs'*dynamics1.w
  w2 = obj2.R_abs'*dynamics2.w
  v1 = dynamics1.v0 + cross(w1,r_rel1)
  v2 = dynamics2.v0 + cross(w2,r_rel2)
  v_rel = v2 - v1
  delta_dot_initial = min(-0.001, dot(v_rel,e_n))
  println("delta_dot_initial = ", delta_dot_initial)
  return delta_dot_initial
end
