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
   #v_small = (cM1.v_small + cM2.v_small)/2
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
   #friction    = abs_v_rel_t > v_small
   #e_t         = v_rel_t/(friction ? abs_v_rel_t : v_small)
   w_rel_n     = dot(w2 - w1,e_n)

   # Contact forces/torques
   f_c  = c*s
   f_d  = d*v_rel_n
   f_cd = f_c + f_d
   f_n  = f_cd >= 0.0 ? 0.0 : f_cd
   abs_f_n = abs(f_n)
   f_t = (mu1*abs_f_n)*v_rel_t #/abs(v_rel_t)
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
                             collMaterial::Union{Modia3D.AbstractContactMaterial, NOTHING},
                             time,
                             file)::Tuple{ModiaMath.Vector3D,ModiaMath.Vector3D,ModiaMath.Vector3D,ModiaMath.Vector3D}

### computing constans out of material definition ###
  if typeof(delta_dot_initial) == NOTHING
  #  error("delta_dot_initial not defined")
  end
  #println("delta_dot_initial ", delta_dot_initial)

  # println("delta_dot_initial = ", delta_dot_initial)
  # v_small  ... small velocity used for regularization
  # w_small  ... small angular velocity used for regularization
  # cor_res  ... resulting coefficient of restitution
  # d_res    ... resulting damping material constant in normal direction d_res = (cM1.d + cM2.d)/2
  # c_res    ... elastic material constant in normal direction
  # mu_k_res ... sliding friction coefficient in tangential direction
  # mu_r_res ... rotational friction torque coefficient
  v_small = (cM1.v_small + cM2.v_small)/2
  w_small = (cM1.w_small + cM2.w_small)/2
  # cor_res  = Modia3D.resultantCoefficientOfRestitution(cM1.cor, cM2.cor, abs(delta_dot_initial), v_small) # glg (2), (3)
  d_res    = Modia3D.resultantDampingCoefficient(cM1.cor, cM2.cor, abs(delta_dot_initial), v_small) # glg (4)
  c_res    = cM1.c*cM2.c/(cM1.c + cM2.c)
  mu_k_res = min(cM1.mu_k, cM2.mu_k)
  mu_r_res = min(cM1.mu_r, cM2.mu_r)
#  println("mu_k_res = $mu_k_res")
#  println("mu_r_res = $mu_r_res")

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
  w_rel = w2 - w1

  # delta_dot ... signed relative velocity in normal direction
  # v_t       ... relative velocity vector in tangential direction
  # delta     ... signed distance
  delta_dot = dot(v_rel,e_n)
  v_t       = v_rel - delta_dot*e_n
  delta     = -s #abs(s)

  # compute normal force and tangential force
  r  = getCommonRadius(obj1, obj2)
  fn = normalForce(c_res, d_res, delta, delta_dot, r)
  ft = tangentialForce(fn, mu_k_res, v_t, v_small)
  #println("... fn = ", fn, ", delta = ", delta, ", delta_dot = ", delta_dot, ", c_res= ", c_res)
  f1 = fn*e_n + ft
  f2 = -f1
  tau = compTau(fn, mu_r_res, w_rel, w_small, r)
  #println("tau = ", tau)
  t1 = cross(r_rel1,f1) + tau
  t2 = cross(r_rel2,f2) - tau
  return (f1,f2,t1,t2)
end


function getCommonRadius(obj1::Object3D, obj2::Object3D)
  if typeof(obj1.data.geo) == Modia3D.Solids.SolidSphere && typeof(obj2.data.geo) == Modia3D.Solids.SolidSphere
    r1 = obj1.data.geo.Dx*0.5
    r2 = obj2.data.geo.Dx*0.5
    r = r1*r2/(r1 + r2)
  elseif typeof(obj1.data.geo) == Modia3D.Solids.SolidSphere && typeof(obj2.data.geo) != Modia3D.Solids.SolidSphere
    r = obj1.data.geo.Dx*0.5
  elseif typeof(obj1.data.geo) != Modia3D.Solids.SolidSphere && typeof(obj2.data.geo) == Modia3D.Solids.SolidSphere
    r = obj2.data.geo.Dx*0.5
  else
    r = nothing
  end
  return r
end


normalForce(c_res, d_res, delta, delta_dot::Float64, r::NOTHING) = -c_res*delta*(1-d_res*delta_dot)
normalForce(c_res, d_res, delta, delta_dot::Float64, r::Float64) = -c_res*(4/3*delta*sqrt(r*abs(delta)))*(1-d_res*delta_dot)
tangentialForce(fn, mu_k_res, v_t, v_small) = -mu_k_res*fn*v_t/Modia3D.regularize(norm(v_t),v_small)
compTau(fn, mu_r_res, w_rel, w_small, r::NOTHING) = -mu_r_res*fn*w_rel/Modia3D.regularize(norm(w_rel),w_small)
compTau(fn, mu_r_res, w_rel, w_small, r::Float64) = -r*mu_r_res*fn*w_rel/Modia3D.regularize(norm(w_rel),w_small)


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
  return delta_dot_initial
end



# response calculation with new contact force law
function responseCalculation(pair::CollisionPair,
                             rContact::ModiaMath.Vector3D,
                             time,
                             file)::Tuple{ModiaMath.Vector3D,ModiaMath.Vector3D,ModiaMath.Vector3D,ModiaMath.Vector3D}

  # v_small  ... small velocity used for regularization
  # w_small  ... small angular velocity used for regularization
  # d_res    ... resulting damping material constant in normal direction d_res = (cM1.d + cM2.d)/2
  # c_res    ... elastic material constant in normal direction
  # mu_k_res ... sliding friction coefficient in tangential direction
  # mu_r_res ... rotational friction torque coefficient
  material = pair.contactPairMaterial
  v_small  = material.v_small
  w_small  = material.w_small
  d_res    = material.d_res
  c_res    = material.c_res
  mu_k     = material.mu_k
  mu_r     = material.mu_r

  obj1 = pair.obj1
  obj2 = pair.obj2
  s    = pair.distanceWithHysteresis
  e_n  = pair.contactNormal

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
  w_rel = w2 - w1
  e_w_reg = w_rel/Modia3D.regularize(norm(w_rel),w_small)

  v_rel = v2 - v1

  # delta_dot ... signed relative velocity in normal direction
  # v_t       ... relative velocity vector in tangential direction
  # delta     ... signed distance
  delta_dot = dot(v_rel,e_n)
  v_t       = v_rel - delta_dot*e_n
  e_t_reg   = v_t/Modia3D.regularize(norm(v_t),v_small)

  delta     = -s

  # compute normal force and tangential force
  (c_geo, n_geo, mu_r_geo)  = getCoefficients(obj1, obj2)

  if c_geo != 1.0 && n_geo != 1.0
    delta_comp = delta * sqrt(abs(delta))
  else
    delta_comp = delta
  end
  fn = -max(0.0, c_res * c_geo * delta_comp * (1 - d_res*delta_dot) )
  ft = -mu_k*fn*e_t_reg


  f1 = fn * e_n + ft
  f2 = -f1

  tau = -mu_r * mu_r_geo * fn * e_w_reg

  t1 = cross(r_rel1,f1) + tau
  t2 = cross(r_rel2,f2) - tau
#=
  if time > 0.785 && time < 0.788  && String(ModiaMath.instanceName(obj1)) != "table.box1"
     println("obj1= \"", ModiaMath.instanceName(obj1), "\" obj2 = ", ModiaMath.instanceName(obj2), " fn = ", fn, " en = ", e_n, " delta_ini ", delta_dot_initial," time = ", time, " mu_k = ", mu_k)
  end
=#
  return (f1,f2,t1,t2)
end


function getCoefficients(obj1::Object3D, obj2::Object3D)
  if typeof(obj1.data.geo) == Modia3D.Solids.SolidSphere && typeof(obj2.data.geo) == Modia3D.Solids.SolidSphere
    r1 = obj1.data.geo.Dx*0.5
    r2 = obj2.data.geo.Dx*0.5
    mu_r_geo = r1*r2/(r1 + r2)
    n_geo = 1.5
    c_geo = 4/3*sqrt(mu_r_geo)
  elseif typeof(obj1.data.geo) == Modia3D.Solids.SolidSphere && typeof(obj2.data.geo) != Modia3D.Solids.SolidSphere
    mu_r_geo = obj1.data.geo.Dx*0.5
    n_geo = 1.5
    c_geo = 4/3*sqrt(mu_r_geo)
  elseif typeof(obj1.data.geo) != Modia3D.Solids.SolidSphere && typeof(obj2.data.geo) == Modia3D.Solids.SolidSphere
    mu_r_geo = obj2.data.geo.Dx*0.5
    n_geo = 1.5
    c_geo = 4/3*sqrt(mu_r_geo)
  else
    mu_r_geo = 1.0
    n_geo = 1.0
    c_geo = 1.0
  end
  return (c_geo, n_geo, mu_r_geo)
end
