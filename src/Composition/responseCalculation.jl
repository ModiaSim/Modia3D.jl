# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#

function responseCalculation(cM1::Solids.ContactMaterialElastic,
                             cM2::Solids.ContactMaterialElastic, 
                             obj1::Object3D,
                             obj2::Object3D,
                             s::Float64,
                             rContact::ModiaMath.Vector3D,
                             e_n::ModiaMath.Vector3D,
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

   # println("... 1: ", ModiaMath.instanceName(obj1), ", 2: ", ModiaMath.instanceName(obj2), ", s = ", s, ", e_n = ", e_n, ", e_t = ", e_t, ", rContact = ", rContact, ", f_n = ", f_n, ", f_t = ", f_t)

   return (f1,f2,t1,t2)
end