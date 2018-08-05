# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#



# Measurements

""" 
    angle = planarRotationAngle(e, v1, v2)
          = planarRotationAngle(frame1, frame2)

Return angle of a planar rotation, given the rotation axis e (a unit vector)
and the representations of a vector in frame 1 (v1) and frame 2 (v2).

Under the assumption that the z-axes of frame1 and frame2 coincide, return the
angle between the x-axis of frame1 and the position vector from frame1 to frame2.
"""
planarRotationAngle(e::AbstractVector, v1::AbstractVector, v2::AbstractVector) = 
     atan2( dot(-cross(e,v1), v2), dot(v1,v2) - dot(e,v1)*dot(e,v2) )


function planarRotationAngle(frame1::Object3D, frame2::Object3D)
   # Extract needed vectors
   e1z = frame1.R_abs[3,:]
   e2z = frame2.R_abs[3,:]
   e1x = frame1.R_abs[1,:]
   r1  = frame1.r_abs
   r2  = frame2.r_abs

   # Check that z-axes of both frames coincide
   angle_z_axes = acos( dot(e1z,e2z) )
   if abs(angle_z_axes) > Basics.neps
      error("\nError from Modia3D.planarRotationAngle(", ModiaMath.fullName(frame1), ", ", ModiaMath.fullName(frame2), "):\n",
                 "The z-axes of the two frames do not coincide.\n",
                 "( angle(zaxis(frame1), zaxis(frame2)) = ", string(angle_z_axes*Basics.radToDeg), " > ", Basics.neps, ")")
   end

   # Angle between frame2.r_abs - frame1.r_abs and x_axis of frame1
   r12 = r2-r1
   if norm(r12) < 100*eps()
      error("\nError from Modia3D.planarRotationAngle(", ModiaMath.fullName(frame1), ", ", ModiaMath.fullName(frame2), "):\n",
              "The origins of the two frames coincide (this is not allowed for this measurement.")
   end
   r12n = r12/norm(r12)
   return atan2(dot(cross(e1x,r12n), e1z), dot(e1x,r12n) )
end


"""
    d = distance(frame1, frame2)

Return the distance between the origin of frame1 and the origin of frame2
"""
distance(frame1::Object3D, frame2::Object3D) = norm(frame2.r_abs - frame1.r_abs)


function distanceAndAngles(frame1::Composition.Object3D, frame2::Composition.Object3D)
   # Extract needed vectors
   e1z = frame1.R_abs[3,:]
   e2z = frame2.R_abs[3,:]
   r1  = frame1.r_abs
   r2  = frame2.r_abs

   # Check that z-axes of both frames are pointing in the same direction
   @assert(abs(dot(e1z,e2z) - 1.0) <= 1e-10)

   # Angle between frame1 and frame2 along z-axis
   rdiff0 = r2-r1
   ey = normalize(rdiff0)

   ey1  = frame1.R_abs*ey
   ey2  = frame2.R_abs*ey 
   ex1b = cross(ey1, e1z)
   ex2b = cross(ey2, e2z)
   ex1a = frame1.R_abs[:,1]
   ex2a = frame2.R_abs[:,1]

   phi1 = planarRotationAngle(e1z, ex1a, ex1b)
   phi2 = planarRotationAngle(e2z, ex2a, ex2b)
   d    = norm(rdiff0)
   return (d,phi1,phi2)
end