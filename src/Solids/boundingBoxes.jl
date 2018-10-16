# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module 
#   Modia3D.Solids (Modia3D/Solids/_module.jl)
#

#------------------------------------------------------------------------------------
# supportPoint(geo,r_abs,R_abs,e) - Return support point of geometry geo at position r_abs/R_abs in direction of unit vector e
# function is called from mpr algorithm
supportPoint(geo::Modia3D.AbstractSolidGeometry, r_abs::AbstractVector, R_abs::AbstractMatrix, e::AbstractVector) =
            r_abs + R_abs'*centroid(geo) + R_abs'*supportPoint_abs(geo, R_abs*e) + e*geo.rsmall

supportPoint(geo::SolidSphere, r_abs::AbstractVector, R_abs::AbstractMatrix, e::AbstractVector) = r_abs + (geo.Dx/2)*e

supportPoint(geo::SolidEllipsoid, r_abs::AbstractVector, R_abs::AbstractMatrix, e::AbstractVector) =
            r_abs + R_abs'*supportPoint_abs(geo, R_abs*e)

supportPoint(geo::SolidFileMesh, r_abs::AbstractVector, R_abs::AbstractMatrix, e::AbstractVector) =
            r_abs + R_abs'*supportPoint_abs(geo, R_abs*e)


#------------------------------------------------------------------------------------
# Otter: Nomenclature seems not good. Better:
#          supportPoint_abs -> supportPoint_ref
#          e_abs            -> e_ref
#
# supportPoint_abs(geo,e_abs) - Return support point of geometry geo at the reference frame of geo in direction of unit vector e_abs (resolved in geo reference frame)
# see e.g. [Gino v.d. Bergen, Collision Detection in Interactive 3D Environments, 2003]
# [G. Snethen, Xenocollide: Complex collision made simple. In Game Programming Gems 7, 2008]

# [Gino v.d. Bergen, p. 135]
function supportPoint_abs(geo::SolidEllipsoid, e_abs::AbstractVector)
   return [(geo.Dx/2)^2*e_abs[1],(geo.Dy/2)^2*e_abs[2],(geo.Dz/2)^2*e_abs[3]]/norm([(geo.Dx/2)*e_abs[1],(geo.Dy/2)*e_abs[2],(geo.Dz/2)*e_abs[3]])
end

# [Gino v.d. Bergen, p. 135]
supportPoint_abs(geo::SolidBox, e_abs::AbstractVector) = [Basics.sign_eps(e_abs[1])*geo.Lx/2, Basics.sign_eps(e_abs[2])*geo.Ly/2, Basics.sign_eps(e_abs[3])*geo.Lz/2]

# [Gino v.d. Bergen, p. 136, XenoCollide, p. 168, 169]
supportPoint_abs(geo::SolidCylinder, e_abs::AbstractVector) = norm([e_abs[1],e_abs[2]]) <= Modia3D.neps ?
                Basics.sign_eps(e_abs[3])*[0.0,0.0,geo.Lz/2] :
                Basics.sign_eps(e_abs[3])*[0.0,0.0,geo.Lz/2] + [(geo.Dx/2)*e_abs[1],(geo.Dy/2)*e_abs[2],0.0]/norm([e_abs[1],e_abs[2]])

# A. Neumayr: Cylinder + Ellipsoid as bottom and top
function supportPoint_abs(geo::SolidCapsule, e_abs::AbstractVector)
  if norm([(geo.Dx/2)*e_abs[1],(geo.Dy/2)*e_abs[2],(geo.Dy/2)*e_abs[3]]) <= Modia3D.neps
    return Basics.sign_eps(e_abs[3])*[0.0,0.0,geo.Lz/2]
  else
    return Basics.sign_eps(e_abs[3])*[0.0,0.0,geo.Lz/2] + [(geo.Dx/2)^2*e_abs[1],(geo.Dy/2)^2*e_abs[2],(geo.Dy/2)^2*e_abs[3]]/norm([(geo.Dx/2)*e_abs[1],(geo.Dy/2)*e_abs[2],(geo.Dy/2)*e_abs[3]])
  end
end

# A. Neumayr: Cuboid + arc of a circle in x dimension
function supportPoint_abs(geo::SolidBeam, e_abs::AbstractVector)
  return [Basics.sign_eps(e_abs[1])*geo.Lx/2 , Basics.sign_eps(e_abs[2])*geo.Dy/2, Basics.sign_eps(e_abs[3])*geo.Lz/2] + [ (geo.Dy/2)*Basics.sign_eps(e_abs[1]), 0.0, 0.0] #/norm([e_abs[1]])
end

# for cone: [Gino v.d. Bergen, p. 136]]
# for frustum of a cone: A. Neumayr
function supportPoint_abs(geo::SolidCone, e_abs::AbstractVector)
  radius_x = geo.Dx/2
  eta = geo.Lz/2
  sin_phi_x = radius_x/sqrt(radius_x^2 + (2*eta)^2)
  radius_y = geo.Dy/2
  sin_phi_y = radius_y/sqrt(radius_y^2 + (2*eta)^2)

  value = e_abs[3]/norm([e_abs[1],e_abs[2],e_abs[3]])

  if geo.extras[1] == 0.0 # total cone
    if      value >= sin_phi_x  && value >= sin_phi_y  # apex is support point
      return [0.0,0.0,eta]
    end
  else # frustum of a cone
    if      value >= sin_phi_x && value >= sin_phi_y && norm([e_abs[1],e_abs[2]]) <= Modia3D.neps   # apex is support point
      return [0.0,0.0,eta]
    elseif  value >= sin_phi_x && value >= sin_phi_y && norm([e_abs[1],e_abs[2]]) > Modia3D.neps # top surface is a smaller ellipse
      return ([0.0,0.0,eta]  + [(geo.Dx/2*geo.extras[1])*e_abs[1],(geo.Dy/2*geo.extras[1])*e_abs[2],0.0]/norm([e_abs[1],e_abs[2]]))
    end
  end
  if value < sin_phi_x && value < sin_phi_y && norm([e_abs[1],e_abs[2]]) > Modia3D.neps        # bottom surface is a bigger ellipse
    return ([0.0,0.0,-eta] + [(geo.Dx/2)*e_abs[1],(geo.Dy/2)*e_abs[2],0.0]/norm([e_abs[1],e_abs[2]]))
  else
    return  [0.0,0.0,-eta]
  end
end

# see Cylinder
supportPoint_abs(geo::SolidPipe, e_abs::AbstractVector) = norm([e_abs[1],e_abs[2]]) <= Modia3D.neps ?
                Basics.sign_eps(e_abs[3])*[0.0,0.0,geo.Lz/2] :
                Basics.sign_eps(e_abs[3])*[0.0,0.0,geo.Lz/2] + [(geo.Dx/2)*e_abs[1],(geo.Dy/2)*e_abs[2],0.0]/norm([e_abs[1],e_abs[2]])

# [Gino v.d. Bergen, p. 131]
function supportPoint_abs(geo::SolidFileMesh, e_abs::AbstractVector)
  # error("SolidFileMesh: function missing das haben wir noch nicht")
  max_value = Float64
  position = Float64
  for i in eachindex(geo.objPoints)
    if i == 1
      max_value = dot(geo.objPoints[i],e_abs)
      position = i
    else
      act_value = dot(geo.objPoints[i],e_abs)
      if act_value > max_value
        max_value = act_value
        position = i
      end
    end
  end
  return geo.objPoints[position]
end


#------------------------------------------------------------------------------------
# boundingBox!(..) calculates the AABB (= Axis Aligned Bounding Box) for the geometries
# it also uses support points for finding the points which are furthest away in each x,y,z - direction
# it calles supportPoint_i
function boundingBox!(geo::Modia3D.AbstractSolidGeometry, AABB::Basics.BoundingBox, r_abs::AbstractVector, R_abs::AbstractMatrix; tight::Bool=true, scaleFactor::Float64=0.1)
  xmin = supportPoint_i(geo, r_abs[1], R_abs[:,1], -1)
  xmax = supportPoint_i(geo, r_abs[1], R_abs[:,1], +1)
  ymin = supportPoint_i(geo, r_abs[2], R_abs[:,2], -1)
  ymax = supportPoint_i(geo, r_abs[2], R_abs[:,2], +1)
  zmin = supportPoint_i(geo, r_abs[3], R_abs[:,3], -1)
  zmax = supportPoint_i(geo, r_abs[3], R_abs[:,3], +1)

  if tight    # best fitting AABB which is possible
   AABB.x_min = xmin
   AABB.x_max = xmax
   AABB.y_min = ymin
   AABB.y_max = ymax
   AABB.z_min = zmin
   AABB.z_max = zmax
 else
   # scales each axis with scaleFactor 0.1 means 10 percent bigger than best fitting AABB and so on
   @assert(scaleFactor > 0.0)
   scale = max(abs(xmax - xmin)*scaleFactor/2,abs(ymax - ymin)*scaleFactor/2,abs(zmax - zmin)*scaleFactor/2)
   AABB.x_min = xmin - scale
   AABB.x_max = xmax + scale
   AABB.y_min = ymin - scale
   AABB.y_max = ymax + scale
   AABB.z_min = zmin - scale
   AABB.z_max = zmax + scale
 end
 return AABB
 #=
  # other idea for AABB
  # scales each axis with scaleFactor 0.1 means 10 percent bigger than best fitting AABB and so on
  scaleX = abs(xmax - xmin)*scaleFactor/2
  scaleY = abs(ymax - ymin)*scaleFactor/2
  scaleZ = abs(zmax - zmin)*scaleFactor/2
  AABB.x_min = xmin - scaleX
  AABB.x_max = xmax + scaleX
  # so on

  # FileMesh
   longestEdge = max(geo.scaleFactor[1], geo.scaleFactor[2], geo.scaleFactor[3]) * geo.longestEdge * sqrt(2)
   x_min = -shape.longestEdge + r_rel[1]
   x_max =  shape.longestEdge + r_rel[1]
   y_min = -shape.longestEdge + r_rel[2]
   y_max =  shape.longestEdge + r_rel[2]
   z_min = -shape.longestEdge + r_rel[3]
   z_max = shape.longestEdge + r_rel[3]
 =#
end


function boundingBox!(geo::SolidSphere, AABB::Basics.BoundingBox, r_abs::AbstractVector, R_abs::AbstractMatrix; tight::Bool=true, scaleFactor::Float64=0.1)
  r = geo.Dx/2
  if tight    # best fitting AABB which is possible
    AABB.x_min = r_abs[1] - r
    AABB.x_max = r_abs[1] + r
    AABB.y_min = r_abs[2] - r
    AABB.y_max = r_abs[2] + r
    AABB.z_min = r_abs[3] - r
    AABB.z_max = r_abs[3] + r
  else
    # scales each axis with scaleFactor 0.1 means 10 percent bigger than best fitting AABB and so on
    @assert(scaleFactor > 0.0)
    scale = r*scaleFactor/2
    AABB.x_min = r_abs[1] - r - scale
    AABB.x_max = r_abs[1] + r + scale
    AABB.y_min = r_abs[2] - r - scale
    AABB.y_max = r_abs[2] + r + scale
    AABB.z_min = r_abs[3] - r - scale
    AABB.z_max = r_abs[3] + r + scale
  end
  return AABB
end

#------------------------------------------------------------------------------------
# supportPoint_i are utility functions for boundingBox!(..)
# it is based on supportPoint_abs (its nearly the same than supportPoint(..))
supportPoint_i(geo::Modia3D.AbstractSolidGeometry, r_absi::Float64, R_absi::AbstractVector, isign::Int) =
               r_absi + R_absi'*centroid(geo) + R_absi'*supportPoint_abs(geo, isign*R_absi) + isign*geo.rsmall

supportPoint_i(geo::SolidEllipsoid, r_absi::Float64, R_absi::AbstractVector, isign::Int) =
               r_absi + dot(R_absi, supportPoint_abs(geo, isign*R_absi))

supportPoint_i(geo::SolidFileMesh, r_absi::Float64, R_absi::AbstractVector, isign::Int) =
               r_absi + dot(R_absi, supportPoint_abs(geo, isign*R_absi))
