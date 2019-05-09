# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.ContactDetectionMPR (Modia3D/contactDetection/ContactDetectionMPR/_module.jl)
#


# Collision detection algorithm based on the MPR algorithm

mutable struct SupportPoint
  p::SVector{3,Float64}    # support point
  n::SVector{3,Float64}    # support normal unit vector
  a::SVector{3,Float64}    # point on shapeA
  b::SVector{3,Float64}    # point on shapeB
  function SupportPoint(p::SVector{3,Float64},n::SVector{3,Float64},a::SVector{3,Float64},b::SVector{3,Float64})
    new(p,n,a,b)
  end
end


function getSupportPoint(shapeA::Modia3D.Composition.Object3D, shapeB::Composition.Object3D, n::SVector{3,Float64})
  a = Modia3D.supportPoint(shapeA.data.geo, shapeA.r_abs, shapeA.R_abs, n)
  b = Modia3D.supportPoint(shapeB.data.geo, shapeB.r_abs, shapeB.R_abs, -n)
  return SupportPoint(a-b,n,a,b)
end


function barycentric(r1::SupportPoint,r2::SupportPoint,r3::SupportPoint,r4::SupportPoint)
  r21 = r2.p - r1.p
  r31 = r3.p - r1.p
  r41 = r4.p - r1.p

  # Determine normal to triangle (and test that r4 is on the triangle plane)
  n = cross(r21,r31)
  e = normalize(n)
  #@assert(abs(dot(e,r41)) <= 1e-10)

  # Determine normals to r21 and e, as well as r31 and e
  n21 = cross(r21,e)
  n31 = cross(r31,e)

  # Determine barycentric coordinates b21, b31:
  #   n21*(r41 = b21*r21 + b31*r31)    ->  n21*r21 = 0
  #   n31*(r41 = b21*r21 + b31*r31)    ->  n31*r31 = 0
  b21 = dot(n31,r41) / dot(n31,r21)
  b31 = dot(n21,r41) / dot(n21,r31)

  r4.a = r1.a + b21*(r2.a - r1.a) + b31*(r3.a - r1.a)
  r4.b = r1.b + b21*(r2.b - r1.b) + b31*(r3.b - r1.b)


  ### only works if r4.n goes through zero
  #=
  b1 = dot(cross(r2.p,r3.p),r4.n)
  b2 = dot(cross(r3.p,r1.p),r4.n)
  b3 = dot(cross(r1.p,r2.p),r4.n)
  b_ges = b1 + b2 + b3

  r4.a .= (b1*r1.a + b2*r2.a + b3*r3.a) / b_ges
  r4.b .= (b1*r1.b + b2*r2.b + b3*r3.b) / b_ges
  =#
end


###########      Phase 1, Minkowski Portal Refinement      ###################

getCentroid(obj::Composition.Object3D, centroidObj::SVector{3,Float64}) = (obj.r_abs + obj.R_abs'*centroidObj)


# checks if centers of shapeA and shapeB are overlapping
# belongs to construction of r0
function checkCentersOfShapesOverlapp(r0::SupportPoint, neps::Float64, shapeA::Composition.Object3D, shapeB::Composition.Object3D)
  if norm(r0.p) <= neps
    error("MPR: Too large penetration (prerequisite of MPR violated). Centers are overlapping. Look at shapeA = ", shapeA, " shapeB = ", shapeB)
  end
end


function checkIfShapesArePlanar(r0::SupportPoint,r1::SupportPoint,r2::SupportPoint,n2::SVector{3,Float64}, neps::Float64,
                                shapeA::Composition.Object3D,shapeB::Composition.Object3D)
  n3 = cross(r1.p-r0.p, r2.p-r0.p)
  if norm(n3) <= neps
    n2 = -n2
    r2 = getSupportPoint(shapeA, shapeB, n2) # change search direction
    if abs(dot((r2.p-r1.p),n2)) <= neps
      # Shape is purely planar. Computing the shortest distance for a planar shape
      # requires an MPR 2D algorithm (using lines instead of triangles as portals).
      # However, this is not implemented and therefore the shortest distance cannot be computed
      error("MPR: Shapes are planar and MPR2D is not supported. abs(dot((r2.p-r1.p),n2)). Look at shapeA = ", shapeA, " shapeB = ", shapeB)
    end
    n3 = cross(r1.p-r0.p, r2.p-r0.p)   # |n3| > 0 guaranteed, due to construction
  end
  if dot(n3,r0.p) >= neps
    n3 = -n3
  end
  # check if points r1, r2, r3 are on the same line
  r3 = getSupportPoint(shapeA, shapeB, Basics.normalizeVector(n3))
  n3b = cross(r2.p-r1.p, r3.p-r1.p)
  if norm(n3b) <= neps
    r3 = getSupportPoint(shapeA, shapeB, -r3.n) # change search direction
    if abs(dot((r3.p-r1.p),r3.n)) <= neps
      # Shape is purely planar. Computing the shortest distance for a planar shape
      # requires an MPR 2D algorithm (using lines instead of triangles as portals).
      # However, this is not implemented and therefore the shortest distance cannot be computed
      error("MPR: Shapes are planar and MPR2D is not supported. r1, r2, r3 are on the same ray. Look at shapeA = ", shapeA, " shapeB = ", shapeB)
    end
  end
  return (r2, r3, n2, n3)
end


###########      Phase 2, Minkowski Portal Refinement      ###################
# loop around to "ensure" the tetrahedron r0,r1,r2 and r3 encloses the origin
# stimmt so nicht wirklich, muss ich nochmal nachlesen!!!
# Der Ursprung muss nicht enthalten sein!!!
function tetrahedronEncloseOrigin(r0::SupportPoint,r1::SupportPoint,r2::SupportPoint,r3::SupportPoint,
                                  neps::Float64, niter_max::Int64,
                                  shapeA::Composition.Object3D,shapeB::Composition.Object3D)
  aux = SVector(0.0, 0.0, 0.0)
  success = false
  for i in 1:niter_max
    aux = cross(r1.p-r0.p,r3.p-r0.p)
    if dot(aux,r0.p) < -neps
       r2 = r3
       r3 = getSupportPoint(shapeA,shapeB,Basics.normalizeVector(aux))
       continue
    end
    aux = cross(r3.p-r0.p,r2.p-r0.p)
    if dot(aux,r0.p) < -neps
       r1 = r3
       r3 = getSupportPoint(shapeA,shapeB,Basics.normalizeVector(aux))
       continue
    end
    success = true
    break
  end
  if success != true
    error("MPR: Max. number of iterations is reached in phase2. Look at shapeA = ", shapeA, " shapeB = ", shapeB)
  end
  # pointInTetrahedron(r0.p,r1.p,r2.p,r3.p,SVector(0.0, 0.0, 0.0))
  return (r1, r2, r3)
end


###########      Phase 3, Minkowski Portal Refinement      ###################
# construction of r4
function constructR4(r0::SupportPoint,r1::SupportPoint,r2::SupportPoint,r3::SupportPoint,
                     neps::Float64, shapeA::Composition.Object3D,shapeB::Composition.Object3D)
  n4 = cross(r2.p-r1.p, r3.p-r1.p)
  if norm(n4) <= neps
    r3 = getSupportPoint(shapeA, shapeB, -r3.n) # change search direction
    if abs(dot((r3.p-r1.p),r3.n)) <= neps
      # Shape is purely planar. Computing the shortest distance for a planar shape
      # requires an MPR 2D algorithm (using lines instead of triangles as portals).
      # However, this is not implemented and therefore the shortest distance cannot be computed
      error("MPR: Shapes are planar and MPR2D is not supported. abs(dot((r3.p-r1.p),r3.n)). Look at shapeA = ", shapeA, " shapeB = ", shapeB)
    end
    n4 = cross(r2.p-r1.p, r3.p-r1.p)   # |n4| > 0 guaranteed, due to construction
  end
  if dot(n4,r0.p) >= neps
    n4 = -n4
  end
  r4 = getSupportPoint(shapeA, shapeB, Basics.normalizeVector(n4))

  return (r3, r4, n4)
end


isNextPortal(r0,r1,r2,r4) = dot( cross( (r2.p-r0.p), (r4.p-r0.p) ), r0.p ) <= 0.0 &&
                            dot( cross( (r4.p-r0.p), (r1.p-r0.p) ), r0.p ) <= 0.0


# Construction of three baby tetrahedrons and decide which one will replace
# the current tetrahedron r1r2 - r2r3 - r3r1 - current tetrahedron edges
# leads to 3 new tetrahedrons:
#   r1r2r4
#   r2r3r4
#   r3r1r4
function createBabyTetrahedrons(r0::SupportPoint,r1::SupportPoint,r2::SupportPoint,r3::SupportPoint,r4::SupportPoint,
                                shapeA::Composition.Object3D,shapeB::Composition.Object3D)
  if isNextPortal(r0,r1,r2,r4)
     r3 = r4
  elseif isNextPortal(r0,r2,r3,r4)
     r1 = r4
  elseif isNextPortal(r0,r3,r1,r4)
     r2 = r4
  else
     # Compute the closest distance to the portal and return it:
     (r4.p, distance) = signedDistanceToPortal(r0.p,r1.p,r2.p,r3.p)
     barycentric(r1,r2,r3,r4)
     error("MPR: Numerical issues with distance computation between shapeA = ", shapeA, " and shapeB = ", shapeB,". Used distance = ",distance)
     return (distance,r4.a,r4.b,r4.n,r1.a,r1.b,r2.a,r2.b,r3.a,r3.b)
  end
  return (r1,r2,r3)
end

# MPR - Minkowski Portal Refinement algorithm
# Phase 1
#   Construction of r0 and of initial portal triangle points r1, r2, r3
#   Phase 1.1: construction of r0
#   Phase 1.2: construction of initial r1
#     Termination Condition 1
#   Phase 1.3: construction of initial r2
#   Phase 1.4: construction of initial r3
# Phase 2
#   loop around to "ensure" the tetrahedron r0,r1,r2 and r3 encloses the origin
# Phase 3
#   Phase 3.1: construct r4,
#   Phase 3.2: check if r4 is close to the origin
#     Termination Condition 2
#     Termination Condition 3
#   Phase 3.3: construct baby tetrahedrons with r1,r2,r3,r4 and create a new portal
function mpr(ch::Composition.ContactDetectionMPR_handler, shapeA::Composition.Object3D, shapeB::Modia3D.Composition.Object3D)
  tol_rel = ch.tol_rel
  niter_max = ch.niter_max
  neps = ch.neps

  ###########      Phase 1, Minkowski Portal Refinement      ###################
  # Construction of r0 and initial portal triangle points r1, r2, r3

  ### Phase 1.1: construction of r0 ###
  centroidA = getCentroid(shapeA, Modia3D.centroid(shapeA.data.geo))
  centroidB = getCentroid(shapeB, Modia3D.centroid(shapeB.data.geo))
  r0 = SupportPoint(centroidA-centroidB, SVector{3,Float64}(0.0,0.0,0.0), SVector{3,Float64}(0.0,0.0,0.0), SVector{3,Float64}(0.0,0.0,0.0))
  # check if centers of shapes are overlapping
  checkCentersOfShapesOverlapp(r0, neps, shapeA, shapeB)

  ### Phase 1.2: construction of initial r1 ###
  r1 = getSupportPoint(shapeA, shapeB, Basics.normalizeVector(-r0.p))

  ### Phase 1.3: construction of initial r2 ###
  n2 = cross(r0.p, r1.p)
  n2abs = norm(n2)
  ## TERMINATION CONDITION 1 ##
  if n2abs <= neps
    # centers of shapes (r0.p and r1.p) are on the same line segment
    # e.g. any collision between two spheres
    println("TC 1")
    distance = dot(r1.p,normalize(r0.p))
    return (distance,r1.a,r1.b,r1.n, nothing, nothing, nothing, nothing, nothing, nothing)
  else
    n2 = n2/n2abs
  end
  r2 = getSupportPoint(shapeA, shapeB, n2)

  ### Phase 1.4: construction of initial r3 ###
  (r2, r3, n2, n3) = checkIfShapesArePlanar(r0, r1, r2, n2, neps, shapeA, shapeB)


  ###########      Phase 2, Minkowski Portal Refinement      ###################
  # loop around to "ensure" the tetrahedron r0,r1,r2 and r3 encloses the origin
  (r1,r2,r3) = tetrahedronEncloseOrigin(r0,r1,r2,r3,neps,niter_max,shapeA,shapeB)
  # doesRayIntersectPortal(r1.p,r2.p,r3.p, r0.p,neps) # Portal.A, Portal.B, Portal.C, point, neps


  ###########      Phase 3, Minkowski Portal Refinement      ###################
  for i in 1:niter_max
    ### Phase 3.1: construct r4 ###
    # Find support point using the tetrahedron face
    (r3,r4,n4) = constructR4(r0,r1,r2,r3,neps,shapeA,shapeB)


    ### Phase 3.2: check if r4 is close to the origin ###
    # check if the new point r4 is already on the plane of the triangle r1,r2,r3,
    # we're already as close as we can get to the origin

    ## TERMINATION CONDITION 2 ##
    if norm(cross(r4.p,r4.n)) < tol_rel
      println("TC 2")
      #if !analyzeFinalPortal(r1.p, r2.p, r3.p, r4.p, neps)
        # error("shapeA = ", shapeA, " shapeB = ", shapeB)
      #end
      distance = -dot(r4.n, r4.p)
      barycentric(r1,r2,r3,r4)
      # println("r4.a = ", r4.a, ", r4.b = ", r4.b, " ,r1.a = ", r1.a, " ,r1.b = ", r1.b," ,r2.a = ", r2.a, " ,r2.b = ",r2.b," ,r3.a = ",r3.a," ,r3.b = ",r3.b)
      return (distance,r4.a,r4.b,r4.n,r1.a,r1.b,r2.a,r2.b,r3.a,r3.b)

    ## TERMINATION CONDITION 3 ##
    elseif abs(dot(r4.p-r1.p, r4.n)) < tol_rel
      println("TC 3")
      #doesRayIntersectPortal(r1.p,r2.p,r3.p, r4.p,neps)
      #println("r1.p = ", r1.p , " r2.p = ", r2.p ," r3.p = ", r3.p)
      #println("r4.p = ", r4.p)
      #println(" ")
      #if !analyzeFinalPortal(r1.p, r2.p, r3.p, r4.p, neps)
        # error("shapeA = ", shapeA, " shapeB = ", shapeB)
      #end
      (r4.p, distance) = signedDistanceToPortal(r0.p,r1.p,r2.p,r3.p)
      barycentric(r1,r2,r3,r4)
      #println("r4.n = ", r4.n)
      # println("r4.a = ", r4.a, ", r4.b = ", r4.b, " ,r1.a = ", r1.a, " ,r1.b = ", r1.b," ,r2.a = ", r2.a, " ,r2.b = ",r2.b," ,r3.a = ",r3.a," ,r3.b = ",r3.b)
      return (distance,r4.a,r4.b,r4.n,r1.a,r1.b,r2.a,r2.b,r3.a,r3.b)
    end

    #### Phase 3.3: construct baby tetrahedrons with r1,r2,r3,r4 and create a new portal ###
    # Construction of three baby tetrahedrons
    (r1,r2,r3) = createBabyTetrahedrons(r0,r1,r2,r3,r4,shapeA,shapeB)


#=
    r0RayOnPortal = isr0RayOnPortal(r0.p,r1.p,r2.p,r3.p)
    if r0RayOnPortal[1] != true
       println("r0RayOnPortal = ", r0RayOnPortal, ": Wrong baby-tetrahedron selected. " , i)
    end
    =#

  end
  error("MPR: Should never get here! Computation failed. Look at shapeA = ", ModiaMath.instanceName(shapeA),
       " shapeB = ",  ModiaMath.instanceName(shapeB))
end
