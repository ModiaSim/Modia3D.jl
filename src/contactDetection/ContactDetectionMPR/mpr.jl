# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.ContactDetectionMPR (Modia3D/contactDetection/ContactDetectionMPR/_module.jl)
#


# Collision detection algorithm

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


#=
function printWarning(shapeA, shapeB, text)
  warn(text)
end
=#

#=
function printWarning(text)
  warn(text)
end
=#


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

#=
function isNextTetrahedron!(r0::SupportPoint,r4::SupportPoint,r2::SupportPoint,r1::SupportPoint,r3::SupportPoint)
  if dot(-cross(r2.p-r0.p,r4.p-r0.p),r0.p) > neps && dot(-cross(r4.p-r0.p,r1.p-r0.p),r0.p) > neps
    r3 = r4
    return true
  end
  return false
end
=#

#obj.r_abs .= SVector{3,Float64}(r_solid + T_solid'*obj.r_rel)
getCentroid(obj::Composition.Object3D, centroidObj::SVector{3,Float64}) = (obj.r_abs + obj.R_abs'*centroidObj)

#getCentroid(shape::Shapes.AbstractGeometry) = shape.r_abs


isNextPortal(r0,r1,r2,r4) = dot( cross( (r2.p-r0.p), (r4.p-r0.p) ), r0.p ) <= 0.0 &&
                            dot( cross( (r4.p-r0.p), (r1.p-r0.p) ), r0.p ) <= 0.0


function collision(ch::Composition.ContactDetectionMPR_handler, shapeA::Composition.Object3D, shapeB::Modia3D.Composition.Object3D)
  tol_rel = ch.tol_rel
  niter_max = ch.niter_max
  #println("niter_max = ", niter_max)
  neps = ch.neps

   ### Phase 1, Minkowski Portal Refinement
   centroidA = getCentroid(shapeA, Modia3D.centroid(shapeA.data.geo))
   centroidB = getCentroid(shapeB, Modia3D.centroid(shapeB.data.geo))

   r0 = SupportPoint(centroidA-centroidB, SVector{3,Float64}(0.0,0.0,0.0), SVector{3,Float64}(0.0,0.0,0.0), SVector{3,Float64}(0.0,0.0,0.0))
   #println("1. r0 = ", r0.p)
   if norm(r0.p) <= neps # centers of shapes are overlapping
     error("MPR: Too large penetration (prerequisite of MPR violated). Centers are overlapping. Look at shapeA = ", shapeA, " shapeB = ", shapeB)
   end
   r1   = getSupportPoint(shapeA, shapeB, Basics.normalizeVector(-r0.p))
   # println("2. r1.p = ", r1.p)
   n2 = cross(r0.p, r1.p)
   n2abs = norm(n2)
   #if n2abs < 1e-10
   #   println("... (",shapeA, ",", shapeB, "): n2abs = ", n2abs)
   #   if n2abs > neps
   #      println("      r0.p = ", r0.p, ", r1.p = ", r1.p, ", n2 = ", n2)
   #   end
   #end

   if n2abs <= neps
     # r0.p and r1.p are on the same linesegment
     # any collision between two spheres
     distance = dot(r1.p,normalize(r0.p))
     #println("r1.a, r1.b = ", r1.a, " ", r1.b)
     #println("SVector{3,Float64}(r1.a) = ", SVector{3,Float64}(r1.a))
     #println("... (",shapeA, ",", shapeB, "): termination 1")
     println("TC 1")
     # println("Fall n2abs <= neps ", r1.a, " r1.b " , r1.b, " r1.n ", r1.n)

     return (distance, SVector{3,Float64}(r1.a), SVector{3,Float64}(r1.b), SVector{3,Float64}(r1.n), nothing, nothing, nothing, nothing, nothing, nothing)
   else
     n2 = n2/n2abs
   end
   r2 = getSupportPoint(shapeA, shapeB, n2)
   # println("3. r2.p = ", r2.p)

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
   r3 = getSupportPoint(shapeA, shapeB, Basics.normalizeVector(n3))
   # println("4. r3.p = ", r3.p)
   # check if points r1, r2, r3 are on the same line
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

   # Phase 2
   # r0,r1,r2 and r3 form a tetrahedron - however, we need to do a bit
   # of looping around to "ensure" the tetrahedron encloses the origin
   aux = SVector(0.0, 0.0, 0.0)
   success = false
   for i in 1:niter_max
     #println("phase 2, iteration i = ", i)
     #println("r1.p = ", r1.p , " r2.p = ", r2.p ," r3.p = ", r3.p)
      aux = cross(r1.p-r0.p,r3.p-r0.p)
      if dot(aux,r0.p) < -neps
         r2 = r3
         r3 = getSupportPoint(shapeA,shapeB,Basics.normalizeVector(aux))
         #println("4. r3.p = ", r3.p)
         continue
      end
      aux = cross(r3.p-r0.p,r2.p-r0.p)
      if dot(aux,r0.p) < -neps
         r1 = r3
         r3 = getSupportPoint(shapeA,shapeB,Basics.normalizeVector(aux))
         #println("5. r3.p = ", r3.p)
         continue
      end
      # println("counter von loop = ", i)
      success = true
      break
   end

   if success != true
     error("MPR: Max. number of iterations is reached. Look at shapeA = ", shapeA, " shapeB = ", shapeB)
   end

   ### Phase 3
   na = SVector(0.0, 0.0, 0.0)
   nb = SVector(0.0, 0.0, 0.0)
   for i in 1:niter_max
      # Find support point using the tetrahedron face
      #println("phase 3, iteration i = ", i)
      n4 = cross(r2.p-r1.p, r3.p-r1.p)
      if norm(n4) <= neps
        r3 = getSupportPoint(shapeA, shapeB, -r3.n) # change search direction
        #println("6. r3.p = ", r3.p)
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
      #println("7. r4.p = ", r4.p)
      # Is our new point already on the plane of our triangle,
      # we're already as close as we can get to the origin

#=
    println("... i = $i, |cross(r4.p,r4.n)| = ", norm(cross(r4.p/norm(r4.p),r4.n)),
                      ", |dot(r4.p-r1.p, r4.n)| = ", abs(dot(r4.p-r1.p, r4.n)),
                      ", r4.p/|r4.p| = ", r4.p/norm(r4.p), ", r4.n = ", r4.n)
=#

      if norm(cross(r4.p,r4.n)) < tol_rel
        println("TC 2")
        #println("... cross(r4.p,r4.n) < tol_rel")
        if !analyzeFinalPortal(r1.p, r2.p, r3.p, r4.p, neps)
          error("shapeA = ", shapeA, " shapeB = ", shapeB)
        end
        distance = -dot(r4.n, r4.p)
        barycentric(r1,r2,r3,r4)
        # println("r4.a = ", r4.a, ", r4.b = ", r4.b, " ,r1.a = ", r1.a, " ,r1.b = ", r1.b," ,r2.a = ", r2.a, " ,r2.b = ",r2.b," ,r3.a = ",r3.a," ,r3.b = ",r3.b)
        return (distance,SVector{3,Float64}(r4.a),SVector{3,Float64}(r4.b),SVector{3,Float64}(r4.n),SVector{3,Float64}(r1.a),SVector{3,Float64}(r1.b),SVector{3,Float64}(r2.a),SVector{3,Float64}(r2.b),SVector{3,Float64}(r3.a),SVector{3,Float64}(r3.b))
      elseif abs(dot(r4.p-r1.p, r4.n)) < tol_rel
        println("TC 3")
        #println("r1.p = ", r1.p , " r2.p = ", r2.p ," r3.p = ", r3.p)
        #println("r4.p = ", r4.p)
        #println(" ")
        if !analyzeFinalPortal(r1.p, r2.p, r3.p, r4.p, neps)
          error("shapeA = ", shapeA, " shapeB = ", shapeB)
        end
        # println("... abs(dot(r4.p-r1.p, r4.n)) < tol_rel")
        (r4.p, distance) = signedDistanceToPortal(r0.p,r1.p,r2.p,r3.p)
        barycentric(r1,r2,r3,r4)
        # println("r4.a = ", r4.a, ", r4.b = ", r4.b, " ,r1.a = ", r1.a, " ,r1.b = ", r1.b," ,r2.a = ", r2.a, " ,r2.b = ",r2.b," ,r3.a = ",r3.a," ,r3.b = ",r3.b)
        return (distance,SVector{3,Float64}(r4.a),SVector{3,Float64}(r4.b),SVector{3,Float64}(r4.n),SVector{3,Float64}(r1.a),SVector{3,Float64}(r1.b),SVector{3,Float64}(r2.a),SVector{3,Float64}(r2.b),SVector{3,Float64}(r3.a),SVector{3,Float64}(r3.b))
      end


      # We’ll create three baby tetrahedrons and decide which one will replace
      # the current tetrahedron r1r2 - r2r3 - r3r1 - current tetrahedron edges
      # leads to 3 new tetrahedrons:
      #   r1r2r4
      #   r2r3r4
      #   r3r1r4

#=
      while true
        #if isNextTetrahedron!(r0,r4,r2,r1,r3)
        #  break
        #end
        #if isNextTetrahedron!(r0,r4,r3,r2,r1)
        #  break
        #end
        #if isNextTetrahedron!(r0,r4,r1,r3,r2)
        #  break
        #end
        #  break

         na .= -cross(r2.p-r0.p, r4.p-r0.p)
         nb .= -cross(r4.p-r0.p, r1.p-r0.p)
         if dot(na, r0.p) > neps && dot(nb, r0.p) > neps
            # Inside this tetrahedron
            r3 = r4
            break
         end

         na .= -cross(r3.p-r0.p, r4.p-r0.p)
         nb .= -cross(r4.p-r0.p, r2.p-r0.p)
         if dot(na, r0.p) > neps && dot(nb, r0.p) > neps
            # Inside this tetrahedron
            r1 = r4
            break
         end

         na .= -cross(r1.p-r0.p, r4.p-r0.p)
         nb .= -cross(r4.p-r0.p, r3.p-r0.p)
         if dot(na, r0.p) > neps && dot(nb, r0.p) > neps
            # Inside this tetrahedron
            r2 = r4
            break
         end
         #println("shapeA.r_solid = ", shapeA.r_solid, " shapeB.r_solid = ", shapeB.r_solid)
         error("Distance computation failed. ShapeA = ", shapeA, " shapeB = ", shapeB) # Should never get here - as it must be in one of the children!!!
      end
=#

      if isNextPortal(r0,r1,r2,r4)
         r3 = r4
      elseif isNextPortal(r0,r2,r3,r4)
         r1 = r4
      elseif isNextPortal(r0,r3,r1,r4)
         r2 = r4
      else
         # println("Collision: Distance computation failed.\nShapeA = ", shapeA, " shapeB = ", shapeB)
         # error("Distance computation failed. ShapeA = ", shapeA, " shapeB = ", shapeB)
         # Compute the closest distance to the portal and return it:
         (r4.p, distance) = signedDistanceToPortal(r0.p,r1.p,r2.p,r3.p)
         println("MPR: Numerical issues with distance computation between shapeA = ", shapeA, " and shapeB = ", shapeB,". Used distance = ",distance)
         barycentric(r1,r2,r3,r4)
         return (distance,SVector{3,Float64}(r4.a),SVector{3,Float64}(r4.b),SVector{3,Float64}(r4.n),SVector{3,Float64}(r1.a),SVector{3,Float64}(r1.b),SVector{3,Float64}(r2.a),SVector{3,Float64}(r2.b),SVector{3,Float64}(r3.a),SVector{3,Float64}(r3.b))
      end

      r0RayOnPortal = isr0RayOnPortal(r0.p,r1.p,r2.p,r3.p)
      if r0RayOnPortal[1] != true
         # println("r0RayOnPortal = ", r0RayOnPortal, ": Wrong baby-tetrahedron selected. " , i)
         # println("... (",shapeA, ",", shapeB, "): Wrong baby-tetrahedron selected. " , i)
      end
   end
   error("MPR: Should never get here! Computation failed. Look at shapeA = ", ModiaMath.instanceName(shapeA),
         " shapeB = ",  ModiaMath.instanceName(shapeB))

   # Barycentric coordinates to map from the Minkowski difference
   # onto the original shape
   # contactPoint = mapPointOrigin(r1.p,r2.p,r3.p, r1.a,r2.a,r3.a)
end
