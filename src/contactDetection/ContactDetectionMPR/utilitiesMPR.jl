# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.ContactDetectionMPR (Modia3D/contactDetection/ContactDetectionMPR/_module.jl)
#

# Functions to test properties of portal

"""
    (onPortal, b21, b31) = isPointOnPortal(r4,r1,r2,r3) - Determine whether point r4 is on the portal triangle

- onPortal = true, if r4 is on the portal r1,r2,r3 and false if it is outside of the portal.

- b21, b31 are the barycentric coordinates such that r4 = r1 + b21*(r2-r1) + b31*(r3-r1).
  If r4 is on the portal, b21 >= 0, b31 >= 0 and b21+b31 <= 1
"""
function isPointOnPortal(r4::SVector{3,Float64}, r1::SVector{3,Float64}, r2::SVector{3,Float64}, r3::SVector{3,Float64})
    r21 = r2 - r1
    r31 = r3 - r1
    r41 = r4 - r1

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

    # Determine whether r4 is on the triangle
    onPortal = b21 >= 0.0 && b31 >= 0.0 && b21 + b31 <= 1.0

    return (onPortal, b21, b31)
end


"""
    (r, onPortal, b21, b31) = isNormalRayOnPortal(r1,r2,r3) - Determine whether normal ray through origin is on the portal triangle

- r: Intersection of normal ray through origin with portal plane
- onPortal: r is on the portal triangle
"""
function isNormalRayOnPortal(r1::SVector{3,Float64}, r2::SVector{3,Float64}, r3::SVector{3,Float64})
    # Determine unit normal on portal
    n = cross(r2-r1, r3-r1)
    e = normalize(n)

    # Determine intersection of normal ray through origin with portal plane
    #   Ray:   r = lambda*e
    #   Plane: e*(r-r1) = 0
    #   Intersection point:  e*(lambda*e - r1) = 0 -> lambda = e*r1
    lambda = dot(e,r1)
    r4 = lambda*e

    # Determine whether r is on the portal triangle
    (onPortal, b21, b31) = isPointOnPortal(r4,r1,r2,r3)

    return (onPortal, b21, b31, r4)
end


"""
    (r, onPortal, b21, b31) = isr0RayOnPortal(r0,r1,r2,r3) - Determine whether ray from r0 through origin is on the portal triangle

- r: Intersection of ray through origin with portal plane
- onPortal: r is on the portal triangle
"""
function isr0RayOnPortal(r0::SVector{3,Float64}, r1::SVector{3,Float64}, r2::SVector{3,Float64}, r3::SVector{3,Float64})
    # Determine unit normal on portal
    n = cross(r2-r1, r3-r1)
    e = normalize(n)

    # Determine intersection point of ray with portal plane
    #   Ray:   r = lambda*r0
    #   Plane: e*(r-r1) = 0
    #   Intersection point:  e*(lambda*r0 - r1) = 0 -> lambda = e*r1/(e*r0)
    lambda = dot(e,r1)/dot(e,r0)
    r4 = lambda*r0   # intersection point

    # Determine whether intersection point is on portal
    (onPortal, b21, b31) = isPointOnPortal(r4,r1,r2,r3)

    return (onPortal, b21, b31, r4)
end


"""
    Test functions
"""
function testIntersectionWithPortal()
    # Portal, parallel to x-y plane
    r1 = SVector( 1.0,0.0,2.0)
    r2 = SVector( 0.0,1.0,2.0)
    r3 = SVector(-1.0,-0.1,2.0)

    # Check r0 ray on portal
    r0 = SVector(0.0,0.0,-1.0)
    (onPortal, b21, b31, r4) = isr0RayOnPortal(r0,r1,r2,r3)
    println("onPortal = ", onPortal, ", b21 = ", b21, ", b31 = ", b31, ", r4 = ", r4)

    # Check r0 ray not on portal
    r0 = SVector(2.0,0.0,-1.0)
    (onPortal, b21, b31, r4) = isr0RayOnPortal(r0,r1,r2,r3)
    println("onPortal = ", onPortal, ", b21 = ", b21, ", b31 = ", b31, ", r4 = ", r4)

    # Check normal ray on portal
    (onPortal, b21, b31, r4) = isNormalRayOnPortal(r1,r2,r3)
    println("onPortal = ", onPortal, ", b21 = ", b21, ", b31 = ", b31, ", r4 = ", r4)
end


"""
    (r4,d4) = signedDistanceToLineSegmentWithR4(r1,r2,r21,e,r4_old,d4_old) - Determine closest point on line

- r4: Point on line r1 + lambda*r21 that is closest to the origin with 0<=lambda<=1, or r4_old, if it is closer.
- d4: Signed distance of r4 to the origin or d4_old if abs(d4_old) < norm(r1 + lambda*r21)
"""
function signedDistanceToLineSegmentWithR4(r1::SVector{3,Float64}, r2::SVector{3,Float64},
    r21::SVector{3,Float64}, e::SVector{3,Float64},
    r4_old::SVector{3,Float64}, d4_old::Float64=0.0)
    # r = r1 + lambda*r21  (0 <= lambda <= 1)
    #
    # The closest point to the line is point r4, such that r21*r4 = 0
    # -> r21*(r1+lambda*r12) = 0 -> lambda = -r21*r1/(r21*r21)
    lambda = -dot(r21,r1) / dot(r21,r21)
    r4 = lambda < 0.0 ? r1 : (lambda > 1.0 ? r2 : r1 + lambda*r21)
    d4 = norm(r4)
    if dot(r4,e) > 0.0
       d4 = -d4
    end
    if abs(d4) <= abs(d4_old)
       return (r4,d4)
    else
       return (r4_old, d4_old)
    end
end

"""
    (r4,d4) = signedDistanceToLineSegment(r1,r2,r21,e) - Determine closest point on line

- r4: Point on line r1 + lambda*r21 that is closest to the origin with 0<=lambda<=1.
- d4: Signed distance of r4 to the origin
"""
function signedDistanceToLineSegment(r1::SVector{3,Float64}, r2::SVector{3,Float64},
    r21::SVector{3,Float64}, e::SVector{3,Float64})
    # r = r1 + lambda*r21  (0 <= lambda <= 1)
    #
    # The closest point to the line is point r4, such that r21*r4 = 0
    # -> r21*(r1+lambda*r12) = 0 -> lambda = -r21*r1/(r21*r21)
    lambda = -dot(r21,r1) / dot(r21,r21)
    r4 = lambda < 0.0 ? r1 : (lambda > 1.0 ? r2 : r1 + lambda*r21)
    d4 = norm(r4)
    if dot(r4,e) > 0.0
        d4 = -d4
    end
    return (r4,d4)
end


"""
    (r4,d) = signedDistanceToPortal(r0,r1,r2,r3) - Determine closest point on portal

- r4: Point on triangle portal r1, r2, r3 that is closest to the origin.
- d : Signed distance of r4 to the origin: d > 0: origin is outside of tetrahedron r0,r1,r2,r3, otherwise inside.
"""
function signedDistanceToPortal(r0::SVector{3,Float64}, r1::SVector{3,Float64},
    r2::SVector{3,Float64}, r3::SVector{3,Float64})
    # Determine unit normal on portal
    r21 = r2 - r1
    r31 = r3 - r1
    n   = cross(r21, r31)
    e   = normalize(n)
    if dot(r0,e) > 0.0
        e = -e
    end

    # Determine intersection of normal ray through origin with portal plane
    #   Ray:   r = lambda*e
    #   Plane: e*(r-r1) = 0
    #   Intersection point:  e*(lambda*e - r1) = 0 -> lambda = e*r1
    lambda = dot(e,r1)
    r4 = lambda*e

    # Determine whether r4 is on the portal triangle by computing its barycentric coordinates
    r41 = r4 - r1
    n21 = cross(r21,e)
    n31 = cross(r31,e)

    # Determine barycentric coordinates b21, b31:
    #   n21*(r41 = b21*r21 + b31*r31)    ->  n21*r21 = 0
    #   n31*(r41 = b21*r21 + b31*r31)    ->  n31*r31 = 0
    b21 = dot(n31,r41) / dot(n31,r21)
    b31 = dot(n21,r41) / dot(n21,r31)

    # Determine whether r4 is on the triangle
    onPortal = b21 >= 0.0 && b31 >= 0.0 && b21 + b31 <= 1.0
    if onPortal
        return (r4, -lambda)
    else
        # println("\n... Contact point outside portal (distance might be not precise)")
    end

    # The closest point is on one of the edges.
    (r4,d4) = signedDistanceToLineSegment(r1, r2, r21, e)
    (r4,d4) = signedDistanceToLineSegmentWithR4(r1, r3, r31, e, r4, d4)
    (r4,d4) = signedDistanceToLineSegmentWithR4(r2, r3, r3-r2, e, r4, d4)

    return (r4,d4)
end


"""
    Test functions
"""
function testSignedDistanceToPortal()
    # Portal, parallel to x-y plane
    r0 = SVector( 0.0,0.0,-2.0)
    r1 = SVector( 1.0,0.0,2.0)
    r2 = SVector( 0.0,1.0,2.0)
    r3 = SVector(-1.0,-0.1,2.0)
    (r4,d4) = signedDistanceToPortal(r0,r1,r2,r3)
    println("r4 = ", r4, ", d4 = ", d4)

    r3 = SVector(2.0,1.5,2.0)
    (r4,d4) = signedDistanceToPortal(r0,r1,r2,r3)
    println("r4 = ", r4, ", d4 = ", d4)

    r0 = SVector(0.0,0.0,+4.0)
    r2 = SVector(0.5,1.0,2.0)
    r3 = SVector(0.5,0.0,2.0)
    (r4,d4) = signedDistanceToPortal(r0,r1,r2,r3)
    println("r4 = ", r4, ", d4 = ", d4)
end
