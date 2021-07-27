
mutable struct Segment
   P0::SVector{3,Float64}    # Point 0 (support vector of line segment)
   P1::SVector{3,Float64}    # Point 1
   function Segment(P0::SVector{3,Float64},P1::SVector{3,Float64})
     new(P0,P1)
   end
end


mutable struct Plane
   V0::SVector{3,Float64}    # Point 0 of plane (support vector)
   V1::SVector{3,Float64}    # Point 1
   V2::SVector{3,Float64}    # Point 1
   n::SVector{3,Float64}     # normal of plane
   function Plane(V0::SVector{3,Float64}, V1::SVector{3,Float64}, V2::SVector{3,Float64})
      n = cross(V1-V0, V2-V0)
      new(V0,V1,V2,n)
   end
end


function intersect3DSegmentPlane(seg::Segment, plane::Plane, neps)
   u = seg.P1 - seg.P0
   w = seg.P0 - plane.V0

   dividend = -dot(plane.n, w)
   divisor = dot(plane.n, u)

   # checks if segment and plane are parallel
   if abs(divisor) < neps  # segment is parallel to plane
      if (abs(dividend) < neps)  # segemnt is parallel to plane AND lies in plane
         error("Segment is parallel to plane and lies in plane")
         return (2, nothing)
      else
         error("Segment is parallel to plane")
         return (0, nothing)
      end
   end

   # computes intersection point of plane and segment
   SI = dividend/divisor
   I = seg.P0 + SI*u
   return (1, I)
end


function sameSideTriangle(p1, p2, a, b)::Bool
   cp1 = cross(b-a, p1-a)
   cp2 = cross(b-a, p2-a)
   return (dot(cp1, cp2) >= 0.0)
end
pointInTriangle(p,a,b,c) = (sameSideTriangle(p,a,b,c) && sameSideTriangle(p,b,a,c) && sameSideTriangle(p,c,a,b))


function doesRayIntersectPortal(r1,r2,r3,point,neps)
   plane = Plane(r1,r2,r3)
   segment = Segment(point, SVector(0.0, 0.0, 0.0))

   (value, intersectionPoint) = intersect3DSegmentPlane(segment, plane, neps)
   if value == 1
      if !pointInTriangle(intersectionPoint, r1, r2, r3)
         error("Ray = ", point ," does not intersect portal (r1,r2,r3) = ", r1, r2, r3)
         return false
      end
   end

   return true
end

function analyzeFinalPortal(r1, r2, r3, r4, neps)
   plane = Plane(r1,r2,r3)
   segment = Segment(r4, SVector(0.0, 0.0, 0.0))
   (value, intersectionPoint) = intersect3DSegmentPlane(segment, plane, neps)
   if value == 1
      if !pointInTriangle(intersectionPoint, r1, r2, r3)
         println("Ray r4 = ", r4 ," is not intersecting last portal (r1,r2,r3) = ", r1, r2, r3)
         return false
      end
   end

   areaPlane = norm(plane.n)*1/2
   println("areaPlane = ", areaPlane)
    # det([r2-r1, r3-r1, r4-r1])*1/6
   volumeTetraeder1 = abs(dot(r2-r1,cross(r3-r1,r4-r1)))*1/6
   #volumeTetraeder2 = abs(det(hcat(r2-r1,r3-r1,r4-r1))) *1/6
   println("volumeTetraeder = ", volumeTetraeder1)
   #println("volumeTetraeder2 = ", volumeTetraeder2)
   return true
end



function sameSideTetrahedron(v1,v2,v3,v4,p)
   normal = cross(v2-v1,v3-v1)
   dotV4  = dot(normal, v4-v1)
   dotP   = dot(normal, p-v1)
   return ( sign(dotV4) == sign(dotP) )
end
pointInTetrahedron(v1,v2,v3,v4,p) = (sameSideTetrahedron(v1,v2,v3,v4,p) && sameSideTetrahedron(v2,v3,v4,v1,p) && sameSideTetrahedron(v3,v4,v1,v2,p) && sameSideTetrahedron(v4,v1,v2,v3,p) )
