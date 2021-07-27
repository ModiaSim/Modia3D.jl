### -------------------------------------------------------------------------------
# supportPoint(solid, shape,r_abs,R_abs,e) - Return support point of shape shape at position r_abs/R_abs
#                                   in direction of unit vector e
#                                   function is called from mpr algorithm
# boundingBox!(solid, shape,r_abs,R_abs,AABB) - Update vertex positions of smallest AABB box that contains shape

"""
    r = supportPoint(solid, shape, r_abs, R_abs, e)

Return the absolute position vector `r` from world frame to *support point* of
solid `shape`, resolved in world frame in [m]. The support point is the point of solid `shape`
that is the most extreme in direction of unit vector `e`.

# Arguments
- `shape::Modia3D.AbstractGeometry`: Solid shape object.
- `r_abs::AbstractVector`: Absolute position vector of `shape` reference frame.
- `R_abs::AbstractMatrix`: Rotation matrix to rotate world frame in `shape` reference frame.
- `e::AbstractVector`: Unit vector pointing into the desired direction.
"""
@inline supportPoint_Box(shape::Box, r_abs::SVector{3,Float64}, R_abs::SMatrix{3,3,Float64,9}, e::SVector{3,Float64}, collisionSmoothingRadius::Float64) =
            r_abs + R_abs'*supportPoint_abs_Box(shape, R_abs*e, collisionSmoothingRadius) + e*collisionSmoothingRadius
#
@inline supportPoint_Cylinder( shape::Cylinder, r_abs::SVector{3,Float64}, R_abs::SMatrix{3,3,Float64,9}, e::SVector{3,Float64}, collisionSmoothingRadius::Float64) =
            r_abs + R_abs'*supportPoint_abs_Cylinder(shape, R_abs*e) + e*collisionSmoothingRadius

@inline supportPoint_Cone(shape::Cone, r_abs::SVector{3,Float64}, R_abs::SMatrix{3,3,Float64,9}, e::SVector{3,Float64}, collisionSmoothingRadius::Float64) =
            r_abs + R_abs'*supportPoint_abs_Cone(shape, R_abs*e) + e*collisionSmoothingRadius

@inline supportPoint_Capsule(shape::Capsule, r_abs::SVector{3,Float64}, R_abs::SMatrix{3,3,Float64,9}, e::SVector{3,Float64}, collisionSmoothingRadius::Float64) =
            r_abs + R_abs'*supportPoint_abs_Capsule(shape, R_abs*e) + e*collisionSmoothingRadius

@inline supportPoint_Beam(shape::Beam, r_abs::SVector{3,Float64}, R_abs::SMatrix{3,3,Float64,9}, e::SVector{3,Float64}, collisionSmoothingRadius::Float64) =
            r_abs + R_abs'*supportPoint_abs_Beam(shape, R_abs*e) + e*collisionSmoothingRadius

@inline supportPoint_Sphere(shape::Sphere, r_abs::SVector{3,Float64}, R_abs::SMatrix{3,3,Float64,9}, e::SVector{3,Float64}) =
            r_abs + (shape.diameter/2)*e

@inline supportPoint_Ellipsoid(shape::Ellipsoid, r_abs::SVector{3,Float64}, R_abs::SMatrix{3,3,Float64,9}, e::SVector{3,Float64}) =
            r_abs + R_abs'*supportPoint_abs_Ellipsoid(shape, R_abs*e)

@inline supportPoint_FileMesh(shape::FileMesh, r_abs::SVector{3,Float64}, R_abs::SMatrix{3,3,Float64,9}, e::SVector{3,Float64}) =
            r_abs + R_abs'*supportPoint_abs_FileMesh(shape, R_abs*e)


# supportPoint_abs(shape, e_abs) - Return support point of shape shape at the shape reference
#         frame of shape in direction of unit vector e_abs (resolved in shape reference frame = object3d frame)
# see e.g. [Gino v.d. Bergen, Collision Detection in Interactive 3D Environments, 2003]
# or [G. Snethen, Xenocollide: Complex collision made simple. In Game Programming Gems 7, 2008]
# or Andrea Neumayr took ideas for computing based on those books

# [Gino v.d. Bergen, p. 135]
@inline supportPoint_abs_Ellipsoid(shape::Ellipsoid, e_abs::SVector{3,Float64}) =
    @inbounds SVector( (shape.lengthX/2)^2*e_abs[1], (shape.lengthY/2)^2*e_abs[2], (shape.lengthZ/2)^2*e_abs[3] )/norm(SVector((shape.lengthX/2)*e_abs[1], (shape.lengthY/2)*e_abs[2], (shape.lengthZ/2)*e_abs[3]) )

# [Gino v.d. Bergen, p. 135]
@inline supportPoint_abs_Box(shape::Box, e_abs::SVector{3,Float64},  collisionSmoothingRadius::Float64) =
    @inbounds SVector( Basics.sign_eps(e_abs[1])*(shape.lengthX/2-collisionSmoothingRadius), Basics.sign_eps(e_abs[2])*(shape.lengthY/2-collisionSmoothingRadius), Basics.sign_eps(e_abs[3])*(shape.lengthZ/2-collisionSmoothingRadius) )

# [Gino v.d. Bergen, p. 136, XenoCollide, p. 168, 169]
@inline function supportPoint_abs_Cylinder(shape::Cylinder, e_abs::SVector{3,Float64})
    @inbounds begin
        if shape.axis == 1
            enorm = norm(SVector(e_abs[2], e_abs[3]))
            if enorm <= Modia3D.neps
                return Basics.sign_eps(e_abs[1])*SVector(shape.length/2, 0.0, 0.0)
            else
                return Basics.sign_eps(e_abs[1])*SVector(shape.length/2, 0.0, 0.0) + SVector(0.0, (shape.diameter/2)*e_abs[2], (shape.diameter/2)*e_abs[3])/enorm
            end
        elseif shape.axis == 2
            enorm = norm(SVector(e_abs[3], e_abs[1]))
            if enorm <= Modia3D.neps
                return Basics.sign_eps(e_abs[2])*SVector(0.0, shape.length/2, 0.0)
            else
                return Basics.sign_eps(e_abs[2])*SVector(0.0, shape.length/2, 0.0) + SVector((shape.diameter/2)*e_abs[1], 0.0, (shape.diameter/2)*e_abs[3])/enorm
            end
        else
            enorm = norm(SVector(e_abs[1], e_abs[2]))
            if enorm <= Modia3D.neps
                return Basics.sign_eps(e_abs[3])*SVector(0.0, 0.0, shape.length/2)
            else
                return Basics.sign_eps(e_abs[3])*SVector(0.0, 0.0, shape.length/2) + SVector((shape.diameter/2)*e_abs[1], (shape.diameter/2)*e_abs[2], 0.0)/enorm
            end
        end
    end
end

# G. Hippmann: Cylinder + Sphere as bottom and top
@inline function supportPoint_abs_Capsule(shape::Capsule, e_abs::SVector{3,Float64})
    @inbounds begin
        if shape.axis == 1
            return Basics.sign_eps(e_abs[1])*SVector(0.5*shape.length, 0.0, 0.0) + 0.5*shape.diameter*e_abs
        elseif shape.axis == 2
            return Basics.sign_eps(e_abs[2])*SVector(0.0, 0.5*shape.length, 0.0) + 0.5*shape.diameter*e_abs
        else
            return Basics.sign_eps(e_abs[3])*SVector(0.0, 0.0, 0.5*shape.length) + 0.5*shape.diameter*e_abs
        end
    end
end

# for cone: [Gino v.d. Bergen, p. 136]]
# for frustum of a cone: A. Neumayr, G. Hippmann
@inline function supportPoint_abs_Cone(shape::Cone, e_abs::SVector{3,Float64})
    @inbounds begin
        baseRadius = shape.diameter/2
        rightCone = shape.topDiameter == 0.0
        if rightCone
            sin_phi = baseRadius/sqrt(baseRadius^2 + shape.length^2)  # sin of base angle
        else
            topRadius = shape.topDiameter/2
            diffRadius = baseRadius - topRadius
            sin_phi = diffRadius/sqrt(diffRadius^2 + shape.length^2)  # sin of base angle
        end
        if shape.axis == 1
            value = e_abs[1] / norm(SVector(e_abs[1], e_abs[2], e_abs[3]))
            if value >= sin_phi
                if rightCone
                    return SVector(shape.length, 0.0, 0.0)  # apex is support point
                else  # frustum of a cone
                    enorm = norm(SVector(e_abs[2], e_abs[3]))
                    if enorm > Modia3D.neps
                        return SVector(shape.length, 0.0, 0.0) + SVector(0.0, topRadius*e_abs[2], topRadius*e_abs[3]) / enorm  # point on top circle is support point
                    else
                        return SVector(shape.length, 0.0, 0.0)  # top circle center is support point
                    end
                end
            else
                enorm = norm(SVector(e_abs[2], e_abs[3]))
                if enorm > Modia3D.neps
                    return SVector(0.0, baseRadius*e_abs[2], baseRadius*e_abs[3]) / enorm  # point on base circle is support point
                else
                    return SVector(0.0, 0.0, 0.0)  # base circle center is support point
                end
            end
        elseif shape.axis == 2
            value = e_abs[2] / norm(SVector(e_abs[1], e_abs[2], e_abs[3]))
            if value >= sin_phi
                if rightCone
                    return SVector(0.0, shape.length, 0.0)  # apex is support point
                else  # frustum of a cone
                    enorm = norm(SVector(e_abs[3], e_abs[1]))
                    if enorm > Modia3D.neps
                        return SVector(0.0, shape.length, 0.0) + SVector(topRadius*e_abs[1], 0.0, topRadius*e_abs[3]) / enorm  # point on top circle is support point
                    else
                        return SVector(0.0, shape.length, 0.0)  # top circle center is support point
                    end
                end
            else
                enorm = norm(SVector(e_abs[3], e_abs[1]))
                if enorm > Modia3D.neps
                    return SVector(baseRadius*e_abs[1], 0.0, baseRadius*e_abs[3]) / enorm  # point on base circle is support point
                else
                    return SVector(0.0, 0.0, 0.0)  # base circle center is support point
                end
            end
        else
            value = e_abs[3] / norm(SVector(e_abs[1], e_abs[2], e_abs[3]))
            if value >= sin_phi
                if rightCone
                    return SVector(0.0, 0.0, shape.length)  # apex is support point
                else  # frustum of a cone
                    enorm = norm(SVector(e_abs[1], e_abs[2]))
                    if enorm > Modia3D.neps
                        return SVector(0.0, 0.0, shape.length) + SVector(topRadius*e_abs[1], topRadius*e_abs[2], 0.0) / enorm  # point on top circle is support point
                    else
                        return SVector(0.0, 0.0, shape.length)  # top circle center is support point
                    end
                end
            else
                enorm = norm(SVector(e_abs[1], e_abs[2]))
                if enorm > Modia3D.neps
                    return SVector(baseRadius*e_abs[1], baseRadius*e_abs[2], 0.0) / enorm  # point on base circle is support point
                else
                    return SVector(0.0, 0.0, 0.0)  # base circle center is support point
                end
            end
        end
    end
end

# G. Hippmann: Outer half circles of beam
@inline function supportPoint_abs_Beam(shape::Beam, e_abs::SVector{3,Float64})
    @inbounds begin
        if shape.axis == 1
            enorm = norm(SVector(e_abs[1], e_abs[2]))
            if enorm <= Modia3D.neps
                return SVector(Basics.sign_eps(e_abs[1])*shape.length/2, 0.0, Basics.sign_eps(e_abs[3])*shape.thickness/2)
            else
                return SVector(Basics.sign_eps(e_abs[1])*shape.length/2, 0.0, Basics.sign_eps(e_abs[3])*shape.thickness/2) + SVector(shape.width/2*e_abs[1], shape.width/2*e_abs[2], 0.0)/enorm
            end
        elseif shape.axis == 2
            enorm = norm(SVector(e_abs[2], e_abs[3]))
            if enorm <= Modia3D.neps
                return SVector(Basics.sign_eps(e_abs[1])*shape.thickness/2, Basics.sign_eps(e_abs[2])*shape.length/2, 0.0)
            else
                return SVector(Basics.sign_eps(e_abs[1])*shape.thickness/2, Basics.sign_eps(e_abs[2])*shape.length/2, 0.0) + SVector(0.0, shape.width/2*e_abs[2], shape.width/2*e_abs[3])/enorm
            end
        else
            enorm = norm(SVector(e_abs[3], e_abs[1]))
            if enorm <= Modia3D.neps
                return SVector(0.0, Basics.sign_eps(e_abs[2])*shape.thickness/2, Basics.sign_eps(e_abs[3])*shape.length/2)
            else
                return SVector(0.0, Basics.sign_eps(e_abs[2])*shape.thickness/2, Basics.sign_eps(e_abs[3])*shape.length/2) + SVector(shape.width/2*e_abs[1], 0.0, shape.width/2*e_abs[3])/enorm
            end
        end
    end
end

# [Gino v.d. Bergen, p. 131]
@inline function supportPoint_abs_FileMesh(shape::FileMesh, e_abs::SVector{3,Float64})
    @inbounds begin
        max_value = Float64
        position = Float64
        for i in eachindex(shape.objPoints)
            if i == 1
            max_value = dot(shape.objPoints[i],e_abs)
            position = i
            else
            act_value = dot(shape.objPoints[i],e_abs)
            if act_value > max_value
                max_value = act_value
                position = i
        end; end; end
        return shape.objPoints[position]
    end
end


#------------------------------------------------------------------------------------
# boundingBox!(..) calculates the AABB (= Axis Aligned Bounding Box) for the geometries
# it also uses support points for finding the points which are furthest away in each x,y,z - direction
# it calles supportPoint_i
"""
    boundingBox!(shape, AABB, r_abs, R_abs; tight=true, scaleFactor=0.01)

Returns the *Axis Aligned Bounding Box* of solid `shape` in argument `AABB`.

# Arguments
- `shape::Modia3D.AbstractGeometry`: Solid shape object.
- `AABB::Modia3D.BoundingBox`: On return, update AABB with the actual *Axis Aligned Bounding Box* of solid `shape`.
- `r_abs::AbstractVector`: Absolute position vector of `shape` reference frame.
- `R_abs::AbstractMatrix`: Rotation matrix to rotate world frame in `shape` reference frame.
- `tight::Bool`: If true, return the tightest AABB. If false return an AABB that is
                scaleFactor bigger than the best fitting AABB
                (for example, scaleFactor=0.1 means that the returned AABB is 10 percent bigger
                than the best fitting AABB).
- `scaleFactor::Float64`: If `tight=false`, the returned AABB is `scaleFactor` bigger than the
                          best fitting AABB.
"""
#=
function boundingBox!(solid, shape::Modia3D.AbstractGeometry, AABB::Basics.BoundingBox, r_abs::SVector{3,Float64}, R_abs::SMatrix{3,3,Float64,9}; tight::Bool=true, scaleFactor::Float64=0.01)
    @inbounds begin
        xmin = supportPoint_i(solid, shape, r_abs[1], SVector(R_abs[:,1]), -1)
        xmax = supportPoint_i(solid, shape, r_abs[1], SVector(R_abs[:,1]), +1)
        ymin = supportPoint_i(solid, shape, r_abs[2], SVector(R_abs[:,2]), -1)
        ymax = supportPoint_i(solid, shape, r_abs[2], SVector(R_abs[:,2]), +1)
        zmin = supportPoint_i(solid, shape, r_abs[3], SVector(R_abs[:,3]), -1)
        zmax = supportPoint_i(solid, shape, r_abs[3], SVector(R_abs[:,3]), +1)

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
    end
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
   longestEdge = max(shape.scaleFactor[1], shape.scaleFactor[2], shape.scaleFactor[3]) * shape.longestEdge * sqrt(2)
   x_min = -shape.longestEdge + r_rel[1]
   x_max =  shape.longestEdge + r_rel[1]
   y_min = -shape.longestEdge + r_rel[2]
   y_max =  shape.longestEdge + r_rel[2]
   z_min = -shape.longestEdge + r_rel[3]
   z_max = shape.longestEdge + r_rel[3]
 =#
end

function boundingBox!(solid, shape::Sphere, AABB::Basics.BoundingBox, r_abs::SVector{3,Float64}, R_abs::SMatrix{3,3,Float64,9}; tight::Bool=true, scaleFactor::Float64=0.01)
    @inbounds begin
        r = shape.diameter/2
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
end
=#
#------------------------------------------------------------------------------------
# supportPoint_i are utility functions for boundingBox!(..)
# it is based on supportPoint_abs (its nearly the same as supportPoint(..))
@inline supportPoint_i_Box(shape::Box, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int, collisionSmoothingRadius) =
               r_absi + R_absi'*supportPoint_abs_Box(shape, isign*R_absi, collisionSmoothingRadius) + isign*collisionSmoothingRadius

@inline supportPoint_i_Cylinder(shape::Cylinder, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int, collisionSmoothingRadius) =
               r_absi + R_absi'*supportPoint_abs_Cylinder(shape, isign*R_absi) + isign*collisionSmoothingRadius

@inline supportPoint_i_Cone(shape::Cone, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int, collisionSmoothingRadius) =
               r_absi + R_absi'*supportPoint_abs_Cone(shape, isign*R_absi) + isign*collisionSmoothingRadius

@inline supportPoint_i_Capsule(shape::Capsule, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int, collisionSmoothingRadius) =
               r_absi + R_absi'*supportPoint_abs_Capsule(shape, isign*R_absi) + isign*collisionSmoothingRadius
#
@inline supportPoint_i_Beam(shape::Beam, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int, collisionSmoothingRadius) =
               r_absi + R_absi'*supportPoint_abs_Beam(shape, isign*R_absi) + isign*collisionSmoothingRadius

@inline supportPoint_i_Ellipsoid(shape::Ellipsoid, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int) =
               r_absi + dot(R_absi, supportPoint_abs_Ellipsoid(shape, isign*R_absi))

@inline supportPoint_i_FileMesh(shape::FileMesh, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int) =
               r_absi + dot(R_absi, supportPoint_abs_FileMesh(shape, isign*R_absi))
