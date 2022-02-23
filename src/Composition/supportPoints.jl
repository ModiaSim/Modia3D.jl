function supportPoint(obj::Composition.Object3D{F}, e::SVector{3,T})::SVector{3, T} where {T,F}
    shapeKind = obj.shapeKind
    solid::Modia3D.Shapes.Solid{F} = obj.feature
    collisionSmoothingRadius = T(solid.collisionSmoothingRadius)
    obj_r_abs = SVector{3, T}(obj.r_abs)
    obj_R_abs = SMatrix{3,3,T,9}(obj.R_abs)

    if shapeKind == Modia3D.SphereKind
        sphere::Modia3D.Shapes.Sphere{F} = obj.shape
        return Modia3D.supportPoint_Sphere(sphere, obj_r_abs, obj_R_abs, e)
    elseif shapeKind == Modia3D.EllipsoidKind
        ellipsoid::Modia3D.Shapes.Ellipsoid{F} = obj.shape
        return Modia3D.supportPoint_Ellipsoid(ellipsoid, obj_r_abs, obj_R_abs, e)
    elseif shapeKind == Modia3D.BoxKind
        box::Modia3D.Shapes.Box{F} = obj.shape
        return Modia3D.supportPoint_Box(box, obj_r_abs, obj_R_abs, e, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.CylinderKind
        cylinder::Modia3D.Shapes.Cylinder{F} = obj.shape
        return Modia3D.supportPoint_Cylinder(cylinder, obj_r_abs, obj_R_abs, e, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.ConeKind
        cone::Modia3D.Shapes.Cone{F} = obj.shape
        return Modia3D.supportPoint_Cone(cone, obj_r_abs, obj_R_abs, e, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.CapsuleKind
        capsule::Modia3D.Shapes.Capsule{F} = obj.shape
        return Modia3D.supportPoint_Capsule(capsule, obj_r_abs, obj_R_abs, e)
    elseif shapeKind == Modia3D.BeamKind
        beam::Modia3D.Shapes.Beam{F} = obj.shape
        return Modia3D.supportPoint_Beam(beam, obj_r_abs, obj_R_abs, e, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.FileMeshKind
        fileMesh::Modia3D.Shapes.FileMesh = obj.shape
        return Modia3D.supportPoint_FileMesh(fileMesh, obj_r_abs, obj_R_abs, e)
    else
        error("not supported shape for support points")
    end
end


function boundingBox!(obj::Composition.Object3D{F}, AABB::Basics.BoundingBox{F}; tight::Bool=true, scaleFactor::F=F(0.01) ) where F <: Modia3D.VarFloatType
    shapeKind = obj.shapeKind
    solid::Modia3D.Shapes.Solid{F} = obj.feature
    collisionSmoothingRadius = solid.collisionSmoothingRadius

    if shapeKind == Modia3D.SphereKind
        sphere::Modia3D.Shapes.Sphere{F} = obj.shape
        @inbounds begin
            r = sphere.diameter/2
            if tight    # best fitting AABB which is possible
                AABB.x_min = obj.r_abs[1] - r
                AABB.x_max = obj.r_abs[1] + r
                AABB.y_min = obj.r_abs[2] - r
                AABB.y_max = obj.r_abs[2] + r
                AABB.z_min = obj.r_abs[3] - r
                AABB.z_max = obj.r_abs[3] + r
            else
                # scales each axis with scaleFactor 0.1 means 10 percent bigger than best fitting AABB and so on
                @assert(scaleFactor > 0.0)
                scale = r*scaleFactor/2
                AABB.x_min = obj.r_abs[1] - r - scale
                AABB.x_max = obj.r_abs[1] + r + scale
                AABB.y_min = obj.r_abs[2] - r - scale
                AABB.y_max = obj.r_abs[2] + r + scale
                AABB.z_min = obj.r_abs[3] - r - scale
                AABB.z_max = obj.r_abs[3] + r + scale
            end
            return AABB
        end
    elseif shapeKind == Modia3D.EllipsoidKind
        ellipsoid::Modia3D.Shapes.Ellipsoid{F} = obj.shape
        xmin = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1)
        xmax = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1)
        ymin = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1)
        ymax = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1)
        zmin = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1)
        zmax = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1)
    elseif shapeKind == Modia3D.BoxKind
        box::Modia3D.Shapes.Box{F} = obj.shape
        xmin = Modia3D.supportPoint_i_Box(box, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1, collisionSmoothingRadius)
        xmax = Modia3D.supportPoint_i_Box(box, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1, collisionSmoothingRadius)
        ymin = Modia3D.supportPoint_i_Box(box, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1, collisionSmoothingRadius)
        ymax = Modia3D.supportPoint_i_Box(box, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1, collisionSmoothingRadius)
        zmin = Modia3D.supportPoint_i_Box(box, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1, collisionSmoothingRadius)
        zmax = Modia3D.supportPoint_i_Box(box, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.CylinderKind
        cylinder::Modia3D.Shapes.Cylinder{F} = obj.shape
        xmin = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1, collisionSmoothingRadius)
        xmax = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1, collisionSmoothingRadius)
        ymin = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1, collisionSmoothingRadius)
        ymax = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1, collisionSmoothingRadius)
        zmin = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1, collisionSmoothingRadius)
        zmax = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.ConeKind
        cone::Modia3D.Shapes.Cone{F} = obj.shape
        xmin = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1, collisionSmoothingRadius)
        xmax = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1, collisionSmoothingRadius)
        ymin = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1, collisionSmoothingRadius)
        ymax = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1, collisionSmoothingRadius)
        zmin = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1, collisionSmoothingRadius)
        zmax = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.CapsuleKind
        capsule::Modia3D.Shapes.Capsule{F} = obj.shape
        xmin = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1)
        xmax = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1)
        ymin = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1)
        ymax = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1)
        zmin = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1)
        zmax = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1)
    elseif shapeKind == Modia3D.BeamKind
        beam::Modia3D.Shapes.Beam{F} = obj.shape
        xmin = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1, collisionSmoothingRadius)
        xmax = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1, collisionSmoothingRadius)
        ymin = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1, collisionSmoothingRadius)
        ymax = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1, collisionSmoothingRadius)
        zmin = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1, collisionSmoothingRadius)
        zmax = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.FileMeshKind
        fileMesh::Modia3D.Shapes.FileMesh = obj.shape
        xmin = F(Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1))
        xmax = F(Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1))
        ymin = F(Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1))
        ymax = F(Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1))
        zmin = F(Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1))
        zmax = F(Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1))
    else
        error("boundingBox! not supported for shape")
    end

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


function contactPointIsLocallyBijectiveToNormal(obj::Composition.Object3D)
    isBijective = false
    shapeKind = obj.shapeKind
    if shapeKind == Modia3D.SphereKind || shapeKind == Modia3D.EllipsoidKind
        isBijective = true
    end
    return isBijective
end
