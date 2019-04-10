# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Solids (Modia3D/Solids/_module.jl)
#
#
# Content:
#
# Solid geometries and operations on the geometries:
#    volume(geo)                      - Returns volume of solid
#    centroid(geo)                    - Return position vector from solid reference frame to centroid of solid
#    inertiaMatrix(geo,mass)          - Return inertia matrix of solid with respect to reference frame
#    supportPoint(geo,r_abs,R_abs,e)    - Return support point in direction of unit vector e
#    boundingBox!(geo,r_abs,R_abs,AABB) - Update vertex positions of smallest AABB box that contains geo
#
# Useful web pages:
#    https://en.wikipedia.org/wiki/List_of_moments_of_inertia
#    https://en.wikipedia.org/wiki/List_of_second_moments_of_area

using LinearAlgebra
EYE3() = Matrix(1.0I,3,3)

const InertiaMatrix  = SMatrix{3,3,Float64,9}


#------------------------------------ Sphere ----------------------------------
struct SolidSphere <: Modia3D.AbstractSolidGeometry
  Dx::Float64
  function SolidSphere(Dx::Float64; rsmall=0.001)    # rsmall is ignored
    @assert(Dx >= 0.0)
    new(Dx)
  end
end


#------------------------------------ Ellipsoid -------------------------------
struct SolidEllipsoid <: Modia3D.AbstractSolidGeometry
  Dx::Float64
  Dy::Float64
  Dz::Float64
  function SolidEllipsoid(Dx::Float64,Dy::Float64,Dz::Float64; rsmall=0.001)   # rsmall is ignored
    @assert(Dx >= 0.0)
    @assert(Dy  >= 0.0)
    @assert(Dz >= 0.0)
    new(Dx,Dy,Dz)
  end
end


#------------------------------------ Box ------------------------------------
struct SolidBox <: Modia3D.AbstractSolidGeometry
  Lx::Float64
  Ly::Float64
  Lz::Float64
  rsmall::Float64   # Radius of small sphere to smooth surface for collision handling
  function SolidBox(Lx::Float64,Ly::Float64,Lz::Float64; rsmall=0.001)
    @assert(Lx >= 0.0)
    @assert(Ly >= 0.0)
    @assert(Lz >= 0.0)
    @assert(rsmall >= 0.0)
    rsmall2 = min(rsmall, 0.1*min(Lx, Ly, Lz)) # at most 10% of the smallest edge length
    new(Lx,Ly,Lz,rsmall2)
  end
end


#------------------------------------ Cylinder --------------------------------
struct SolidCylinder <: Modia3D.AbstractSolidGeometry
  Dx::Float64
  Dy::Float64
  Lz::Float64
  rsmall::Float64   # Radius of small sphere to smooth surface for collision handling
  function SolidCylinder(Dx::Float64, Lz::Float64; Dy=Dx, rsmall=0.001)
    @assert(Dx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    @assert(rsmall >= 0.0)
    rsmall2 = min(rsmall, 0.1*min(Dx, Dy, Lz)) # at most 10% of the smallest edge length
    new(Dx,Dy,Lz,rsmall2)
  end
end


#------------------------------------ Capsule --------------------------------
struct SolidCapsule <: Modia3D.AbstractSolidGeometry
  Dx::Float64
  Dy::Float64
  Lz::Float64
  rsmall::Float64   # Radius of small sphere to smooth surface for collision handling
  function SolidCapsule(Dx::Float64, Lz::Float64; Dy=Dx, rsmall=0.001)
    @assert(Dx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    @assert(rsmall >= 0.0)
    rsmall2 = min(rsmall, 0.1*min(Dx, Dy, Lz)) # at most 10% of the smallest edge length
    new(Dx,Dy,Lz,rsmall2)
  end
end


#------------------------------------ Beam -----------------------------------
struct SolidBeam <: Modia3D.AbstractSolidGeometry
  Lx::Float64
  Dy::Float64
  Lz::Float64
  rsmall::Float64   # Radius of small sphere to smooth surface for collision handling
  function SolidBeam(Lx::Float64,Dy::Float64,Lz::Float64; rsmall=0.001)
    #@assert(length >= height)
    @assert(Lx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    rsmall2 = min(rsmall, 0.1*min(Lx, Dy, Lz)) # at most 10% of the smallest edge length
    new(Lx,Dy,Lz,rsmall2)
  end
end


#------------------------------------ Cone -----------------------------------
struct SolidCone <: Modia3D.AbstractSolidGeometry
  Dx::Float64
  Dy::Float64
  Lz::Float64
  extras::MVector{3,Float64}
  rsmall::Float64   # Radius of small sphere to smooth surface for collision handling
  function SolidCone(Dx::Float64, Lz::Float64; Dy=Dx,relativeTipDiameter=0.5,relativeInnerDiameter=0.0, rsmall=0.001)
    # Tip diameter >= 0.0, Tip diameter is relative to base diameter (0 = real cone, 1 = cylinder),
    # Inner diameter any real number, Inner diameter of the cone is proportional to tip size (0..1)
    @assert(Dx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    @assert(relativeTipDiameter>=0.0 && relativeTipDiameter<=1.0)
    @assert(relativeInnerDiameter>=0.0 && relativeInnerDiameter<1.0)
    @assert(rsmall >= 0.0)
    rsmall2 = min(rsmall, 0.1*min(Dx, Dy, Lz)) # at most 10% of the smallest edge length
    extras=[relativeTipDiameter,relativeInnerDiameter,0.0]
    new(Dx,Dy,Lz,extras,rsmall2)
  end
end


#------------------------------------ Pipe -----------------------------------
struct SolidPipe <: Modia3D.AbstractSolidGeometry
  Dx::Float64
  Dy::Float64
  Lz::Float64
  extras::MVector{3,Float64}
  rsmall::Float64   # Radius of small sphere to smooth surface for collision handling
  function SolidPipe(Dx::Float64, Lz::Float64; Dy=Dx, relativeInnerDiameter=0.9, rsmall=0.001)
    #relativeInnerDiameter ... Inner diameter of pipe (0 = rigid, 1 = fully hollow), every real number is allowed
    #relativeTipDiameter ... Tip diameter of pipe relative to base diameter (0 = real cone, 1 = cylinder), every real number >= 0 is possible
    @assert(Dx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    @assert(relativeInnerDiameter >=0.0 && relativeInnerDiameter<1.0)
    @assert(rsmall >= 0.0)
    relativeTipDiameter=1.0
    rsmall2 = min(rsmall, 0.1*min(Dx, Dy, Lz)) # at most 10% of the smallest edge length
    extras=[0.0,relativeInnerDiameter,relativeTipDiameter]
    new(Dx,Dy,Lz,extras,rsmall2)
  end
end


#------------------------------------ SolidFileMesh -----------------------------------
struct SolidFileMesh <: Modia3D.AbstractSolidGeometry
  filename::AbstractString
  scaleFactor::MVector{3,Float64}
  useGraphicsMaterialColor::Bool
  smoothNormals::Bool
  centroid::MVector{3,Float64}
  longestEdge::Float64
  objPoints::Vector{MVector{3,Float64}}
  facesIndizes::Vector{AbstractVector{}}
  volume::Float64
  centroidAlgo::SVector{3,Float64}
  inertia::SMatrix{3,3,Float64,9}
  function SolidFileMesh(filename::AbstractString; scaleFactor=Basics.onesMVector()  , useGraphicsMaterialColor::Bool=false, smoothNormals::Bool=false)
     @assert(isfile(filename))
     @assert(scaleFactor[1] >= 0.0)
     @assert(scaleFactor[2] >= 0.0)
     @assert(scaleFactor[3] >= 0.0)
     (centroid, longestEdge, objPoints, facesIndizes) = getObjInfos(filename, scaleFactor)
     (volume, centroidAlgo, inertia) = computeMassProperties(objPoints, facesIndizes; bodyCoords=false)
     #println("volume = ", volume)
     fileMesh = new(filename, scaleFactor, useGraphicsMaterialColor, smoothNormals, centroid, longestEdge, objPoints, facesIndizes, volume, centroidAlgo, inertia)
     return fileMesh
  end
end
SolidFileMesh(filename::AbstractString, scale::Number; useGraphicsMaterialColor::Bool=false, smoothNormals::Bool=false) =
        SolidFileMesh(filename; scaleFactor=[scale,scale,scale], useGraphicsMaterialColor=useGraphicsMaterialColor, smoothNormals=smoothNormals)


#------------ bottom area ----------------------------------
bottomArea(geo::SolidBox)       = geo.Lx*geo.Ly
bottomArea(geo::SolidCylinder)  = pi*geo.Dx/2*geo.Dy/2
function bottomArea(geo::SolidBeam)
  w = geo.Dy
  h = geo.Lx
  # Area = rectangle + circle
  return h*w + pi*(w/2)^2
end
function bottomArea(geo::SolidCone)
  A = geo.Dx/2
  B = geo.Dy/2
  # maybe for inner hole (circle)
  a_inner = geo.Dx/2 * geo.extras[1] * geo.extras[2]
  b_inner = geo.Dy/2 * geo.extras[1] * geo.extras[2]
  if geo.extras[1] == 0.0   # total cone
    return pi*A*B
  else  # frustum of a cone, maybe with a hole (inner circle)
    return pi*A*B - pi*a_inner*b_inner
  end
end
function bottomArea(geo::SolidPipe)
  A = geo.Dx/2
  B = geo.Dy/2
  a = geo.Dx/2*geo.extras[2]
  b = geo.Dy/2*geo.extras[2]
  return pi*(A*B - a*b)
end
bottomArea(geo::Modia3D.AbstractSolidGeometry)  = error(typeof(geo), ": has no bottom or top area!")


#------------top area ----------------------------------
function topArea(geo::SolidCone)
  # top area
  a = geo.Dx/2 * geo.extras[1]
  b = geo.Dy/2 * geo.extras[1]
  # for cylinder (also top area)
  a_inner = geo.Dx/2 * geo.extras[1] * geo.extras[2]
  b_inner = geo.Dy/2 * geo.extras[1] * geo.extras[2]
  if geo.extras[1] > 0.0   # total cone
    return pi*a*b - pi*a_inner*b_inner
  end
end
topArea(geo::Modia3D.AbstractSolidGeometry) = bottomArea(geo)


#------------------volume-----------------------------------------------
volume(geo::Modia3D.AbstractSolidGeometry) = bottomArea(geo)*lengthGeo(geo)
volume(geo::SolidSphere)                   = 4/3*pi*(geo.Dx/2)^3
function volume(geo::SolidEllipsoid)
  a = geo.Dx/2
  b = geo.Dy/2
  c = geo.Dz/2
  return 4/3*pi*a*b*c
end
function volume(geo::SolidCapsule)
  a = geo.Dx/2
  b = geo.Dy/2
  h = geo.Lz
  # volume cylinder: pi*h*a*b
  # volume ellipsoid: only changing in x and y direction: 4/3*pi*b^2*a
  return pi*h*a*b + 4/3*pi*b^2*a
end
function volume(geo::SolidCone)
  h = geo.Lz
  # bottom area
  A = geo.Dx/2
  B = geo.Dy/2
  # top area
  a = geo.Dx/2 * geo.extras[1]
  b = geo.Dy/2 * geo.extras[1]
  # for cylinder (also top area)
  a_inner = geo.Dx/2 * geo.extras[1] * geo.extras[2]
  b_inner = geo.Dy/2 * geo.extras[1] * geo.extras[2]
  if geo.extras[1] == 0.0   # total cone
    return (h*pi/3)*A*B
  else    # frustum of a cone (with elliptic ground area): (h*pi/3) * (A*B^2 - a*b^2)/(B - b)
    # solid frustum of a cone - cylinder
    return (h*pi/3) * (A*B^2 - a*b^2)/(B - b) - pi*h*a_inner*b_inner
  end
end
function volume(geo::SolidFileMesh)
    if !isempty(geo.facesIndizes)
    #  println("geo.volume = ", geo.volume)
      return geo.volume
    else
      println("SolidFileMesh: ", geo.filename, ". The surface areas must be triangular, and each triangle should be specified in right-handed/counter-clockwise order. Otherwise it is not possible to compute a volume.")
      return nothing
    end
end


#--------------------longest edge (maximum of all directions)---------------------------------
longestEdge(geo::SolidSphere)    = geo.Dx
longestEdge(geo::SolidEllipsoid) = max(geo.Dx,geo.Dy,geo.Dz)
longestEdge(geo::SolidBox)       = max(geo.Lx,geo.Ly,geo.Lz)
longestEdge(geo::SolidBeam)      = max(geo.Lx,geo.Dy,geo.Lz)
longestEdge(geo::SolidFileMesh)  = geo.longestEdge
longestEdge(geo::Modia3D.AbstractSolidGeometry) = max(geo.Dx,geo.Dy,geo.Lz)


#---------------------length of geometry (length in z direction)----------------------------------------------------------
# length of geometries
lengthGeo(geo::Modia3D.AbstractSolidGeometry) = geo.Lz
lengthGeo(geo::SolidSphere)                   = geo.Dx
lengthGeo(geo::SolidEllipsoid)                = geo.Dz # error("There is no length for an ellipsoid!")
lengthGeo(geo::SolidFileMesh)                 = error("lengthGeo(SolidFileMesh): It is not implemented yet!")


#------------------------centroid of geometries----------------------------------------
centroid(geo::Modia3D.AbstractSolidGeometry) = ModiaMath.ZeroVector3D
centroid(geo::SolidCone)                     = SVector{3,Float64}([0.0,0.0,geo.Lz/4])
function centroid(geo::SolidFileMesh)
    if !isempty(geo.facesIndizes)
        return geo.centroidAlgo
    else
        return geo.centroid
    end
end


#------------------------inertia matrix of geometries-----------------------------------
inertiaMatrix(geo::SolidSphere, mass::Number) = InertiaMatrix(mass/10*geo.Dx^2*EYE3())
inertiaMatrix(geo::SolidEllipsoid, mass::Number) = InertiaMatrix(mass/20*Diagonal([geo.Dy^2 + geo.Dz^2,
                                                                                   geo.Dx^2 + geo.Dz^2,
                                                                                   geo.Dx^2 + geo.Dy^2]))
inertiaMatrix(geo::SolidBox, mass::Number) = InertiaMatrix(1/12*mass*Diagonal([geo.Ly^2 + geo.Lz^2,
                                                                               geo.Lx^2 + geo.Lz^2,
                                                                               geo.Lx^2 + geo.Ly^2]))
function inertiaMatrix(geo::SolidCylinder, mass::Number)
    #println("mass = ", mass)
    return InertiaMatrix(mass*Diagonal([1/4*(geo.Dy/2)^2 + 1/3*geo.Lz^2,
                                        1/4*(geo.Dx/2)^2 + 1/3*geo.Lz^2,
                                        1/4*((geo.Dx/2)^2 + (geo.Dy/2)^2)]))
end
function inertiaMatrix(geo::SolidCapsule, massGeo::Number)
    @warn "from Modia3D.inertiaMatrix(SolidCapsule): inertia matrix is not fully tested yet!"
    # mass = rho * volume
    #=
    volGeo = volume(geo)
    rho = massGeo/vollGeo
    volBox = geo.Lx*geo.Lz*geo.Dy
    massBox = volBox*rho
    =#

    volGeo = volume(geo)
    rho = massGeo/vollGeo

    volCyl = geo.Lz*(geo.Dx/2)*(geo.Dy/2)*pi
    massCyl = volCyl*rho

    volHalfEll = (4/3*pi*(geo.Dy/2)^2*(geo.Dx/2))/2
    massHalfEll = volHalfEll*rho

    IxxCyl = massCyl*(1/4*(geo.Dy/2)^2 + 1/3*(geo.Lz^2))
    IyyCyl = massCyl*(1/4*(geo.Dx/2)^2 + 1/3*(geo.Lz^2))
    IzzCyl = massCyl*(1/4*((geo.Dx/2)^2 + (geo.Dy/2)^2))

    # SolidEllipsoid: with Dx, Dy, Dz = Dy
    IxxEll = massHalfEll*1/20*((geo.Dy/2)^2 + (geo.Dy/2)^2)
    IyyEll = massHalfEll*1/20*((geo.Dx/2)^2 + (geo.Dy/2)^2)
    IzzEll = massHalfEll*1/20*((geo.Dx/2)^2 + (geo.Dy/2)^2)

    IxxSteiner = IxxEll + (geo.Lz/2)^2*massHalfEll
    IyySteiner = IyyEll + (geo.Lz/2)^2*massHalfEll

    Ixx    = IxxCyl + 2*IxxSteiner
    Iyy    = IyyCyl + 2*IyySteiner
    Izz    = IzzCyl + 2*IzzEll
    return InertiaMatrix(Diagonal([Ixx,Iyy,Izz]))
end
function inertiaMatrix(geo::SolidBeam, massGeo::Number)
    # mass = rho * volume
    volGeo = volume(geo)
    rho = massGeo/volGeo
    volBox = geo.Lx*geo.Lz*geo.Dy
    massBox = volBox*rho

    volHalfCyl  = geo.Lz*(geo.Dy/2)^2*pi/2
    massHalfCyl = volHalfCyl*rho

    IxxBox = 1/12*massBox * (geo.Dy^2 + geo.Lz^2)
    IyyBox = 1/12*massBox * (geo.Lx^2 + geo.Lz^2)
    IzzBox = 1/12*massBox * (geo.Lx^2 + geo.Dy^2)

    # http://www.efunda.com/math/solids/solids_display.cfm?SolidName=HalfCircularCylinder
    IxxCylHalf = massHalfCyl*(1/4*(geo.Dy/2)^2 + 1/3*(geo.Lz^2))
    IyyCylHalf = massHalfCyl*(1/4*(geo.Dy/2)^2 + 1/3*(geo.Lz^2))
    IzzCylHalf = massHalfCyl*(1/2*(geo.Dy/2)^2)

    IyyCylHalfSteiner = IyyCylHalf + massHalfCyl*(geo.Lx/2)^2
    IzzCylHalfSteiner = IzzCylHalf + massHalfCyl*(geo.Lx/2)^2

    Ixx = IxxBox + 2*IxxCylHalf
    Iyy = IyyBox + 2*IyyCylHalfSteiner
    Izz = IzzBox + 2*IzzCylHalfSteiner
    return InertiaMatrix(Diagonal([Ixx,Iyy,Izz]))
end
function inertiaMatrix(geo::SolidCone, mass::Number)
    if geo.Dx == geo.Dy && geo.extras[2] == 0.0
        r = geo.Dx/2
        h = geo.Lz
        if geo.extras[1] == 0.0   # total cone
            # https://en.wikipedia.org/wiki/List_of_moments_of_inertia
            return InertiaMatrix(3*mass*Diagonal([1/20*r^2 + 1/5*h^2,
                                                  1/20*r^2 + 1/5*h^2,
                                                  1/10*r^2]))
        else
            # https://de.wikipedia.org/wiki/Tr%C3%A4gheitsmoment 18.7.18
            # https://matheplanet.com/default3.html?call=viewtopic.php?topic=202081&ref=https%3A%2F%2Fwww.bing.com%2F 18.7.18
            h2 = geo.Lz*geo.extras[1]
            r2 = geo.Dx/2
            r1 = geo.Dx/2*geo.extras[1]
            xks = h2/4*((r2^2+2*r1*r2+3*r1^2)/(r2^2+r1*r2+r1^2))
            PHI = 1/(r2^3-r1^3)*( 3/80*(4+h2^2/(r2-r1)^2)*(r2^5-r1^5) + r2^3*(h2/4*r2/(r2-r1)-xks)^2 - r1^3*(h2/4*(4*r2-3*r1)/(r2-r1)-xks)^2 )
            return InertiaMatrix(mass*Diagonal([PHI,
                                                PHI,
                                                3/10*(r2^5-r1^5)/(r2^3-r1^3)]))
        end
    else
        error("SolidCone: at the moment inertiaMatrix is defined for Dx=Dy and without an inner hole.")
    end
end
function inertiaMatrix(geo::SolidPipe, mass::Number)
    if geo.Dx == geo.Dy
        r1 = geo.Dx/2*geo.extras[2]
        r2 = geo.Dx/2
        h  = geo.Lz
        return InertiaMatrix(mass*Diagonal([1/12*(3*(r1^2+r2^2)^2 + h^2),
                                            1/12*(3*(r1^2+r2^2)^2 + h^2),
                                            1/2*(r1^2+r2^2)^2]))
    else
        r1x = geo.Dx/2*geo.extras[2]
        r2x = geo.Dx/2
        r1y = geo.Dy/2*geo.extras[2]
        r2y = geo.Dy/2
        h  = geo.Lz
        return InertiaMatrix(mass*Diagonal([1/12*(3*(r1y^2+r2y^2)^2 + h^2),
                                            1/12*(3*(r1x^2+r2x^2)^2 + h^2),
                                            1/4*(r1x^2+r2x^2)^2 + 1/4*(r1y^2+r2y^2)^2]))
    end
end
function inertiaMatrix(geo::SolidFileMesh, mass::Number)
  #println("mass = ", mass)
    if !isempty(geo.facesIndizes)
        return geo.inertia.*mass
    else
        println("SolidFileMesh: ", geo.filename, ". The surface areas must be triangular, and each triangle should be specified in right-handed/counter-clockwise order. Otherwise it is not possible to compute an inertia tensor.")
        return nothing
    end
end

include("computePropertiesFileMes.jl")

include("boundingBoxes.jl")

include("utilities.jl")
