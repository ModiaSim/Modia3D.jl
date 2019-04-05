module test_solidProperties

using Modia3D
using Modia3D.ModiaMath
using Modia3D.Unitful
using Modia3D.Test
using Modia3D.LinearAlgebra



filename = joinpath(Modia3D.path, "objects", "primitiveFiles", "cube.obj")

# Properties
vmat1 = Modia3D.Material(color="LightBlue", transparency=0.5, wireframe=false)
vmat2 = deepcopy(vmat1)
vmat2.transparency = 0.7
smat1 = Modia3D.MassProperties(m=1.0)
font  = Modia3D.Font(fontFamily="TimesNewRoman", color="Black", bold=false, charSize=0.2)
rtext = [0.0, 0.0, 1.0]
ltext = [0.0, 0.0, -0.5]

# Dimensions
LxBox = 0.9; LyBox = 0.5; LzBox = 0.3
DxSphere = 0.7
DxEll = 0.7; DyEll = 0.3; DzEll = 0.4
DxCyl = 0.5; DyCyl = 0.6; LzCyl = 0.8
DxCone = 0.3; DyCone = 0.4; LzCone = 0.7
DxCap = 0.4; DyCap = 0.3; LzCap = 0.45
DxPipe = 0.5; DyPipe = 0.8; LzPipe = 0.4
LxBeam = 0.4; DyBeam = 0.5; LzBeam = 0.3

# Solid objects
@assembly Solids begin
   world     = Modia3D.Object3D(visualizeFrame=false)
   box       = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(LxBox,LyBox,LzBox)      , "Aluminium", vmat1); r=[ 4.5,0.0,0.0])
   sphere    = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(DxSphere)           , "Aluminium", vmat1); r=[ 3.0,0.0,0.0])
   ellipsoid = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidEllipsoid(DxEll,DyEll,DzEll), "Aluminium", vmat1); r=[ 1.5,0.0,0.0])
   cylinder  = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidCylinder(DxCyl,LzCyl; Dy=DyCyl)     , smat1      , vmat1); r=[ 0.0,0.0,0.0])
   cone      = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidCone(DxCone,LzCone; Dy=DyCone)         , smat1      , vmat1); r=[-1.5,0.0,0.0])
   capsule   = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidCapsule(DxCap,LzCap; Dy=DyCap)     , smat1      , vmat1); r=[-3.0,0.0,0.0])
   pipe      = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidPipe(DxPipe,LzPipe; Dy=DyPipe)         , smat1      , vmat1); r=[ 4.5,0.0,-2.5])
   beam      = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBeam(LxBeam,DyBeam,LzBeam)     , smat1      , vmat1); r=[ 3.0,0.0,-2.5])

   fileMesh = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidFileMesh(filename,1.0), smat1      , vmat1);  R=ModiaMath.rot2(180u"°")*ModiaMath.rot1(90u"°"), r=[ 1.5,0.0,-2.5])

   # coord Systems
   coordBox     = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 4.5,0.0,0.0])
   coordSphere  = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 3.0,0.0,0.0])
   coordCyl     = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 1.5,0.0,0.0])
   coordCone    = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 0.0,0.0,0.0])
   coordCap     = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ -1.5,0.0,0.0])
   coord2       = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ -3.0,0.0,0.0])
   coordpipe    = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 4.5,0.0,-2.5])
   coordBeam    = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 3.0,0.0,-2.5])
   coordMesh1   = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 1.5,0.0,-2.5])
   # Place text above the shapes
   boxText       = Modia3D.Object3D(box      , Modia3D.TextShape("SolidBox"     ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   sphereText    = Modia3D.Object3D(sphere   , Modia3D.TextShape("SolidSphere"  ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   ellipsoidText = Modia3D.Object3D(ellipsoid, Modia3D.TextShape("SolidEllipsoid";font=font); r=rtext, visualizeFrame=Modia3D.False)
   cylinderText  = Modia3D.Object3D(cylinder , Modia3D.TextShape("SolidCylinder"; font=font); r=rtext, visualizeFrame=Modia3D.False)
   coneText      = Modia3D.Object3D(cone     , Modia3D.TextShape("SolidCone"    ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   capsuleText   = Modia3D.Object3D(capsule  , Modia3D.TextShape("SolidCapsule" ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   pipeText      = Modia3D.Object3D(pipe     , Modia3D.TextShape("SolidPipe"    ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   beamText      = Modia3D.Object3D(beam     , Modia3D.TextShape("SolidBeam"    ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   meshText1     = Modia3D.Object3D(coordMesh1, Modia3D.TextShape("SolidFileMesh"; font=font); r=rtext, visualizeFrame=Modia3D.False)
   meshText4     = Modia3D.Object3D(coordMesh1, Modia3D.TextShape("(.obj)"        ; font=font); r=ltext, visualizeFrame=Modia3D.False)
end
solid = Solids(sceneOptions=Modia3D.SceneOptions(visualizeFrames=false, defaultFrameLength=0.7) )
Modia3D.visualizeAssembly!( solid )

#=
println("Modia3D.volume(box) = ",       Modia3D.volume(solid.box.data.geo))
println("Modia3D.volume(sphere) = ",    Modia3D.volume(solid.sphere.data.geo))
println("Modia3D.volume(ellipsoid) = ", Modia3D.volume(solid.ellipsoid.data.geo))
println("Modia3D.volume(cylinder) = ",  Modia3D.volume(solid.cylinder.data.geo))
println("Modia3D.volume(cone) = ",      Modia3D.volume(solid.cone.data.geo))
println("Modia3D.volume(capsule) = ",   Modia3D.volume(solid.capsule.data.geo))
println("Modia3D.volume(pipe) = ",      Modia3D.volume(solid.pipe.data.geo))
println("Modia3D.volume(beam) = ",      Modia3D.volume(solid.beam.data.geo))
println("Modia3D.volume(fileMesh) = ", Modia3D.volume(solid.fileMesh.data.geo))
=#

#=
# println("Modia3D.bottomArea(sphere) = ",    Modia3D.bottomArea(solid.sphere.data.geo))
# println("Modia3D.bottomArea(ellipsoid) = ", Modia3D.bottomArea(solid.ellipsoid.data.geo))
# println("Modia3D.bottomArea(capsule) = ",   Modia3D.bottomArea(solid.capsule.data.geo))
# println("Modia3D.bottomArea(fileMesh) = ", Modia3D.bottomArea(solid.fileMesh.data.geo))
println("A sphere, an ellipsoid, a capsule, and a fileMesh do not have a bottom area.")
println("Modia3D.bottomArea(box) = ",       Modia3D.bottomArea(solid.box.data.geo))
println("Modia3D.bottomArea(cylinder) = ",  Modia3D.bottomArea(solid.cylinder.data.geo))
println("Modia3D.bottomArea(cone) = ",      Modia3D.bottomArea(solid.cone.data.geo))
println("Modia3D.bottomArea(pipe) = ",      Modia3D.bottomArea(solid.pipe.data.geo))
println("Modia3D.bottomArea(beam) = ",      Modia3D.bottomArea(solid.beam.data.geo))
=#

#=
#println("Modia3D.topArea(sphere) = ",    Modia3D.topArea(solid.sphere.data.geo))
#println("Modia3D.topArea(ellipsoid) = ", Modia3D.topArea(solid.ellipsoid.data.geo))
#println("Modia3D.topArea(capsule) = ",   Modia3D.topArea(solid.capsule.data.geo))
println("A sphere, an ellipsoid, and a capsule do not have a top area.")
println("Modia3D.topArea(box) = ",       Modia3D.topArea(solid.box.data.geo))
println("Modia3D.topArea(cylinder) = ",  Modia3D.topArea(solid.cylinder.data.geo))
println("Modia3D.topArea(cone) = ",      Modia3D.topArea(solid.cone.data.geo))
println("Modia3D.topArea(pipe) = ",      Modia3D.topArea(solid.pipe.data.geo))
println("Modia3D.topArea(beam) = ",      Modia3D.topArea(solid.beam.data.geo))

println("Modia3D.longestEdge(box) = ",       Modia3D.longestEdge(solid.box.data.geo))
println("Modia3D.longestEdge(sphere) = ",    Modia3D.longestEdge(solid.sphere.data.geo))
println("Modia3D.longestEdge(ellipsoid) = ", Modia3D.longestEdge(solid.ellipsoid.data.geo))
println("Modia3D.longestEdge(cylinder) = ",  Modia3D.longestEdge(solid.cylinder.data.geo))
println("Modia3D.longestEdge(cone) = ",      Modia3D.longestEdge(solid.cone.data.geo))
println("Modia3D.longestEdge(capsule) = ",   Modia3D.longestEdge(solid.capsule.data.geo))
println("Modia3D.longestEdge(pipe) = ",      Modia3D.longestEdge(solid.pipe.data.geo))
println("Modia3D.longestEdge(beam) = ",      Modia3D.longestEdge(solid.beam.data.geo))
println("Modia3D.longestEdge(fileMesh) = ", Modia3D.longestEdge(solid.fileMesh.data.geo))

println("Modia3D.lengthGeo(box) = ",       Modia3D.lengthGeo(solid.box.data.geo))
println("Modia3D.lengthGeo(sphere) = ",    Modia3D.lengthGeo(solid.sphere.data.geo))
println("Modia3D.lengthGeo(ellipsoid) = ", Modia3D.lengthGeo(solid.ellipsoid.data.geo))
println("Modia3D.lengthGeo(cylinder) = ",  Modia3D.lengthGeo(solid.cylinder.data.geo))
println("Modia3D.lengthGeo(cone) = ",      Modia3D.lengthGeo(solid.cone.data.geo))
println("Modia3D.lengthGeo(capsule) = ",   Modia3D.lengthGeo(solid.capsule.data.geo))
println("Modia3D.lengthGeo(pipe) = ",      Modia3D.lengthGeo(solid.pipe.data.geo))
println("Modia3D.lengthGeo(beam) = ",      Modia3D.lengthGeo(solid.beam.data.geo))
#println("Modia3D.lengthGeo(fileMesh) = ", Modia3D.lengthGeo(solid.fileMesh.data.geo))


println("Modia3D.centroid(box) = ",       Modia3D.centroid(solid.box.data.geo))
println("Modia3D.centroid(sphere) = ",    Modia3D.centroid(solid.sphere.data.geo))
println("Modia3D.centroid(ellipsoid) = ", Modia3D.centroid(solid.ellipsoid.data.geo))
println("Modia3D.centroid(cylinder) = ",  Modia3D.centroid(solid.cylinder.data.geo))
println("Modia3D.centroid(cone) = ",      Modia3D.centroid(solid.cone.data.geo))
println("Modia3D.centroid(capsule) = ",   Modia3D.centroid(solid.capsule.data.geo))
println("Modia3D.centroid(pipe) = ",      Modia3D.centroid(solid.pipe.data.geo))
println("Modia3D.centroid(beam) = ",      Modia3D.centroid(solid.beam.data.geo))
println("Modia3D.centroid(fileMesh) = ", Modia3D.centroid(solid.fileMesh.data.geo))
=#
#=
println("Modia3D.inertiaMatrix(box) = ",       Modia3D.inertiaMatrix(solid.box.data.geo,1))
println("Modia3D.inertiaMatrix(sphere) = ",    Modia3D.inertiaMatrix(solid.sphere.data.geo,1))
println("Modia3D.inertiaMatrix(ellipsoid) = ", Modia3D.inertiaMatrix(solid.ellipsoid.data.geo,1))
println("Modia3D.inertiaMatrix(cylinder) = ",  Modia3D.inertiaMatrix(solid.cylinder.data.geo,1))
#println("Modia3D.inertiaMatrix(cone) = ",      Modia3D.inertiaMatrix(solid.cone.data.geo,1))
#println("Modia3D.inertiaMatrix(capsule) = ",   Modia3D.inertiaMatrix(solid.capsule.data.geo,1))
println("Modia3D.inertiaMatrix(pipe) = ",      Modia3D.inertiaMatrix(solid.pipe.data.geo,1))
#println("Modia3D.inertiaMatrix(beam) = ",      Modia3D.inertiaMatrix(solid.beam.data.geo,1))
println("Modia3D.inertiaMatrix(fileMesh) = ", Modia3D.inertiaMatrix(solid.fileMesh.data.geo,1))
=#

#=
box
sphere
ellipsoid
cylinder
cone
capsule
pipe
beam
=#


function volumeCone(geo)
    h = LzCone
     # bottom area
    A = DxCone/2
    B = DyCone/2
    # top area
    a = A * geo.extras[1]
    b = B * geo.extras[1]
    # for cylinder (also top area)
    a_inner = A * geo.extras[1] * geo.extras[2]
    b_inner = B * geo.extras[1] * geo.extras[2]
    if geo.extras[1] == 0.0   # total cone
        return (h*pi/3)*A*B
    else    # frustum of a cone (with elliptic ground area): (h*pi/3) * (A*B^2 - a*b^2)/(B - b)
        # solid frustum of a cone - cylinder
        return (h*pi/3) * (A*B^2 - a*b^2)/(B - b) - pi*h*a_inner*b_inner
    end
end

@testset "Solids: volume test" begin
   @test isapprox(Modia3D.volume(solid.box.data.geo),       LxBox*LyBox*LzBox)
   @test isapprox(Modia3D.volume(solid.sphere.data.geo),    4/3*pi*(DxSphere/2)^3)
   @test isapprox(Modia3D.volume(solid.ellipsoid.data.geo), 4/3*pi*DxEll/2 * DyEll/2 * DzEll/2)
   @test isapprox(Modia3D.volume(solid.cylinder.data.geo),  pi * DxCyl/2 * DyCyl/2 * LzCyl)
   @test isapprox(Modia3D.volume(solid.cone.data.geo),      volumeCone(solid.cone.data.geo))
   @test isapprox(Modia3D.volume(solid.capsule.data.geo),   pi*LzCap*DxCap/2*DyCap/2 + 4/3*pi*(DyCap/2)^2*DxCap/2)
   @test isapprox(Modia3D.volume(solid.pipe.data.geo),      pi*LzPipe*(DxPipe*DyPipe - DxPipe*DyPipe*solid.pipe.data.geo.extras[2]^2)/4)
   @test isapprox(Modia3D.volume(solid.beam.data.geo),      LzBeam*(LxBeam*DyBeam + pi*(DyBeam/2)^2))
   @test isapprox(Modia3D.volume(solid.beam.data.geo),      LzBeam*(LxBeam*DyBeam + pi*(DyBeam/2)^2))
   @test isapprox(Modia3D.volume(solid.fileMesh.data.geo),  1.0)

   @test isapprox(Modia3D.bottomArea(solid.box.data.geo),   solid.box.data.geo.Lx*solid.box.data.geo.Ly)
   @test isapprox(Modia3D.bottomArea(solid.cylinder.data.geo), pi*DxCyl/2*DyCyl/2)
   @test isapprox(Modia3D.bottomArea(solid.cone.data.geo), pi*DxCone/2*DyCone/2)
end

println("... success of test_solidProperties.jl!")
end
