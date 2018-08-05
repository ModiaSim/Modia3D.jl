module Visualize_Solids

using Modia3D
using ModiaMath
using Unitful

filename = joinpath(Modia3D.path, "objects", "pascal", "pascal.obj")

# Properties
vmat1 = Modia3D.Material(color="LightBlue", transparency=0.5, wireframe=false)
vmat2 = deepcopy(vmat1)
vmat2.transparency = 0.7
smat1 = Modia3D.MassProperties(m=1.0)
font  = Modia3D.Font(fontFamily="TimesNewRoman", color="Black", bold=false, charSize=0.2)
rtext = [0.0, 0.0, 1.0]
ltext = [0.0, 0.0, -0.5]
Rmesh = ModiaMath.rot123(90u"°", 180u"°", 0.0)

# Solid objects
@assembly Solids begin
   world     = Modia3D.Object3D(visualizeFrame=false)
   box       = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(0.9,0.5,0.3)      , "Aluminium", vmat1); r=[ 4.5,0.0,0.0])
   sphere    = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(0.7)           , "Aluminium", vmat1); r=[ 3.0,0.0,0.0])
   cylinder  = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidCylinder(0.5,0.8)     , smat1      , vmat1); r=[ 1.5,0.0,0.0])
   cone      = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidCone(0.3,0.7)         , smat1      , vmat1); r=[ 0.0,0.0,0.0])
   capsule   = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidCapsule(0.4,0.45)     , smat1      , vmat1); r=[-1.5,0.0,0.0])
   pipe      = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidPipe(0.5,0.8)         , smat1      , vmat1); r=[ 4.5,0.0,-2.5])
   beam      = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBeam(0.4,0.5,0.3)     , smat1      , vmat1); r=[ 3.0,0.0,-2.5])
   fileMesh1 = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidFileMesh(filename,0.2), smat1      , vmat1);  R=Rmesh, r=[ 1.5,0.0,-2.5])
   fileMesh2 = Modia3D.Object3D(world, Modia3D.SolidWithConvexDecomposition(Modia3D.SolidFileMesh(filename,0.2), smat1, vmat1, vmat2); r=[-1.5,0.0,-2.5], R=Rmesh)

   # coord Systems
   coordBox     = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 4.5,0.0,0.0])
   coordSphere  = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 3.0,0.0,0.0])
   coordCyl     = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 1.5,0.0,0.0])
   coordCone    = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 0.0,0.0,0.0])
   coordCap     = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ -1.5,0.0,0.0])
   coordpipe    = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 4.5,0.0,-2.5])
   coordBeam    = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 3.0,0.0,-2.5])
   coordMesh1   = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ 1.5,0.0,-2.5])
   coordMesh2   = Modia3D.Object3D(world,  Modia3D.CoordinateSystem(0.7) ; r=[ -1.5,0.0,-2.5])

   # Place text above the shapes
   boxText       = Modia3D.Object3D(box      , Modia3D.TextShape("SolidBox"     ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   sphereText    = Modia3D.Object3D(sphere   , Modia3D.TextShape("SolidSphere"  ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   cylinderText  = Modia3D.Object3D(cylinder , Modia3D.TextShape("SolidCylinder"; font=font); r=rtext, visualizeFrame=Modia3D.False)
   coneText      = Modia3D.Object3D(cone     , Modia3D.TextShape("SolidCone"    ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   capsuleText   = Modia3D.Object3D(capsule  , Modia3D.TextShape("SolidCapsule" ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   pipeText      = Modia3D.Object3D(pipe     , Modia3D.TextShape("SolidPipe"    ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   beamText      = Modia3D.Object3D(beam     , Modia3D.TextShape("SolidBeam"    ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   meshText1     = Modia3D.Object3D(coordMesh1, Modia3D.TextShape("SolidFileMesh"; font=font); r=rtext, visualizeFrame=Modia3D.False)
   meshText2     = Modia3D.Object3D(coordMesh2, Modia3D.TextShape("SolidFileMesh with convex decomposition"; font=font); r=rtext, visualizeFrame=Modia3D.False)
   meshText3     = Modia3D.Object3D(coordMesh2, Modia3D.TextShape("(.obj)"        ; font=font); r=ltext, visualizeFrame=Modia3D.False)
   meshText4     = Modia3D.Object3D(coordMesh1, Modia3D.TextShape("(.obj)"        ; font=font); r=ltext, visualizeFrame=Modia3D.False)
end

Modia3D.visualizeAssembly!( Solids() )


println("... success of Visualize_Solids.jl!")
end
