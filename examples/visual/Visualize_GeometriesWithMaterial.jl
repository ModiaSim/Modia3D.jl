module Visualize_GeometriesWithMaterial

using Modia3D


filename = joinpath(Modia3D.path, "objects", "fish", "SiameseTiger0.3ds")
#filename = joinpath(Modia3D.path, "objects","engine", "crank", "crank.obj")
#file = FileShape.convexFile(filename; scaleFactor=MVector{3,Float64}(4.0,4.0,4.0))

# Properties
mat1  = Modia3D.Material(color="LightBlue", transparency=0.5)
mat2  = Modia3D.Material(transparency=0.1)
font  = Modia3D.Font(fontFamily="TimesNewRoman", bold=false, charSize=0.2, color="Black")
rtext = [0.0, 0.0, 1.0]
ltext = [0.0, 0.0, -0.7]


# Objects3D
@assembly GeometriesWithMaterial begin
   world            = Modia3D.Object3D(visualizeFrame=false)
   box              = Modia3D.Object3D(world, Modia3D.Box(0.9,0.5,0.3,       material=mat1); r=[ 4.5,0.0, 0.0])
   sphere           = Modia3D.Object3D(world, Modia3D.Sphere(0.7,            material=mat1); r=[ 3.0,0.0, 0.0])
   cylinder         = Modia3D.Object3D(world, Modia3D.Cylinder(0.5,0.8,      material=mat1); r=[ 1.5,0.0, 0.0])
   cone             = Modia3D.Object3D(world, Modia3D.Cone(0.3,0.7,          material=mat1); r=[ 0.0,0.0, 0.0])
   capsule          = Modia3D.Object3D(world, Modia3D.Capsule(0.4,0.45,      material=mat1); r=[-1.5,0.0, 0.0])
   spring           = Modia3D.Object3D(world, Modia3D.Spring(0.3,0.7,        material=mat1); r=[ 4.5,0.0,-2.5])
   gearWheel        = Modia3D.Object3D(world, Modia3D.GearWheel(0.5,0.8,     material=mat1); r=[ 3.0,0.0,-2.5])
   pipe             = Modia3D.Object3D(world, Modia3D.Pipe(0.5,0.8,          material=mat1); r=[ 1.5,0.0,-2.5])
   beam             = Modia3D.Object3D(world, Modia3D.Beam(0.4,0.5,0.3,      material=mat1); r=[ 0.0,0.0,-2.5])
   fileMesh         = Modia3D.Object3D(world, Modia3D.FileMesh(filename,4.0, material=mat2); r=[-1.5,0.0,-2.5])

   # Place text above the shapes
   boxText              = Modia3D.Object3D(box             , Modia3D.TextShape("Box"             ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   sphereText           = Modia3D.Object3D(sphere          , Modia3D.TextShape("Sphere"          ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   cylinderText         = Modia3D.Object3D(cylinder        , Modia3D.TextShape("Cylinder"        ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   coneText             = Modia3D.Object3D(cone            , Modia3D.TextShape("Cone"            ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   capsuleText          = Modia3D.Object3D(capsule         , Modia3D.TextShape("Capsule"         ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   springText           = Modia3D.Object3D(spring          , Modia3D.TextShape("Spring"          ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   gearWheelText        = Modia3D.Object3D(gearWheel       , Modia3D.TextShape("GearWheel"       ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   pipeText             = Modia3D.Object3D(pipe            , Modia3D.TextShape("Pipe"            ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   beamText             = Modia3D.Object3D(beam            , Modia3D.TextShape("Beam"            ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   fileMeshText         = Modia3D.Object3D(fileMesh        , Modia3D.TextShape("FileMesh"        ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   fileMeshTextSb       = Modia3D.Object3D(fileMesh        , Modia3D.TextShape("(.obj, .3ds, .dxf, .stl)"        ; font=font); r=ltext, visualizeFrame=Modia3D.False)
end

Modia3D.visualizeAssembly!( GeometriesWithMaterial(sceneOptions = Modia3D.SceneOptions(visualizeFrames=true,
                                                                                       defaultFrameLength=0.7,
                                                                                       enableContactDetection=false)) )

println("... success of Visualize_GeometriesWithMaterial.jl!")

end
