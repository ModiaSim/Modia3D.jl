module volume_computation3D_obj

using Modia3D
using Modia3D.ModiaMath
using Modia3D.Unitful

# filename = joinpath(Modia3D.path, "objects", "pascal", "pascal.obj")
# filename = joinpath(Modia3D.path, "objects", "pascal", "convexSolids_pascal.obj", "pascal-0.obj")
# filename = joinpath(Modia3D.path, "objects", "test2", "test2.obj")
 filename = joinpath(Modia3D.path, "objects", "primitiveFiles", "cube.obj")
# filename = joinpath(Modia3D.path, "objects", "test1", "test1.obj")

# Properties
vmat1 = Modia3D.Material(color="LightBlue", transparency=0.5, wireframe=false)
vmat2 = deepcopy(vmat1)
vmat2.transparency = 0.7
smat1 = Modia3D.MassProperties(m=1.0)
font  = Modia3D.Font(fontFamily="TimesNewRoman", color="Black", bold=false, charSize=0.2)
rtext = [0.0, 0.0, 1.0]
ltext = [0.0, 0.0, -0.5]
Rmesh = ModiaMath.rot123(90u"°", 180u"°", 0.0)

LxBox = 10.0; LyBox = 0.01; LzBox = 0.01
DxCap = 0.4; DyCap = 0.3; LzCap = 0.45

DxCyl = 2.0; DyCyl = 2.0; LzCyl = 2.0

# Beam: just a box
#LxBeamBox = 10.0; DyBeamBox = 0.01; LzBeamBox = 0.01


LxBeam = 0.001; DyBeam = 2.0; LzBeam = 2.0


# Solid objects
@assembly Solids begin
    world     = Modia3D.Object3D(visualizeFrame=false)
    #box       = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(LxBox,LyBox,LzBox)      , smat1, vmat1); r=[ 4.5,0.0,0.0])
    #capsule   = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidCapsule(DxCap,LzCap; Dy=DyCap)     , smat1      , vmat1); r=[-3.0,0.0,0.0])
    beam      = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBeam(LxBeam,DyBeam,LzBeam)      , smat1, vmat1); r=[ 3.0,0.0,0.0])
    cylinder  = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidCylinder(DxCyl,LzCyl; Dy=DyCyl) , smat1, vmat1); r=[ 0.0,0.0,0.0])

    #fileMesh1 = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidFileMesh(filename,1.0), smat1      , vmat1);  R=Rmesh, r=[ 0.0,0.0,0.0])
    # fileMesh2 = Modia3D.Object3D(world, Modia3D.SolidWithConvexDecomposition(Modia3D.SolidFileMesh(filename,0.2), smat1, vmat1, vmat2); r=[-1.5,0.0,-2.5], R=Rmesh)
end
solid = Solids(sceneOptions=Modia3D.SceneOptions(visualizeFrames=false, defaultFrameLength=0.7) )
Modia3D.visualizeAssembly!( solid )

#=
println("inertia ", Modia3D.inertiaMatrix(solid.fileMesh1.data.geo,1) )
println("volume ", Modia3D.volume(solid.fileMesh1.data.geo) )
println("centroid ", Modia3D.centroid(solid.fileMesh1.data.geo) )

println("solid.fileMesh1.data.geo.centroidAlgo =  ", solid.fileMesh1.data.geo.centroidAlgo)
println("solid.fileMesh1.data.geo.centroid =  ", solid.fileMesh1.data.geo.centroid)
=#

# println("Modia3D.inertiaMatrix(capsule) = ",   Modia3D.inertiaMatrix(solid.capsule.data.geo,1))

#println("Modia3D.inertiaMatrix(box) = ",      Modia3D.inertiaMatrix(solid.box.data.geo,1))
#println("Modia3D.inertiaMatrix(beam) = ",      Modia3D.inertiaMatrix(solid.beam.data.geo,1))
#println("Modia3D.inertiaMatrix(cylinder) = ",      Modia3D.inertiaMatrix(solid.cylinder.data.geo,1))

println("... success of volume_computation3D_obj.jl!")
end
