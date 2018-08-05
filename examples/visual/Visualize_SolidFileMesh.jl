module Visualize_SolidFileMesh

using Modia3D
using StaticArrays

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh
vmat2 = deepcopy(vmat1)                                           # material of convex decomposition of SolidFileMesh
vmat2.transparency = 0.7

font  = Modia3D.Font(fontFamily="Arial", bold=true, charSize=0.2)
rtext = [0.0, 1.0, 0.0]
filenamePascal = joinpath(Modia3D.path, "objects", "pascal", "pascal.obj")


@assembly FileMeshes begin
  world = Modia3D.Object3D(visualizeFrame=false)
  solidFileMesh = Modia3D.SolidFileMesh(filenamePascal,0.2)
  fileMesh1 = Modia3D.Object3D(world, Modia3D.Solid(                       solidFileMesh, nothing, vmat1       ); r=[0.0,0.0,0.0])
  fileMesh2 = Modia3D.Object3D(world, Modia3D.SolidWithConvexDecomposition(solidFileMesh, nothing, vmat1, vmat2); r=[3.0,0.0,0.0])

end

Modia3D.visualizeAssembly!( FileMeshes(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true,defaultFrameLength=0.7,enableContactDetection=false) ) )

println("... success of Visualize_SolidFileMesh.jl!")

end
