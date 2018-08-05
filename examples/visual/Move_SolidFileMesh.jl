module Move_SolidFileMesh

using Modia3D
using StaticArrays

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh
vmat2 = deepcopy(vmat1)                                           # material of convex decomposition of SolidFileMesh
vmat2.transparency = 0.7

font  = Modia3D.Font(fontFamily="Arial", bold=true, charSize=0.2)
rtext = [0.0, 1.0, 0.0]
filenamePascal = joinpath(Modia3D.path, "objects", "pascal", "pascal.obj")

r1 = [0.0, 0.0, 0.0]
r2 = [3.0, 0.0, 0.0]

@assembly FileMeshes begin
  world = Modia3D.Object3D(visualizeFrame=false)

  solidFileMesh = Modia3D.SolidFileMesh(filenamePascal,0.2)
  fileMesh1 = Modia3D.Object3D(world, Modia3D.Solid(                       solidFileMesh, nothing, vmat1       ); r=r1, fixed=false)
  fileMesh2 = Modia3D.Object3D(world, Modia3D.SolidWithConvexDecomposition(solidFileMesh, nothing, vmat1, vmat2); r=r2, fixed=false)
end

fileMeshes = FileMeshes(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true,defaultFrameLength=0.7,enableContactDetection=false))

# Kinematic simulation
Modia3D.initAnalysis!(fileMeshes)

for time = linspace(0.0, 2.0, 101)
  r1[3]  = 2*time
  Modia3D.set_r!(fileMeshes.fileMesh1, r1)

  Modia3D.updatePosition!(fileMeshes)

  # Visualize shapes
  Modia3D.visualize!(fileMeshes,time)
end
Modia3D.closeAnalysis!(fileMeshes)

println("... success of Move_SolidFileMesh.jl!")

end
