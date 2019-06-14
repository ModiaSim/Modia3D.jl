module tests_SolidFileMesh

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh
vmat2 = deepcopy(vmat1)                                           # material of convex decomposition of SolidFileMesh
vmat2.transparency = 0.7

cmat = Modia3D.ElasticContactMaterial("Steel")

@assembly ThreeDFiles begin
  world = Modia3D.Object3D(visualizeFrame=true)

  boxMoving     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(0.1,0.1,0.1) , "Steel", vmat1; contactMaterial = cmat); r=[0.0, 0.0, 3.0], fixed=false ,R=ModiaMath.rot2(-pi/3) )


  box     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(3.0,2.0,5.0) , "Steel", vmat1; contactMaterial = cmat); r=[-3.0, 0.0, 2.5], fixed=true) # R=ModiaMath.rot2(-pi/3),
end


gravField = Modia3D.UniformGravityField(g=9.81, n=[-1,0,0])
threeD = ThreeDFiles(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=true, visualizeSupportPoints=true))


# Modia3D.visualizeAssembly!( threeD )


model = Modia3D.SimulationModel( threeD )
ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=0.64, tolerance=1e-6,interval=0.001, log=false)

ModiaMath.plot(result, ["boxMoving.r","boxMoving.v"])

println("... success of tests_SolidFileMesh.jl!")

end
