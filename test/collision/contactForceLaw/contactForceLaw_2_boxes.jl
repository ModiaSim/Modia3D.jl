module contactForceLaw_2_boxes

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh
vmat2 = deepcopy(vmat1)                                           # material of convex decomposition of SolidFileMesh
vmat2.transparency = 0.7

cmat = Modia3D.ElasticContactMaterial(name="DryWood", mu_r = 0.0, mu_k = 0.0, cor=1.0)

zDim = 1.0
boxDim = 0.3

@assembly ThreeDFiles begin
  world = Modia3D.Object3D(visualizeFrame=false)

  boxMoving     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(boxDim,boxDim,boxDim,rsmall=0.0) , "DryWood", vmat1; contactMaterial = cmat); r=[1.0, 0.0, boxDim/2], fixed=false ) # ,R=ModiaMath.rot2(-pi/3) ) #

  box     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(5.0,2.0,zDim,rsmall=0.0) , "DryWood", vmat1; contactMaterial = cmat); r=[0.0, 0.0, -zDim/2], fixed=true )
end


gravField = Modia3D.UniformGravityField(g=0.0, n=[0,0,-1])
threeD = ThreeDFiles(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=true, visualizeSupportPoints=true))


# Modia3D.visualizeAssembly!( threeD )


model = Modia3D.SimulationModel( threeD )
ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=2.0, tolerance=1e-8,interval=0.001, log=true)

ModiaMath.plot(result, ["boxMoving.r","boxMoving.v"])

println("... success of contactForceLaw_2_boxes.jl!")

end
