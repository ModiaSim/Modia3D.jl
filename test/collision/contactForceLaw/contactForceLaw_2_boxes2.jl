module tests_boxes2

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh
vmat2 = deepcopy(vmat1)                                           # material of convex decomposition of SolidFileMesh
vmat2.transparency = 0.7

cmat = Modia3D.ElasticContactMaterial(name="DryWood", mu_r = 0.0, mu_k = 0.0, cor=0.5)

zDim = 1.0
boxDim = 0.3
rsmall = 0.005
@assembly ThreeDFiles begin
  world = Modia3D.Object3D(visualizeFrame=true)

  boxMoving = Modia3D.ContactBox(world, scale=[boxDim,boxDim,boxDim], rsmall=rsmall,
                                 massProperties="DryWood", material=vmat1,
                                 contactMaterial = cmat, r=[1.0, 0.0, 2*boxDim], fixed=false)
  box     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(5.0,2.0,zDim) , "DryWood", vmat1; contactMaterial = cmat); r=[0.0, 0.0, -zDim/2], fixed=true )
end


gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
threeD = ThreeDFiles(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=false, visualizeSupportPoints=false))

# Modia3D.visualizeAssembly!( threeD )


model = Modia3D.SimulationModel( threeD )
ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=2.0, tolerance=1e-8,interval=0.001, log=true)

ModiaMath.plot(result, ["boxMoving.box.r[3]","boxMoving.box.v[3]"])

println("... success of tests_SolidFileMesh.jl!")

end
