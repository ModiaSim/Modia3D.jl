module contactForceLaw_ballWithBox_45Deg

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh
vmat2 = deepcopy(vmat1)                                           # material of convex decomposition of SolidFileMesh
vmat2.transparency = 0.7

cmat = "Steel"

@assembly ThreeDFiles begin
  world = Modia3D.Object3D(visualizeFrame=true)

  sphereMoving     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(0.5) , "Steel", vmat1; contactMaterial = cmat); r=[0.0, 0.0, 0.0],  fixed=false, v_start=[2.0,0.0,-3.0]) # R=ModiaMath.rot2(-pi/2),

  box     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(20.0,4.0,1.0) , "Steel", vmat1; contactMaterial = cmat); r=[0.0, 0.0, -3.0], fixed=true ) #, R=ModiaMath.rot2(-pi/3))
end

gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
threeD = ThreeDFiles(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=true, visualizeSupportPoints=true))


model = Modia3D.SimulationModel( threeD )
ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=4.0, tolerance=1e-8,interval=0.001, log=false)
ModiaMath.plot(result, ["sphereMoving.r","sphereMoving.v"])


println("... success of contactForceLaw_ballWithBox_45Deg.jl!")

end
