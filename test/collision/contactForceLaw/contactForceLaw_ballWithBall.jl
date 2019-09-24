module contactForceLaw_ballWithBall

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh
vmat11 = Modia3D.Material(color="Red" , transparency=0.5)
vmat2 = deepcopy(vmat1)                                           # material of convex decomposition of SolidFileMesh
vmat2.transparency = 0.7

cmat = "Steel"

@assembly BallWithBall begin
  world = Modia3D.Object3D(visualizeFrame=true)

  sphereMoving     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(0.5) , "Steel", vmat11; contactMaterial = cmat); r=[0.0, 0.0, 0.0],  fixed=false ) # , R=ModiaMath.rot2(-pi/2) )

  sphere     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(1.0) , "Steel", vmat1; contactMaterial = cmat); r=[-3.0, 0.0, 0.0], fixed=true) #, R=ModiaMath.rot2(-pi/3) )
end

gravField = Modia3D.UniformGravityField(g=9.81, n=[-1,0,0])
threeD = BallWithBall(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=true, visualizeSupportPoints=true))


model = Modia3D.SimulationModel( threeD )
result = ModiaMath.simulate!(model; stopTime=10.0, tolerance=1e-8,interval=0.001, log=false)

println("... success of contactForceLaw_ballWithBall.jl!")
end
