module collision_kinematic_ballWithBall

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh
vmat11 = Modia3D.Material(color="Red" , transparency=0.5)
vmat2 = deepcopy(vmat1)                                           # material of convex decomposition of SolidFileMesh
vmat2.transparency = 0.7

cmat = Modia3D.defaultContactMaterial()

@assembly BallWithBall begin
  world = Modia3D.Object3D(visualizeFrame=true)

  sphereMoving     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(0.5) , "Aluminium", vmat11; contactMaterial = cmat); r=[0.0, 0.0, 0.0],  fixed=false) #R=ModiaMath.rot2(-pi/2),

  sphere     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(2.0) , "Aluminium", vmat1; contactMaterial = cmat); r=[-3.0, 0.0, 0.0], fixed=true) # R=ModiaMath.rot2(-pi/3),
end

gravField = Modia3D.UniformGravityField(g=9.81, n=[-1,0,0])
ballBox = BallWithBall(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=true, visualizeSupportPoints=false))

Modia3D.initAnalysis!(ballBox)

tStart=0.0
tEnd  =0.00000001

Lx = 3.5
s =  3.15 #2.7 #2.6996028006546 #2.249
for time = range(tStart, stop=tEnd, length=2)
  delta_phi = Modia3D.linearMovement(pi/3, tStart, tEnd, time)
  Modia3D.set_r!(ballBox.sphereMoving, [-s, 0, 0.0])
  Modia3D.updatePosition!(ballBox)

  Modia3D.setComputationFlag(ballBox)
  Modia3D.selectContactPairs!(ballBox)
  Modia3D.setComputationFlag(ballBox)
  Modia3D.getDistances!(ballBox)

  Modia3D.visualize!(ballBox,time)

  time += 0.01
end

Modia3D.closeAnalysis!(ballBox)
println("... success of collision_kinematic_ballWithBall.jl!")
end
