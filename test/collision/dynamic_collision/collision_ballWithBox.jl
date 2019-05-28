module collision_ballWithBox

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh
vmat2 = deepcopy(vmat1)                                           # material of convex decomposition of SolidFileMesh
vmat2.transparency = 0.7

#c = 1e9, d = 100.0
cmat = Modia3D.ContactMaterialElastic(c=1e10)

@assembly ThreeDFiles begin
  world = Modia3D.Object3D(visualizeFrame=true)

  sphereMoving     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(0.5) , "Aluminium", vmat1; contactMaterial = cmat); r=[0.0, 0.0, 0.0],  fixed=false) # , R=ModiaMath.rot2(-pi/2) )

  box     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(1.0,4.0,4.5) , "Aluminium", vmat1; contactMaterial = cmat); r=[-3.0, 0.0, -2.0], fixed=true) # R=ModiaMath.rot2(-pi/3),
end

gravField = Modia3D.UniformGravityField(g=9.81, n=[-1,0,0])
threeD = ThreeDFiles(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=true, visualizeSupportPoints=true))


model = Modia3D.SimulationModel( threeD )
result = ModiaMath.simulate!(model; stopTime=10.0, tolerance=1e-8,interval=0.001, log=false)


println("... success of collision_ballWithBox.jl!")

end
