module contactForceLaw_ballWithBox_Prismatic

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh
vmat2 = deepcopy(vmat1)                                           # material of convex decomposition of SolidFileMesh
vmat2.transparency = 0.7

cmat = Modia3D.ElasticContactMaterial(name="Steel", mu_r =0.1) #
#cmat = Modia3D.ContactMaterialElastic(c=1e9, d=0.0, mu1=0.1)

@assembly TwoBoxes begin
  world = Modia3D.Object3D(visualizeFrame=true)

  boxMoving     = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidSphere(0.5) , "Steel", vmat1; contactMaterial = cmat) )
  helpFrame1 = Modia3D.Object3D(visualizeFrame=false)
  helpFrame2 = Modia3D.Object3D(visualizeFrame=false)
  prisX = Modia3D.Prismatic(world, helpFrame1; axis=1, v_start=2.0, s_start=0.0, canCollide=true)
  prisY = Modia3D.Prismatic(helpFrame1, helpFrame2, axis=2, v_start=0.0, s_start=0.0, canCollide=true)
  prisZ = Modia3D.Prismatic(helpFrame2, boxMoving, axis=3, v_start=-3.0, s_start=0.0, canCollide=true)


  box     = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(20.0,4.0,1.0) , "Steel", vmat1; contactMaterial = cmat); r=[0.0, 0.0, -3.0], fixed=true) # R=ModiaMath.rot2(-pi/3),
end


gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
threeD = TwoBoxes(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=true, visualizeSupportPoints=true))


# Modia3D.visualizeAssembly!( threeD )


model = Modia3D.SimulationModel( threeD )
ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=0.5, tolerance=1e-8,interval=0.001, log=true)
ModiaMath.plot(result, [("prisX.s","prisY.s","prisZ.s"),("prisX.v"), "prisX.a"])
# ModiaMath.plot(result, ["prisX.s","prisY.s","prisZ.s","prisX.v","prisY.v","prisZ.v"])


println("... success of contactForceLaw_ballWithBox_Prismatic.jl!")

end
