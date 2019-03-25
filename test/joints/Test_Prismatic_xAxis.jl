module Test_Prismatic_xAxis

using  Modia3D
import Modia3D.ModiaMath

@assembly FallingBall(;h=1.0) begin
   world  = Object3D()
   sphere = Object3D( Modia3D.Solid(Modia3D.SolidSphere(0.1), "Aluminium") )

   # Constrain sphere movement (initial placement at position [0,h,0])
   prismatic = Modia3D.Prismatic(world, sphere, axis=1, s_start=h)
end


gravField = Modia3D.UniformGravityField(n=[-1,0,0])
fallBall = FallingBall(h=3.5, sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.3))
simulationModel = Modia3D.SimulationModel( fallBall, stopTime=1.0 )
result          = ModiaMath.simulate!(simulationModel)
ModiaMath.plot(result, ("prismatic.s", "prismatic.v"))

println("... success of Test_Prismatic_xAxis.jl!")

end
