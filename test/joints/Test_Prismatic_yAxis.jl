"""
    module Simulate_FallingBall1

Models and simulates a sphere that is falling downwards along the y-axis (constrained via
a prismatic joint), without collision handling.

See also models [`Simulate_FallingBall2`](@ref) (falling ball with collision handling).
"""
module Test_Prismatic_yAxis

using  Modia3D
import Modia3D.ModiaMath

@assembly FallingBall(;h=1.0) begin
   world  = Object3D()
   sphere = Object3D( Modia3D.Solid(Modia3D.SolidSphere(0.1), "Aluminium") )

   # Constrain sphere movement (initial placement at position [0,h,0])
   prismatic = Modia3D.Prismatic(world, sphere, axis=2, s_start=h)
end

fallBall = FallingBall(h=3.5, sceneOptions=Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3))
simulationModel = Modia3D.SimulationModel( fallBall, stopTime=1.0 )
result          = ModiaMath.simulate!(simulationModel)
ModiaMath.plot(result, ("prismatic.s", "prismatic.v"))

println("... success of Test_Prismatic_yAxis.jl!")

end
