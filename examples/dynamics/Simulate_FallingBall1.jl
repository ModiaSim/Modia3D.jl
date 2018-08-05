"""
    module Simulate_FallingBall1

Models and simulates a sphere that is falling downwards along the y-axis (constrained via 
a prismatic joint), without collision handling.

See also models [`Simulate_FallingBall2`](@ref) (falling ball with collision handling).
"""
module Simulate_FallingBall1

using Modia3D
import ModiaMath

@assembly FallingBall(;h=1.0) begin
   world  = Object3D()
   sphere = Object3D( Modia3D.Solid(Modia3D.SolidSphere(0.1), "Aluminium") )

   # Constrain sphere movement (initial placement at position [0,h,0])
   prismatic = Modia3D.Prismatic(world, sphere, axis=2, s_start=h)  
end

simulationModel = Modia3D.SimulationModel( FallingBall(h=1.5), stopTime=0.5 )
result          = ModiaMath.simulate!(simulationModel)
ModiaMath.plot(result, ("prismatic.s", "prismatic.v"))

println("... success of Simulate_FallingBall1.jl!")

end
