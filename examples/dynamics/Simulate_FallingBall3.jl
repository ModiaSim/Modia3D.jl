"""
    module Simulate_FallingBall3

Models and simulates a sphere that is falling downwards as 6 dof body
(described by quaternions), **without** collision handling.

See also models 

- [`Simulate_FallingBall1`](@ref) (falling ball constrained with prismatic joint),
- [`Simulate_FallingBall2`](@ref) (falling ball constrained with prismatic joint and collision handling).
"""
module Simulate_FallingBall3

using  Modia3D
import Modia3D.ModiaMath

# Solids
groundMaterial = Modia3D.Material(color="DarkGreen", transparency=0.5)
ballMaterial   = Modia3D.Material(color="LightBlue", transparency=0.5)


@assembly FallingBall3(;h_start = 1.0, m = 0.1) begin
   world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5))
   ground = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(1.0,0.1,1.0), nothing, groundMaterial),
                                                  r = [0.5, -0.05, 0.0])
   ball   = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(0.1), "Aluminium", ballMaterial),
                             fixed=false, r=[0.5,h_start,0.0])
end

fallingBall = FallingBall3()

# Modia3D.visualizeAssembly!( fallingBall )

simulationModel = Modia3D.SimulationModel( fallingBall, stopTime=0.6 )
result          = ModiaMath.simulate!(simulationModel, log=true) 

ModiaMath.plot(result, ["ball.r[2]"])



println("... success of Simulate_FallingBall3.jl!")

end
