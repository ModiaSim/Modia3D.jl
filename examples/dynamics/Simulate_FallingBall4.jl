"""
    module Simulate_FallingBall4

Models and simulates a sphere that is falling downwards as 6 dof body
(described by quaternions), **with** collision handling.

See also models

- [`Simulate_FallingBall1`](@ref) (falling ball constrained with prismatic joint),
- [`Simulate_FallingBall2`](@ref) (falling ball constrained with prismatic joint and collision handling).
"""
module Simulate_FallingBall4

using  Modia3D
import Modia3D.ModiaMath

# Solids
groundMaterial  = Modia3D.Material(color="DarkGreen", transparency=0.5)
ballMaterial    = Modia3D.Material(color="LightBlue", transparency=0.5)
contactMaterial = Modia3D.ContactMaterialElastic(mu1=0.5)


@assembly FallingBall4(;h_start = 1.0, D=0.1) begin
   world  = Object3D(Modia3D.CoordinateSystem(0.5))
   ground = Object3D(world, Modia3D.Solid(Modia3D.SolidBox(2.0,0.1,2.0), nothing, groundMaterial,
                                          contactMaterial=contactMaterial), r = [1.0, -0.05, 0.0])
   ball   = Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(D), "Aluminium", ballMaterial,
                                          contactMaterial=contactMaterial),
                     fixed=false, r=[0.5,h_start,0.0], v_start=[0.2,0,0], visualizeFrame=true)
end

fallingBall = FallingBall4()

# Modia3D.visualizeAssembly!( fallingBall )

simulationModel = Modia3D.SimulationModel( fallingBall, stopTime=2.5, scaleConstraintsAtEvents = false ,useOptimizedStructure = false )
#Modia3D.print_ModelVariables(simulationModel)
result = ModiaMath.simulate!(simulationModel, log=true)

ModiaMath.plot(result, ["ball.r[2]"; "ball.v[1]"; "ball.v[2]"; "ball.v[3]"])



println("... success of Simulate_FallingBall4.jl!")

end
