"""
    module Simulate_FallingBall2

Models and simulates a *solid sphere* that is falling downwards along the y-axis (constrained via 
a prismatic joint) on a *solid box*, **with collision handling**. 

Note, a **Solid** is used in collision handling, if a **contactMaterial** is defined for the
solid. By default, collision handling between two Object3Ds is disabled, if connected by a joint.
This default behaviour can be switched off with keyword argument `canCollide=true`. This option
is used in this model for the `Prismatic` joint that constraints the movement of the `Sphere`
(otherwise, no collision handling would take place).

See also models [`Simulate_FallingBall1`](@ref) (falling ball without collision handling).
"""
module Simulate_FallingBall2

using Modia3D
import ModiaMath

groundMaterial  = Modia3D.Material(color="DarkGreen", transparency=0.5)
ballMaterial    = Modia3D.Material(color="LightBlue", transparency=0.5)
contactMaterial = Modia3D.defaultContactMaterial()


@assembly FallingBall2(;h=1.0, D=0.1) begin
   world     = Object3D()
   ground    = Object3D(world, Modia3D.Solid(Modia3D.SolidBox(1.0,0.1,1.0), nothing, groundMaterial,
                                             contactMaterial=contactMaterial), r = [0.5, -0.05, 0.0])
   sphere    = Object3D(Modia3D.Solid(Modia3D.SolidSphere(D), "Aluminium", ballMaterial,
                                      contactMaterial=contactMaterial))
   prismatic = Modia3D.Prismatic(ground, sphere, axis=2, s_start=h, canCollide=true)  
end

fallingBall = FallingBall2()

# Modia3D.visualizeAssembly!( fallingBall )

simulationModel = Modia3D.SimulationModel( fallingBall, stopTime=2 )
result          = ModiaMath.simulate!(simulationModel, log=true)
ModiaMath.plot(result, ("prismatic.s", "prismatic.v"))

println("... success of Simulate_FallingBall2.jl!")

end
