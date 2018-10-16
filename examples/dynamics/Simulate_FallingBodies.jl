"""
    module Simulate_FallingBodies

Models and simulates a sphere that is falling downwards as 6 dof body
(described by quaternions), **without** collision handling.

See also models

- [`Simulate_FallingBall1`](@ref) (falling ball constrained with prismatic joint),
- [`Simulate_FallingBall2`](@ref) (falling ball constrained with prismatic joint and collision handling).
"""
module Simulate_FallingBodies

using  Modia3D
import Modia3D.ModiaMath
using  Modia3D.StaticArrays

# Solids
groundMaterial  = Modia3D.Material(color="DarkGreen", transparency=0.5)
cylMaterial     = Modia3D.Material(color="LightBlue", transparency=0.5)
cm              = Modia3D.ContactMaterialElastic(mu1=0.1)

fileRod   = joinpath(Modia3D.path, "objects/engine/rod.obj")
fileCrank = joinpath(Modia3D.path, "objects/engine/crank.obj")

@assembly FallingBodies() begin
   world  = Object3D(Modia3D.CoordinateSystem(0.5))
   ground = Object3D(world, Modia3D.Solid(Modia3D.SolidBox(2.0,0.1,2.0), nothing, groundMaterial,
                                          contactMaterial=cm), r = [1.0, -0.05, 0.0])

   rod      = Object3D(world, Modia3D.Solid(Modia3D.SolidFileMesh(fileRod  , 0.2), "Aluminium",              contactMaterial=cm), r=[0.62,0.82,0.5], q=ModiaMath.qrot123(pi/3, 0.0, pi/4), fixed=false)
   crank    = Object3D(world, Modia3D.Solid(Modia3D.SolidFileMesh(fileCrank, 0.2), "Aluminium",              contactMaterial=cm), r=[0.6,0.6,0.2], fixed=false)
   #cylinder = Object3D(world, Modia3D.Solid(Modia3D.SolidCylinder(0.3,0.1)       , "Aluminium", cylMaterial, contactMaterial=cm), r=[0.5,0.4,0.8], q=ModiaMath.qrot1(pi/3), fixed=false)
end

fallingBodies = FallingBodies()

# Modia3D.visualizeAssembly!( fallingBall )

simulationModel = Modia3D.SimulationModel( fallingBodies, stopTime=0.4, scaleConstraintsAtEvents = false )
#Modia3D.print_ModelVariables(simulationModel)
result = ModiaMath.simulate!(simulationModel, log=true)

# ModiaMath.plot(result, ["ball.r[2]"; "ball.v[1]"; "ball.v[3]"])



println("... success of Simulate_FallingBodies.jl!")

end
