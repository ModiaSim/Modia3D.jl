"""
    module Simulate_NewtonsCradle

Model of [Newtons' Cradle](https://en.wikipedia.org/wiki/Newton%27s_cradle).

The balls of the Newton Cradle are initially placed in such a way, that there is a small
distance between the balls, so the balls are **not** touching each other. This distance was
determined (by trial-and-error) in such a way, that collisions between the balls
occur in sequence (so if ball1 hits ball2, then ball3 is hit by ball2 only after the
contact between ball1 and ball2 has already ended; this can be easily determined by
setting `simulate!(..., log=true)`). Only in this case, the simulation matches the
real experiment. If the balls are directly touching each other, multiple contacts
occur at the same time instant and the simulation result is then no longer according
to reality.

See also:

M. Payr (2008): **[An Experimental and Theoretical Study of Perfect Multiple
                Contact Collisions in Linear Chains of Balls.](http://www.zfm.ethz.ch/e/dynamics/payr_collisions.htm)**
                Dissertation, ETH Zürich.
"""
module Simulate_NewtonsCradle

using  Modia3D
import Modia3D.ModiaMath

vmatGraphics = Material(color="LightBlue", transparency=0.1)   # material of Graphics
vmatSolids   = Material(color="Red"      , transparency=0.0)   # material of solids

diameter   = 0.02
lengthWire = 0.15
rsmall     = diameter/100
#rsmall = 0.0

Lx = diameter/2
Ly = 5*diameter
Lz = Lx

@assembly Pendulum() begin
  wire = Object3D( Cylinder(diameter/5, lengthWire; material=vmatGraphics) )

  frame1 = Object3D(wire; r=[0.0, 0.0,  lengthWire/2])
  frame2 = Object3D(wire; r=[0.0, 0.0, -lengthWire/2])

  cylinder = Object3D(frame1, Cylinder(0.6*Lx, 1.4*Lx; material=vmatSolids); R=ModiaMath.rot2(-pi/2))
  sphere   = Object3D(frame2, Solid(SolidSphere(diameter), "Steel", vmatSolids; contactMaterial="BilliardBall") )
end

@assembly NewtonsCradle() begin
  world  = Object3D(visualizeFrame=true)
  box    = Object3D(world, Box(Lx, Ly, Lz; material=vmatGraphics))

  frame1 = Object3D(box; r=[0.0, -2*(diameter+rsmall), 0.0])
  frame2 = Object3D(box; r=[0.0,   -(diameter+rsmall), 0.0])
  frame3 = Object3D(box; r=[0.0,                  0.0, 0.0])
  frame4 = Object3D(box; r=[0.0,    (diameter+rsmall), 0.0])
  frame5 = Object3D(box; r=[0.0,  2*(diameter+rsmall), 0.0])

  pendulum1 = Pendulum()
  pendulum2 = Pendulum()
  pendulum3 = Pendulum()
  pendulum4 = Pendulum()
  pendulum5 = Pendulum()

  rev1 = Revolute(frame1, pendulum1.frame1; axis = 1 , phi_start =  -pi/3)
  rev2 = Revolute(frame2, pendulum2.frame1; axis = 1 )#,phi_start =  -pi/3)
  rev3 = Revolute(frame3, pendulum3.frame1; axis = 1 ) #, phi_start =  -pi/3)
  rev4 = Revolute(frame4, pendulum4.frame1; axis = 1 )#,phi_start =  pi/3 ) #
  rev5 = Revolute(frame5, pendulum5.frame1; axis = 1 )#,phi_start =  pi/3 ) #
end

gravField     = UniformGravityField(g=9.81, n=[0,0,-1])
newtonsCradle = NewtonsCradle(sceneOptions=SceneOptions(elasticContactReductionFactor=1.0, gravityField=gravField, visualizeFrames=false,
                                                        defaultFrameLength=2*diameter, enableContactDetection=true))

# Modia3D.visualizeAssembly!( newtonsCradle )
model  = SimulationModel( newtonsCradle )
result = ModiaMath.simulate!(model; stopTime=10.0, tolerance=1e-8, log=false)

println("... success of examples/collisions/Simulate_NewtonsCradle.jl!")
end
