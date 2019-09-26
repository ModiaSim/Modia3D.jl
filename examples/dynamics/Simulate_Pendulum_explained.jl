module Simulate_Pendulum

using  Modia3D
import Modia3D.ModiaMath

# visualization material
visuMaterial1 = Modia3D.Material(color="LightBlue", transparency=0.5)
visuMaterial2 = Modia3D.Material(color="Red")

@assembly Pendulum(;Lx = 1.0, Ly=0.2*Lx, Lz=0.2*Lx) begin
   # world is the reference point and parent Object3D of all other Object3D
   world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.4*Lx))

   # Pendulum
   # the beam is made of Aluminium and is visualized with blue color
   beam     = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), "Aluminium", visuMaterial1))
   # the bearing is a helping Object3D, its parent is the beam
   bearing  = Modia3D.Object3D(beam; r=[-Lx/2, 0.0, 0.0])
   # for visualizing the revolute joint
   cylinder = Modia3D.Object3D(bearing, Modia3D.Cylinder(Ly/2,1.2*Ly; material=visuMaterial2))

   # the revolute joint connects the world with the bearing, which belongs to the beam
   revolute    = Modia3D.Revolute(world, bearing)
end
# a Pendulum object is created, where SceneOptions are set (in this case visualization properties of frames)
pendulum = Pendulum(Lx=1.6, sceneOptions=Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3))

# a simulation model is created, simulated and potted
model  = Modia3D.SimulationModel(pendulum)
result = ModiaMath.simulate!(model, stopTime=5.0, tolerance=1e-6,log=true)
ModiaMath.plot(result, ["revolute.phi", "revolute.w", "revolute.a"])

println("... success of Simulate_Pendulum.jl!")
end
