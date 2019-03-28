module Simulate_Pendulum

using  Modia3D
import Modia3D.ModiaMath


# Solids
groundMaterial = Modia3D.Material(color="DarkGreen", transparency=0.5)
# massProperties = Modia3D.MassProperties()
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")


rCM = zeros(3)
#rCM[2] = 5.0
#I2 = fill(100.0,(9))
I = fill(0.0,(3,3))
#I[3] = 50.0
#I[7] = 80.0
#println("I = ",I)
massProp = Modia3D.MassProperties(5.0, rCM, I)

@assembly Pendulum(;Lx = 1.0, Ly=0.2*Lx, Lz=0.2*Lx, m=1.0) begin
   world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))

   # Pendulum
   body   = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), massProp, vmat1))
   frame1 = Modia3D.Object3D(body; r=[-Lx/2, 0.0, 0.0])
   cyl    = Modia3D.Object3D(frame1,Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2))

   # Connect pendulum to world
   rev    = Modia3D.Revolute(world, frame1)
end

const Lx = 1.6
const m  = 0.5
# Modia3D.visualizeAssembly!( Pendulum(Lx=Lx, m=m) )

pendulum = Pendulum(Lx=Lx, m=m, sceneOptions=Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3))
model    = Modia3D.SimulationModel( pendulum; useOptimizedStructure = false )
result   = ModiaMath.simulate!(model, stopTime=10.0, tolerance=1e-6,log=false)

ModiaMath.plot(result, ["rev.phi", "rev.w", "rev.a"])


println("... success of Simulate_Pendulum.jl!")
end
