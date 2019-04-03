module Simulate_PendulumWithDamper

using  Modia3D
import Modia3D.ModiaMath

# Solids
groundMaterial = Modia3D.Material(color="DarkGreen", transparency=0.5)
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")

# Damper
@forceElement Damper(; d=1.0) begin
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(damper::Damper, sim::ModiaMath.SimulationState)
    damper.tau.value = -damper.d*damper.w.value
end


@assembly PendulumWithDamper(;Lx = 1.0, Ly=0.2*Lx, Lz=0.2*Lx, m=1.0, g=9.81) begin
   world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))

   # Pendulum
   body   = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), Modia3D.MassProperties(m=m), vmat1))
   frame1 = Modia3D.Object3D(body; r=[-Lx/2, 0.0, 0.0])
   cyl    = Modia3D.Object3D(frame1,Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2))

   # Connect pendulum to world with a damper in the joint
   rev    = Modia3D.Revolute(world, frame1)
   d     = Damper(d=0.2)
   damper = Modia3D.AdaptorForceElementToFlange(w=d.w, tau=d.tau)
   Modia3D.connect(damper, rev)
end
pendulum = PendulumWithDamper(Lx=1.6, m=0.5, sceneOptions= Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3, useOptimizedStructure = false))
model = Modia3D.SimulationModel( pendulum )
result = ModiaMath.simulate!(model, stopTime=5.0, interval=0.1, tolerance=1e-4, log=true)

ModiaMath.plot(result, ["rev.phi", "rev.w", "rev.a", "rev.tau"])

println("... success of Simulate_PendulumWithDamper.jl!")
end
