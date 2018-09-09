module Simulate_DamperMacro

using  Modia3D
import Modia3D.ModiaMath



# Damper
@forceElement Damper(; d=1.0) begin
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(damper::Damper, sim::ModiaMath.SimulationState)
  damper.tau.value = -damper.d*damper.w.value
end


# Pendulum
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")

@assembly PendulumWithDamper(;Lx = 1.0, Ly=0.2*Lx, Lz=0.2*Lx, m=1.0) begin
   world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))

   # Pendulum
   body   = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), Modia3D.MassProperties(m=m), vmat1))
   frame1 = Modia3D.Object3D(body; r=[-Lx/2, 0.0, 0.0])
   cyl    = Modia3D.Object3D(frame1,Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2))

   # Connect pendulum to world
   rev        = Modia3D.Revolute(world, frame1)

   d     = Damper(d=0.2)
   damper = Modia3D.AdaptorForceElementToFlange(w=d.w, tau=d.tau)
   Modia3D.connect(damper, rev)
end
pendulum = PendulumWithDamper(Lx=1.6, m=0.5, sceneOptions = Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3))
model    = Modia3D.SimulationModel( pendulum )
result   = ModiaMath.simulate!(model, stopTime=5.0, log=false, tolerance=1e-4)

ModiaMath.plot(result, ["rev.phi", "rev.w", "rev.a", "rev.tau"])

println("... success of Simulate_DamperMacro.jl!")
end
