module Simulate_DoublePendulumWithDampers

using  Modia3D
import Modia3D.ModiaMath


# Solids
groundMaterial = Modia3D.Material(color="DarkGreen", transparency=0.5)
# massProperties = Modia3D.MassProperties()
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")


@forceElement Damper(; d=1.0) begin
   w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
   tau = ModiaMath.RealScalar("tau",  causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(damper::Damper, sim::ModiaMath.SimulationState)
   damper.tau.value = -damper.d*damper.w.value
end


@assembly Bar(;Lx = 0.1, Ly=Lx/5, Lz=Ly, m=1.0) begin
   frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), Modia3D.MassProperties(m=m), vmat1))
   frame1 = Modia3D.Object3D(frame0; r=[-Lx/2, 0.0, 0.0])
   frame2 = Modia3D.Object3D(frame0; r=[ Lx/2, 0.0, 0.0])
   cyl    = Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2)
   cyl1   = Modia3D.Object3D(frame1, cyl)
   cyl2   = Modia3D.Object3D(frame2, cyl)
end

@assembly DoublePendulumWithDampers(;Lx = 1.0, m=1.0) begin
   world = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))
   bar1  = Bar(Lx=Lx, m=m)
   bar2  = Bar(Lx=Lx, m=m)
   rev1  = Modia3D.Revolute(world, bar1.frame1)
   rev2  = Modia3D.Revolute(bar1.frame2, bar2.frame1)
   d1     = Damper(d=0.2)
   damper1 = Modia3D.AdaptorForceElementToFlange(w=d1.w, tau=d1.tau)
   Modia3D.connect(damper1, rev1)
   d2     = Damper(d=0.3)
   damper2 = Modia3D.AdaptorForceElementToFlange(w=d2.w, tau=d2.tau)
   Modia3D.connect(damper2, rev2)
end

model = Modia3D.SimulationModel( DoublePendulumWithDampers(); useOptimizedStructure = true)
result = ModiaMath.simulate!(model, stopTime=5.0, tolerance=1e-6,interval=0.001,log=true)

ModiaMath.plot(result, [("rev1.phi", "rev2.phi"),
                        ("rev1.w"  , "rev2.w"),
                        ("rev1.a"  , "rev2.a")])


println("... success of Simulate_DoublePendulumWithDampers.jl!")
end
