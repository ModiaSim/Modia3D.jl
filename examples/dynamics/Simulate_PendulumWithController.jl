module Simulate_PendulumWithController

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


# Controller
@forceElement Controller(; k1=10.0, k2=10.0, T2=0.01, freqHz=0.5, A=1.0) begin
    PI_x    = ModiaMath.RealScalar("PI_x"   , start=0.0, info="State of PI controller", numericType=ModiaMath.XD_EXP)
    PI_derx = ModiaMath.RealScalar("PI_derx", start=0.0, info="= der(PI_x)"           , numericType=ModiaMath.DER_XD_EXP, integral=PI_x)
    sine_y  = ModiaMath.RealScalar("sine_y" , start=0.0, info="= sin(2*pi*f*time)"    , numericType=ModiaMath.WR)
    phi = ModiaMath.RealScalar("phi", causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(c::Controller, sim::ModiaMath.SimulationState)
    c.sine_y.value  = c.A*sin(2*pi*c.freqHz*sim.time)
    gain_y          = c.k1*(c.sine_y.value - c.phi.value)
    PI_u            = gain_y - c.w.value
    c.PI_derx.value = PI_u/c.T2
    c.tau.value = c.k2*(c.PI_x.value + PI_u)
end


# Pendulum
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")

@assembly PendulumWithController(;Lx = 1.0, Ly=0.2*Lx, Lz=0.2*Lx, m=1.0) begin
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
   c = Controller()
   controller = Modia3D.AdaptorForceElementToFlange(phi=c.phi, w=c.w, tau=c.tau)
   Modia3D.connect(controller, rev)
end
pendulum = PendulumWithController(Lx=1.6, m=0.5, sceneOptions = Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3))
model    = Modia3D.SimulationModel( pendulum; useOptimizedStructure = true )
result   = ModiaMath.simulate!(model, stopTime=5.0, log=true, tolerance=1e-4)

ModiaMath.plot(result, [("rev.phi","c.sine_y"), "rev.w", "rev.a", "rev.tau"])

println("... success of Simulate_PendulumWithController.jl!")
end
