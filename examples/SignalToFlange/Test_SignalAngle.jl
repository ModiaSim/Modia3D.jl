module Test_SignalAngle

using  Modia3D
import Modia3D.ModiaMath

@signal Signal(;A=5.0, B=2.0) begin
   y = ModiaMath.RealScalar("y", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::Signal, sim::ModiaMath.SimulationState)
    signal.y.value  = signal.A*sin(signal.B * sim.time)
end


vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")

@assembly PendulumDrivenKinematically(;Lx = 1.0, Ly=0.2*Lx, Lz=0.2*Lx, m=1.0) begin
   world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))

   # Pendulum
   body   = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), Modia3D.MassProperties(m=m), vmat1))
   frame1 = Modia3D.Object3D(body; r=[-Lx/2, 0.0, 0.0])
   cyl    = Modia3D.Object3D(frame1,Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2))

   rev    = Modia3D.Revolute(world, frame1)

   sig    = Signal()
   signal = Modia3D.SignalToFlangeAngle(sig.y)
   Modia3D.connect(signal, rev)
end

pendulum = PendulumDrivenKinematically(Lx=1.6, m=0.5, sceneOptions = Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3))
model    = Modia3D.SimulationModel(pendulum, analysis=ModiaMath.KinematicAnalysis)
# ModiaMath.print_ModelVariables(model)
result   = ModiaMath.simulate!(model, stopTime=5.0)
ModiaMath.plot(result, ["sig.y", "rev.phi"] )

println("... success of Test_SignalAngle.jl!")
end
