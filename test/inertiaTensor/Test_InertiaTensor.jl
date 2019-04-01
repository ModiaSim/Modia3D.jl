module Test_SignalTorque

using  Modia3D
import Modia3D.ModiaMath


Dx = 0.7
Lx = 1.0
Ly = 0.3
Lz = 0.3

@forceElement Damper(; d=1.0) begin
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(damper::Damper, sim::ModiaMath.SimulationState)
    damper.tau.value = damper.d*sim.time
end



vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")

m = 15.0

rCM = zeros(3)

rCM[1] = 20.0
rCM[2] = 5.0
rCM[3] = 10.0


I = fill(30.0,(3,3))
I[7] = 100.0
I[8] = 80.0
#println("I = ",I)
massProp = Modia3D.MassProperties(m, rCM, I)

@assembly Pendulum() begin
   world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))

   # Pendulum
   body   = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), massProp, vmat1))
   frame1 = Modia3D.Object3D(body; r=[-Lx/2, 0.0, 0.0])
   cyl    = Modia3D.Object3D(frame1,Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2))

   rev1    = Modia3D.Revolute(world, frame1; axis = 3)

   d     = Damper(d=10.0)
   damper = Modia3D.AdaptorForceElementToFlange(w=d.w, tau=d.tau)
   Modia3D.connect(damper, rev1)
end

gravField = Modia3D.UniformGravityField(n=[1,0,0])

pendulum = Pendulum(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true,defaultFrameLength=0.3, enableContactDetection=false))
model    = Modia3D.SimulationModel(pendulum; useOptimizedStructure = true)
result   = ModiaMath.simulate!(model, stopTime=5.0, tolerance=1e-6,interval=0.001, log=false)
ModiaMath.plot(result, ["rev1.phi", "rev1.tau"] )

println("... success of Test_SignalTorque.jl!")
end
