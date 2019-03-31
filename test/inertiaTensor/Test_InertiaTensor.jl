module Test_SignalTorque

using  Modia3D
import Modia3D.ModiaMath


Dx = 0.7
Lx = 1.0
Ly = 0.3
Lz = 0.3
m = 1.0
#=
@signal SignalX(;A=0.1) begin
   y1 = ModiaMath.RealScalar("y", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::SignalX, sim::ModiaMath.SimulationState)
    signal.y1.value  = signal.A*sim.time
end

@signal SignalY(;A=0.1) begin
   y1 = ModiaMath.RealScalar("y", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::SignalY, sim::ModiaMath.SimulationState)
    signal.y1.value  = signal.A*cos(sim.time)
end
=#

@signal SignalZ(;A=0.1) begin
   y1 = ModiaMath.RealScalar("y", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::SignalZ, sim::ModiaMath.SimulationState)
    signal.y1.value  = 0.0 # 3.0*sim.time # signal.A*sin(sim.time)
end



vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")


rCM = zeros(3)
rCM[2] = 0.0
#I2 = fill(100.0,(9))
I = fill(0.0,(3,3))
#I[3] = 50.0
#I[7] = 80.0
#println("I = ",I)
massProp = Modia3D.MassProperties(1.0, rCM, I)

@assembly Pendulum() begin
   world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))

   # Pendulum
   body   = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), massProp, vmat1))
   frame1 = Modia3D.Object3D(body; r=[-Lx/2, 0.0, 0.0])
   cyl    = Modia3D.Object3D(frame1,Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2))

   rev1    = Modia3D.Revolute(world, frame1; axis = 3)

   sig    = SignalZ()
   signal = Modia3D.SignalToFlangeTorque(sig.y1)
   Modia3D.connect(signal, rev1)
end

gravField = Modia3D.UniformGravityField(n=[1,0,0])

pendulum = Pendulum(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true,defaultFrameLength=0.3, enableContactDetection=false))
model    = Modia3D.SimulationModel(pendulum; useOptimizedStructure = true)
result   = ModiaMath.simulate!(model, stopTime=0.1, interval=0.1, log=false)
ModiaMath.plot(result, ["sig.y1", "rev1.phi", "rev1.tau"] )

println("... success of Test_SignalTorque.jl!")
end
