module Test_Signal1Assembly

using Modia3D
using StaticArrays
import ModiaMath


vgreen  = Modia3D.Material(color="Green", transparency=0.5)
vyellow = Modia3D.Material(color="Yellow", transparency=0.5)
vred    = Modia3D.Material(color="Red", transparency=0.5)
cmat    = Modia3D.defaultContactMaterial()

Dx = 0.2
Lx = 1.0
Ly = 0.3
Lz = 0.3

# a few signal macros
@signal SinWave(;A=5.0, B=2.0) begin
   y1 = ModiaMath.RealScalar("y", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::SinWave, sim::ModiaMath.SimulationState)
    signal.y1.value = signal.A*sin(signal.B*sim.time)
end

@signal CosWave(;C=5.0) begin
   y = ModiaMath.RealScalar("y", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::CosWave, sim::ModiaMath.SimulationState)
    signal.y.value = signal.C*cos(sim.time)
end

@signal LinearSig(;D=5.0, E=2.0) begin
   y = ModiaMath.RealScalar("y", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::LinearSig, sim::ModiaMath.SimulationState)
    signal.y.value = signal.D*sim.time + signal.E
end


@assembly Pendulum() begin
world    = Modia3D.Object3D()
world_f1 = Modia3D.Object3D(world , r=[0.5*Lx, 0.0, Lz/2])

green = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vgreen),[ [-Lx/2,0,0],[ Lx/2,0,0] ])
rev1  = Modia3D.Revolute(world, green.frames[1])
green2 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vgreen),[ [-Lx/2,0,0],[ Lx/2,0,0] ])
rev12  = Modia3D.Revolute(green.frames[2], green2.frames[1])
gelb = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vyellow),[ [-Lx/2,0,0],[ Lx/2,0,0] ])
rev2 = Modia3D.Revolute(world_f1, gelb.frames[1])

sinWave     = SinWave()
cosWave     = CosWave()
linearSig   = LinearSig()
sigGreen    = Modia3D.SignalToFlangeAngle(sinWave.y1)
sigGreen2   = Modia3D.SignalToFlangeAngle(cosWave.y)
sigGelb     = Modia3D.SignalToFlangeAngle(cosWave.y)

Modia3D.connect(sigGreen,  rev1)
Modia3D.connect(sigGreen2, rev12)
Modia3D.connect(sigGelb,   rev2)
end
pendulum = Pendulum(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true,defaultFrameLength=0.3, enableContactDetection=false))

#=
Modia3D.connect(pendulum.sigGreen,  pendulum.rev1)
Modia3D.connect(pendulum.sigGreen2, pendulum.rev12)
Modia3D.connect(pendulum.sigGelb,   pendulum.rev2)
=#

model    = Modia3D.SimulationModel(pendulum, analysis=ModiaMath.KinematicAnalysis)
# ModiaMath.print_ModelVariables(model)
result   = ModiaMath.simulate!(model, stopTime=5.0)

ModiaMath.plot(result, ["sinWave.y1", "rev1.phi", "cosWave.y", "rev12.phi", "linearSig.y", "rev2.phi"] )

println("... success of Test_Signal1Assembly.jl!")
end
