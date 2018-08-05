module Test_Signal4Assemblies

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
@signal SinWave(;A=5.0, w=2.0) begin
   y = ModiaMath.RealScalar("y", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::SinWave, sim::ModiaMath.SimulationState)
    signal.y.value = signal.A*sin(signal.w*sim.time)
end

@signal CosWave(;A=5.0, w=2.0) begin
   y = ModiaMath.RealScalar("y", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::CosWave, sim::ModiaMath.SimulationState)
    signal.y.value = signal.A*cos(sim.time)
end

@signal LinearSig(;A=5.0, B=2.0) begin
   y = ModiaMath.RealScalar("y", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::LinearSig, sim::ModiaMath.SimulationState)
    signal.y.value = signal.A*sim.time + signal.B
end



# 3 sub assemblies
@assembly greenAssembly() begin
  green     = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vgreen),[ [-Lx/2,0,0],[ Lx/2,0,0] ])
  green2    = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vgreen),[ [-Lx/2,0,0],[ Lx/2,0,0] ])
  rev12     = Modia3D.Revolute(green.frames[2], green2.frames[1])

  sinWave     = SinWave()
  cosWave     = CosWave()
  sigGreen    = Modia3D.SignalToFlangeAngle(sinWave.y)
  sigGreen2   = Modia3D.SignalToFlangeAngle(cosWave.y)
  Modia3D.connect(sigGreen2, rev12)
end

@assembly yellowAssembly() begin
  gelb    = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vyellow),[ [-Lx/2,0,0],[ Lx/2,0,0] ])

  linearSig   = LinearSig()
  sigGelb     = Modia3D.SignalToFlangeAngle(linearSig.y)
end


# main assembly
@assembly Pendulum() begin
world    = Modia3D.Object3D()
world_f1 = Modia3D.Object3D(world , r=[0.5*Lx, 0.0, Lz/2])
gruen    = greenAssembly()
gelb     = yellowAssembly()


rev1     = Modia3D.Revolute(world, gruen.green.frames[1])
rev2     = Modia3D.Revolute(world_f1, gelb.gelb.frames[1])

Modia3D.connect(gruen.sigGreen, rev1)
Modia3D.connect(gelb.sigGelb,   rev2)
end

pendulum = Pendulum(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true,defaultFrameLength=0.3, enableContactDetection=false))

model    = Modia3D.SimulationModel(pendulum, analysis=ModiaMath.KinematicAnalysis)
# ModiaMath.print_ModelVariables(model)
result   = ModiaMath.simulate!(model, stopTime=5.0)
ModiaMath.plot(result, ["gruen.sinWave.y", "rev1.phi", "gruen.cosWave.y", "gruen.rev12.phi", "gelb.linearSig.y", "rev2.phi"] )

println("... success of Test_Signal4Assemblies.jl!")
end
