# Very basic example, without signals, only with updatePosition!
# one frame is externaly driven

module Test_BasicExBox1

using Modia3D
vgreen  = Modia3D.Material(color="Green", transparency=0.5)

lx = 1.0
ly = 0.5
lz = 0.3

@assembly Pendulum(;length=1.0, width=0.2) begin
  world = Modia3D.Object3D()
  box = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), "Aluminium", vgreen),[ [-lx/2,0,0],[ lx/2,0,0] ])
  rev1 = Modia3D.Revolute(world, box.frames[1])
end
pendulum = Pendulum()

# Kinematic simulation
Modia3D.initAnalysis!(pendulum, Modia3D.SceneOptions(visualizeFrames=true,defaultFrameLength=0.3,enableContactDetection=false))
for time = linspace(0.0, 2*pi, 101)
  Modia3D.setAngle!(pendulum.rev1, sin(time))
  Modia3D.updatePosition!(pendulum)
  Modia3D.visualize!(pendulum,time)
end
Modia3D.closeAnalysis!(pendulum)

println("... success of Test_BasicExBox1.jl!")
end
