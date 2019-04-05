module Move_Pendulum

using  Modia3D
import Modia3D.ModiaMath

# Properties
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")
Lx = 1.6
Ly = 0.2*Lx
Lz = 0.2*Lx
m  = 1.0

# Pendulum
world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))
body   = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), Modia3D.MassProperties(m=m), vmat1))
frame1 = Modia3D.Object3D(body; r=[-Lx/2, 0.0, 0.0])
cyl    = Modia3D.Object3D(frame1,Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2))
rev    = Modia3D.Revolute(world, frame1)

# Visualize pendulum
# Modia3D.visualizeWorld!(world, sceneOptions=Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3*Lx))

scene  = Modia3D.Scene(Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3*Lx))
Modia3D.initAnalysis!(world, scene)

tStart = 0.0
tEnd   = 3.0

for time = range(tStart, stop=tEnd, length=101)
  delta_phi = Modia3D.linearMovement(2*pi, tStart, tEnd, time)
  Modia3D.setAngle!(rev, delta_phi)
  Modia3D.updatePosition!(world)
  Modia3D.visualize!(scene,time)
end
Modia3D.closeAnalysis!(scene)

println("... success of Move_Pendulum.jl!")
end
