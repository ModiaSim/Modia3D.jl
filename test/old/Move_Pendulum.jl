module Move_Pendulum

import Modia3D

# Properties
vmat1 = Modia3D.VisualMaterial(color="LightBlue", transparency=0.5)
vmat2 = Modia3D.VisualMaterial(color="Red")
Lx = 1.6
Ly = 0.2*Lx
Lz = 0.2*Lx
m  = 1.0

# Pendulum
world = Modia3D.Object3D(feature = Modia3D.Visual(shape =
    Modia3D.CoordinateSystem(length=0.5*Lx)))
body = Modia3D.Object3D(feature = Modia3D.Solid(
    shape = Modia3D.Shapes.Beam(axis=1, length=Lx, width=Ly, thickness=Lz), massProperties=Modia3D.MassProperties(mass=m), visualMaterial=vmat1))
frame1 = Modia3D.Object3D(parent=body, translation=[-Lx/2, 0.0, 0.0])
cyl = Modia3D.Object3D(parent=frame1, feature = Modia3D.Visual(shape =
    Modia3D.Cylinder(axis=3, diameter=Ly/2, length=1.2*Ly), visualMaterial=vmat2))
rev = Modia3D.Revolute(obj1=world, obj2=frame1)

# Visualize pendulum
scene = Modia3D.Scene(visualizeFrames=true, defaultFrameLength=0.3*Lx)
Modia3D.initAnalysis!(world, scene)

tStart = 0.0
tEnd   = 3.0

for time = range(tStart, stop=tEnd, length=101)
    delta_phi = Modia3D.linearMovement(2*pi, tStart, tEnd, time)
    Modia3D.setAngle!(rev, delta_phi)
    Modia3D.updatePosition!(world)
    Modia3D.visualize!(scene, time)
end
Modia3D.closeAnalysis!(scene)

println("... test/old/Move_Pendulum.jl completed.")

end
