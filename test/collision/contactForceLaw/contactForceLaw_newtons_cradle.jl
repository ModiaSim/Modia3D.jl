module collision_newtons_cradle

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmatGraphics = Modia3D.Material(color="LightBlue" , transparency=0.1)    # material of Graphics
vmatSolids = Modia3D.Material(color="Red" , transparency=0.0)         # material of solids


cmat = Modia3D.ElasticContactMaterial2("Steel")

Lx = 0.3
Ly = 4.1
Lz = 0.1



diameter = 1.0
lengthWire = 4.0



@assembly NewtonComparison begin
  world = Modia3D.Object3D(visualizeFrame=false)
  cradle = NewtonsCradle(world)
#  cradle2 = NewtonsCradleSolidWire(world)
end



@assembly NewtonsCradle(world) begin
#   world = Modia3D.Object3D(visualizeFrame=false)
  box    = Modia3D.Object3D(world, Modia3D.Box(Lx, Ly, Lz; material=vmatGraphics)   ; r=[0.0, 0.0, 0.0] )

  frame1 = Modia3D.Object3D(box; r=[0.0, -Ly/2, 0.0])

  frame2 = Modia3D.Object3D(box; r=[0.0, -Ly/4, 0.0])
  frame3 = Modia3D.Object3D(box; r=[0.0, 0.0, 0.0])
  frame4 = Modia3D.Object3D(box; r=[0.0, Ly/4, 0.0])
  frame5 = Modia3D.Object3D(box; r=[0.0, Ly/2, 0.0])


  pendulum1 = Pendulum()
  pendulum2 = Pendulum()
  pendulum3 = Pendulum()
  pendulum4 = Pendulum()
  pendulum5 = Pendulum()

  rev1   = Modia3D.Revolute(frame1, pendulum1.frame1; axis = 1 , phi_start =  -pi/3)
  rev2   = Modia3D.Revolute(frame2, pendulum2.frame1; axis = 1 , phi_start =  -pi/3)
  rev3   = Modia3D.Revolute(frame3, pendulum3.frame1; axis = 1 ) #, phi_start =  -pi/3)
  rev4   = Modia3D.Revolute(frame4, pendulum4.frame1; axis = 1 )#, phi_start =  pi/3 ) #
  rev5   = Modia3D.Revolute(frame5, pendulum5.frame1; axis = 1  , phi_start =  pi/3 ) #
end

@assembly Pendulum() begin
  wire = Modia3D.Object3D( Modia3D.Cylinder(diameter/5, lengthWire; material=vmatGraphics) )

  frame1 = Modia3D.Object3D(wire; r=[0.0, 0.0, lengthWire/2])
  frame2 = Modia3D.Object3D(wire; r=[0.0, 0.0, -lengthWire/2])

  cyl    = Modia3D.Cylinder(0.15,Lx; material=vmatGraphics)
  cylinder   = Modia3D.Object3D(frame1, cyl; R=ModiaMath.rot2(-pi/2), visualizeFrame=false)

  sphere = Modia3D.Object3D(frame2, Modia3D.Solid(Modia3D.SolidSphere(diameter) , "Steel", vmatSolids; contactMaterial = cmat) )
end


@assembly NewtonsCradleSolidWire(world) begin
  # world = Modia3D.Object3D(visualizeFrame=false)
  box    = Modia3D.Object3D(world, Modia3D.Box(Lx, Ly, Lz; material=vmatGraphics); r=[0.0, -8.0, 0.0] )

  frame1 = Modia3D.Object3D(box; r=[0.0, -Ly/2, 0.0])

  frame2 = Modia3D.Object3D(box; r=[0.0, -Ly/4, 0.0])
  frame3 = Modia3D.Object3D(box; r=[0.0, 0.0, 0.0])
  frame4 = Modia3D.Object3D(box; r=[0.0, Ly/4, 0.0])
  frame5 = Modia3D.Object3D(box; r=[0.0, Ly/2, 0.0])


  pendulum1 = Pendulum1()
  pendulum2 = Pendulum1()
  pendulum3 = Pendulum1()
  pendulum4 = Pendulum1()
  pendulum5 = Pendulum1()

  rev1   = Modia3D.Revolute(frame1, pendulum1.frame1; axis = 1 , phi_start =  -pi/3)
  rev2   = Modia3D.Revolute(frame2, pendulum2.frame1; axis = 1 , phi_start =  -pi/3)
  rev3   = Modia3D.Revolute(frame3, pendulum3.frame1; axis = 1 ) #, phi_start =  -pi/3)
  rev4   = Modia3D.Revolute(frame4, pendulum4.frame1; axis = 1 ) #, phi_start =  pi/3 ) #
  rev5   = Modia3D.Revolute(frame5, pendulum5.frame1; axis = 1  , phi_start =  pi/3 ) #
end


@assembly Pendulum1() begin
  wire = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidCylinder(diameter/5, lengthWire), "Steel", vmatSolids) )

  frame1 = Modia3D.Object3D(wire; r=[0.0, 0.0, lengthWire/2])
  frame2 = Modia3D.Object3D(wire; r=[0.0, 0.0, -lengthWire/2])

  cyl    = Modia3D.Cylinder(0.15,Lx; material=vmatGraphics)
  cylinder   = Modia3D.Object3D(frame1, cyl; R=ModiaMath.rot2(-pi/2), visualizeFrame=false)

  sphere = Modia3D.Object3D(frame2, Modia3D.Solid(Modia3D.SolidSphere(diameter) , "Steel", vmatSolids; contactMaterial = cmat) )
end

gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
newton = NewtonComparison(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=false, visualizeSupportPoints=false))

# Modia3D.visualizeAssembly!( newton )


model = Modia3D.SimulationModel( newton )
result = ModiaMath.simulate!(model; stopTime=10.0, tolerance=1e-8,interval=0.001, log=false)


println("... success of collision_newtons_cradle.jl!")
end
