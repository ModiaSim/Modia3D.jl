module Test_Collision

using Modia3D
using Modia3D.StaticArrays
using Modia3D.ModiaMath
using Modia3D.Unitful


# Materials
massProperties = "Aluminium"
vmat1   = Modia3D.Material(color="LightBlue", transparency=0.5, wireframe=false)
vblue   = Modia3D.Material(color="LightBlue" , transparency=0.5)
vred    = Modia3D.Material(color="Red", transparency=0.5)
vgreen  = Modia3D.Material(color="Green", transparency=0.5)
vblack  = Modia3D.Material(color="Black", transparency=0.5)
vyellow = Modia3D.Material(color="Yellow", transparency=0.5)
vpink   = Modia3D.Material(color="Pink", transparency=0.5)
cmat    = Modia3D.defaultContactMaterial()
vMatAABB = Modia3D.Material(color="Grey", transparency=0.95)

# dimensions
Lx  = 0.7
Dx = 0.5
Ly    = 0.2
Lz   = 1.0

groundWidth  = 1.0
groundHeight = 0.01
cyl  = Modia3D.Cylinder(1.2*Ly, Ly/2)
Tcyl = ModiaMath.rot1(90u"°")
TGround = ModiaMath.rot1(-90u"°")



@assembly CollisionSolids(world) begin
world_f1 = Modia3D.Object3D(world , r=[0.5*Lx, 0.0, groundWidth/2])

blau1     = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vblue;contactMaterial = cmat),    [ [0.0, 0.0, -Lz/2],[ 0.0, 0.0, Lz/2] ] )
blau2     = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vblue; contactMaterial = cmat),   [ [0.0, 0.0, -Lz/2], [ 0.0, 0.0, Lz/2] ] )

schwarz1  = Modia3D.Part(Modia3D.Solid(Modia3D.SolidCone(Dx,Lz; relativeTipDiameter=0.0), "Aluminium", vblack; contactMaterial = cmat),  [ [0.0, 0.0, 0.0]] )
rosa1     = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vpink; contactMaterial = cmat),   [ [0.0, 0.0, -Lz/2],[ 0.0, 0.0, Lz/2] ] )
green1    = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vgreen; contactMaterial = cmat),     [ [0.0, 0.0, -Lz/2],[ 0.0, 0.0, Lz/2] ] )
green2    = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vgreen; contactMaterial = cmat),     [ [0.0, 0.0, -Lz/2]] )
gelb1     = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vyellow; contactMaterial = cmat), [ [0.0, 0.0, -Lz/2],[ 0.0, 0.0, Lz/2] ] )
gelb2     = Modia3D.Part(Modia3D.Solid(Modia3D.SolidEllipsoid(Lx,Ly,Lz), "Aluminium", vyellow; contactMaterial = cmat), [ [0.0, 0.0, -Lz/2] ] )
gelb3     = Modia3D.Part(Modia3D.Solid(Modia3D.SolidSphere(Dx), "Aluminium", vyellow; contactMaterial = cmat),    [ [0.0, 0.0, -Lz/2] ] )
rotM1      = Modia3D.Part(Modia3D.Solid(Modia3D.SolidPipe(Dx,Lz), nothing, vred; contactMaterial = cmat),          [[0.0, 0.0, -Lz/2] , [ 0.0, 0.0,Lz/2]] )
rotM2      = Modia3D.Part(Modia3D.Solid(Modia3D.SolidCylinder(Dx, Lz), nothing, vred; contactMaterial = cmat),     [[0.0, 0.0, -Lz/2], [ 0.0, 0.0,Lz/2]])
neuBlau1     = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vblue;contactMaterial = cmat),    [ [0.0, 0.0, -Lz/2],[ 0.0, 0.0, Lz/2] ] )
neuBlau2     = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vblue; contactMaterial = cmat),   [ [0.0, 0.0, -Lz/2], [ 0.0, 0.0, Lz/2] ] )

visuElem     = Modia3D.Part(cyl,   [ [0.0, 0.0, -Lz/2], [ 0.0, 0.0, Lz/2] ] )


Modia3D.connect(world_f1, blau1.frames[1]; R=ModiaMath.rot2(160u"°"))
Modia3D.connect(world_f1, blau2.frames[1]; R=ModiaMath.rot2(45u"°"))


rev2 = Modia3D.Revolute(blau2.frames[2], rosa1.frames[1]; phi_start =-pi/2)
rev5 = Modia3D.Revolute(blau1.frames[2], schwarz1.frames[1]; phi_start =-pi/2)
rev3 = Modia3D.Revolute(rosa1.frames[2], green1.frames[1])


Modia3D.connect(neuBlau1.frames[2],neuBlau2.frames[1]; R=ModiaMath.rot1(130u"°"))
rev6 = Modia3D.Revolute(rosa1.frame0, neuBlau1.frames[1])


Modia3D.connect(green1.frame0, green2.frames[1]; R=ModiaMath.rot2(45u"°"))
rev4 = Modia3D.Revolute(green1.frames[2], gelb1.frames[1])

Modia3D.connect(gelb1.frames[2],gelb2.frames[1]; R=ModiaMath.rot2(130u"°"))
Modia3D.connect(gelb1.frames[2], gelb3.frames[1]; R=ModiaMath.rot2(90u"°"))

#Modia3D.connect(world, rotM1.frames[1]; fixed=false, R=ModiaMath.rot3(90u"°"))
Modia3D.connect(blau2.frame0, rotM1.frames[1]; fixed=false, R=ModiaMath.rot2(90u"°"))
Modia3D.connect(rotM1.frames[2], rotM2.frames[1]) #; R=ModiaMath.rot2(90u"°"))

Modia3D.connect(gelb1.frames[2], visuElem.frames[2])
end


@assembly CollisionTest() begin
  world = Modia3D.Object3D(Modia3D.CoordinateSystem(1.0))

  collisionSolids = CollisionSolids(world)
end

collisionTest = CollisionTest(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3,nz_max = 80))

Modia3D.initAnalysis!(collisionTest)



tStart=0.0
tEnd  =1.0

@static if VERSION >= v"0.7.0-DEV.2005"
    LINSPACE(start,stop,length) = range(0.0, stop=stop, length=length)
else
    LINSPACE(start,stop,length) = linspace(start,stop,length)
end

for time = LINSPACE(tStart, tEnd, 101)
  s = Modia3D.linearMovement(2*Lx, tStart, tEnd, time)
  delta_phi = Modia3D.linearMovement(pi/3, tStart, tEnd, time)

  Modia3D.setAngle!(collisionTest.collisionSolids.rev2,-pi/2 + delta_phi)
  Modia3D.setAngle!(collisionTest.collisionSolids.rev3, delta_phi)
  Modia3D.setAngle!(collisionTest.collisionSolids.rev4,-delta_phi)
  Modia3D.setAngle!(collisionTest.collisionSolids.rev5, delta_phi)
#  Modia3D.setAngle!(collisionTest.collisionSolids.rev6, -2*delta_phi)

  Modia3D.set_r!(collisionTest.collisionSolids.rotM1.frames[1], [0, 0.0, -s])

  Modia3D.updatePosition!(collisionTest)
#  Modia3D.selectContactPairs!(collisionTest)


#  Modia3D.setComputationFlag(collisionTest)
  Modia3D.visualize!(collisionTest,time)

#=
  time += 0.01
  s = Modia3D.linearMovement(2*Lx, tStart, tEnd, time)
  Modia3D.set_r!(rotM1, [0, 0, s])
  Modia3D.updatePosition!(world)
  Modia3D.selectContactPairs!(world)
  Modia3D.setComputationFlag(world)
  Modia3D.visualize!(world,time)
=#
  time += 0.01
end

Modia3D.closeAnalysis!(collisionTest)
println("... success of Test_Collision.jl!")
end
