module Test_Solids

using Modia3D
using Modia3D.StaticArrays
using Modia3D.Unitful
using Modia3D.ModiaMath


massProperties = "Aluminium"

groundMaterial = Modia3D.Material(color="DarkGreen", transparency=0.5)
vblue   = Modia3D.Material(color="LightBlue" , transparency=0.5)
vred    = Modia3D.Material(color="Red", transparency=0.5)
vgreen  = Modia3D.Material(color="Green", transparency=0.5)
vblack  = Modia3D.Material(color="Black", transparency=0.5)
vyellow = Modia3D.Material(color="Yellow", transparency=0.5)
vpink   = Modia3D.Material(color="Pink", transparency=0.5)
vMatAABB = Modia3D.Material(color="Grey", transparency=0.95)

cmat = Modia3D.defaultContactMaterial()

# Base dimensions
length1  = 2.0
length = 0.7
width    = 0.2
height   = 0.2
diameter1 = 0.1
diameter = 0.5
groundWidth  = 1.0
groundHeight = 0.01
cyl  = Modia3D.Cylinder(1.2*width, width/2)
Tcyl = ModiaMath.rot1(90u"°")
TGround = ModiaMath.rot1(-90u"°")

filenameMesh = joinpath(Modia3D.path, "objects", "bunny", "bunny.obj")


@assembly collisionSolids() begin
   world = Modia3D.Object3D(Modia3D.CoordinateSystem(1.0))

   #DxCone = 2.0
   #DyCone = 0.7
   #LzCone = 0.5
   #rotM1 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidCone(DxCone,LzCone; Dy = DyCone, relativeTipDiameter=0.3,relativeInnerDiameter=0.6), nothing, vred; contactMaterial = cmat),  [ [0.0, 0.0, LzCone/2] ])


   DxCone = 0.7
   DyCone = 0.5
   LzCone = 2.2
   rotM1 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidCone(DxCone,LzCone; Dy = DyCone, relativeTipDiameter=0.0,relativeInnerDiameter=0.6), nothing, vred; contactMaterial = cmat),  [ [0.0, 0.0, LzCone/2] ])

   Modia3D.connect(world, rotM1.frame0; r=[0.0, 0.0, 0.0], R=ModiaMath.rot3(45u"°"), fixed=false)

   mesh = Modia3D.Object3D(world, Modia3D.SolidWithConvexDecomposition(Modia3D.SolidFileMesh(filenameMesh,0.1), nothing, vyellow, vyellow; contactMaterial = cmat); r=[-2.0, 0.0, 0.0], fixed=false)


   #
   lPipe = 2.0
   dPipe = 0.5
   wPipe = 0.7
   rotM2 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidPipe(dPipe,lPipe; Dy=wPipe, relativeInnerDiameter=0.3), nothing, vred; contactMaterial = cmat), [ [0.0, 0.0, lPipe/2] ] )
   Modia3D.connect(world, rotM2.frame0; r=[2.0, 0.0, 0.0], q=ModiaMath.qrot3(45u"°"), fixed=false)

   lCapsule = 1.0
   dCapsule = 0.3
   wCapsule = 0.8
   green = Modia3D.Part(Modia3D.Solid(Modia3D.SolidCapsule(dCapsule,lCapsule; Dy=wCapsule), nothing, vgreen; contactMaterial = cmat), [ [dCapsule/2, 0.0,lCapsule/2],[0.0, wCapsule/2, -lCapsule/2],[0.0,0.0, lCapsule/2+wCapsule/2],[0.0,0.0, -lCapsule/2-wCapsule/2]  ] )
   Modia3D.connect(world, green.frame0; r=[4.0, 0.0, 0.0], fixed=false)

   green1 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidSphere(diameter), nothing, vgreen; contactMaterial = cmat ), [ [0.0, 0.0, 0.0] ])
   Modia3D.connect(world, green1.frame0; r=[5.0, 0.0, 0.0],  fixed=false)

   lxBeam = 2.7
   dyBeam = 0.7
   lzBeam = 0.5
   green2 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBeam(lxBeam,dyBeam,lzBeam), nothing, vgreen; contactMaterial = cmat), [ [-lxBeam/2-dyBeam/2, 0.0, 0.0],[ lxBeam/2 + dyBeam/2, 0.0, 0.0],[lxBeam/2, dyBeam/2, 0.0] ] )
   Modia3D.connect(world, green2.frame0; r=[0.0, 5.0, 0.0], fixed=false)

   hBox = 0.7
   wBox = 0.5
   lBox = 2.0
   gelb1 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(hBox,wBox,lBox), nothing, vyellow; contactMaterial = cmat), [ [0.0, 0.0, -lBox/2],[ 0.0, 0.0, lBox/2] ] )
   Modia3D.connect(world, gelb1.frame0; r=[6.0, 0.0, 0.0], fixed=false)


   #Bounding Boxes
   rotM1AABB    = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
   rotM2AABB    = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
   greenAABB   = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
   green2AABB  = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
   cameAABB    = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
   cameAABB1   = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
   cameAABB2   = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
   cameAABB3   = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
   cameAABB4   = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
   cameAABB5   = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))


   world_min = Modia3D.Object3D(world , r=[-0.50151, 0.0, 0.0])
   world_max = Modia3D.Object3D(world , r=[0.50151, 0.0, 0.0])

   world_cylinder_min = Modia3D.Object3D(world , r=[-0.50151, 0.0, 0.0])
   world_cylinder_max = Modia3D.Object3D(world , r=[0.50151, 0.0, 0.0])
end

as = collisionSolids(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3, nz_max = 8))

# Kinematic simulation
Modia3D.initAnalysis!(as)

AABB = as._internal.scene.options.contactDetection.contactPairs.AABB

tStart=0.0
tEnd  =1.0
#tEnd = 0.01

for time = range(tStart, stop=tEnd, length=101)  # 0:0.01:0.01
  s = Modia3D.linearMovement(2*length1, tStart, tEnd, time)
  sdeg = 10*s*u"°"
  Modia3D.set_q!(as.rotM1.frame0, ModiaMath.qrot1(sdeg))
  Modia3D.set_r!(as.rotM1.frame0, [0, 0, s])

  Modia3D.set_q!(as.rotM2.frame0, ModiaMath.qrot1(sdeg))
  Modia3D.set_r!(as.rotM2.frame0, [2.0, 0, 0])
  Modia3D.set_q!(as.green2.frame0, ModiaMath.qrot3(sdeg))
  Modia3D.set_r!(as.green2.frame0, [0, 5.0, s])
  Modia3D.set_q!(as.green.frame0, ModiaMath.qrot1(sdeg))
  Modia3D.set_r!(as.green.frame0, [4.0, 0.0, s])


  Modia3D.updatePosition!(as)
  Modia3D.selectContactPairs!(as)

  as.rotM1AABB.data.Lx = abs(AABB[2][1].x_max - AABB[2][1].x_min)
  as.rotM1AABB.data.Ly = abs(AABB[2][1].y_max - AABB[2][1].y_min)
  as.rotM1AABB.data.Lz = abs(AABB[2][1].z_max - AABB[2][1].z_min)
  as.rotM1AABB.r_abs = [(AABB[2][1].x_max+AABB[2][1].x_min)/2,(AABB[2][1].y_max+AABB[2][1].y_min)/2,(AABB[2][1].z_max+AABB[2][1].z_min)/2 ]

  as.world_min.r_abs = [AABB[2][1].x_min, AABB[2][1].y_min, AABB[2][1].z_min]
  as.world_max.r_abs = [AABB[2][1].x_max, AABB[2][1].y_max, AABB[2][1].z_max]

  as.cameAABB.data.Lx = abs(AABB[3][1].x_max - AABB[3][1].x_min)
  as.cameAABB.data.Ly = abs(AABB[3][1].y_max - AABB[3][1].y_min)
  as.cameAABB.data.Lz = abs(AABB[3][1].z_max - AABB[3][1].z_min)
  as.cameAABB.r_abs = [(AABB[3][1].x_max+AABB[3][1].x_min)/2,(AABB[3][1].y_max+AABB[3][1].y_min)/2,(AABB[3][1].z_max+AABB[3][1].z_min)/2 ]

  as.cameAABB1.data.Lx = abs(AABB[3][2].x_max - AABB[3][2].x_min)
  as.cameAABB1.data.Ly = abs(AABB[3][2].y_max - AABB[3][2].y_min)
  as.cameAABB1.data.Lz = abs(AABB[3][2].z_max - AABB[3][2].z_min)
  as.cameAABB1.r_abs = [(AABB[3][2].x_max+AABB[3][2].x_min)/2,(AABB[3][2].y_max+AABB[3][2].y_min)/2,(AABB[3][2].z_max+AABB[3][2].z_min)/2 ]

  as.cameAABB2.data.Lx = abs(AABB[3][3].x_max - AABB[3][3].x_min)
  as.cameAABB2.data.Ly = abs(AABB[3][3].y_max - AABB[3][3].y_min)
  as.cameAABB2.data.Lz = abs(AABB[3][3].z_max - AABB[3][3].z_min)
  as.cameAABB2.r_abs = [(AABB[3][3].x_max+AABB[3][3].x_min)/2,(AABB[3][3].y_max+AABB[3][3].y_min)/2,(AABB[3][3].z_max+AABB[3][3].z_min)/2 ]

  as.cameAABB3.data.Lx = abs(AABB[3][4].x_max - AABB[3][4].x_min)
  as.cameAABB3.data.Ly = abs(AABB[3][4].y_max - AABB[3][4].y_min)
  as.cameAABB3.data.Lz = abs(AABB[3][4].z_max - AABB[3][4].z_min)
  as.cameAABB3.r_abs = [(AABB[3][4].x_max+AABB[3][4].x_min)/2,(AABB[3][4].y_max+AABB[3][4].y_min)/2,(AABB[3][4].z_max+AABB[3][4].z_min)/2 ]

  as.cameAABB4.data.Lx = abs(AABB[3][5].x_max - AABB[3][5].x_min)
  as.cameAABB4.data.Ly = abs(AABB[3][5].y_max - AABB[3][5].y_min)
  as.cameAABB4.data.Lz = abs(AABB[3][5].z_max - AABB[3][5].z_min)
  as.cameAABB4.r_abs = [(AABB[3][5].x_max+AABB[3][5].x_min)/2,(AABB[3][5].y_max+AABB[3][5].y_min)/2,(AABB[3][5].z_max+AABB[3][5].z_min)/2 ]

  as.cameAABB5.data.Lx = abs(AABB[3][6].x_max - AABB[3][6].x_min)
  as.cameAABB5.data.Ly = abs(AABB[3][6].y_max - AABB[3][6].y_min)
  as.cameAABB5.data.Lz = abs(AABB[3][6].z_max - AABB[3][6].z_min)
  as.cameAABB5.r_abs = [(AABB[3][6].x_max+AABB[3][6].x_min)/2,(AABB[3][6].y_max+AABB[3][6].y_min)/2,(AABB[3][6].z_max+AABB[3][6].z_min)/2 ]

  as.rotM2AABB.data.Lx = abs(AABB[4][1].x_max - AABB[4][1].x_min)
  as.rotM2AABB.data.Ly = abs(AABB[4][1].y_max - AABB[4][1].y_min)
  as.rotM2AABB.data.Lz = abs(AABB[4][1].z_max - AABB[4][1].z_min)
  as.rotM2AABB.r_abs = as.rotM2.frame0.r_abs

  as.world_cylinder_min.r_abs = [AABB[4][1].x_min, AABB[4][1].y_min, AABB[4][1].z_min]
  as.world_cylinder_max.r_abs = [AABB[4][1].x_max, AABB[4][1].y_max, AABB[3][1].z_max]

  as.greenAABB.data.Lx = abs(AABB[5][1].x_max - AABB[5][1].x_min)
  as.greenAABB.data.Ly = abs(AABB[5][1].y_max - AABB[5][1].y_min)
  as.greenAABB.data.Lz = abs(AABB[5][1].z_max - AABB[5][1].z_min)
  as.greenAABB.r_abs = as.green.frame0.r_abs

  as.green2AABB.data.Lx = abs(AABB[7][1].x_max - AABB[7][1].x_min)
  as.green2AABB.data.Ly = abs(AABB[7][1].y_max - AABB[7][1].y_min)
  as.green2AABB.data.Lz = abs(AABB[7][1].z_max - AABB[7][1].z_min)
  as.green2AABB.r_abs = as.green2.frame0.r_abs


  Modia3D.setComputationFlag(as)
  Modia3D.visualize!(as,time)
  time += 0.01
end

Modia3D.closeAnalysis!(as)
println("... success of Test_Solids.jl!")
end
