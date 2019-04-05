module Move_AllVisualObjects

using Modia3D



filename = joinpath(Modia3D.path, "objects", "fish", "SiameseTiger0.3ds")
#filename = joinpath(Modia3D.path, "objects","engine", "crank", "crank.obj")
#file = FileShape.convexFile(filename; scaleFactor=MVector{3,Float64}(4.0,4.0,4.0))


# Material for Visualization
mat1 = Modia3D.Material(color="Blue", transparency=0.5)
mat2 = Modia3D.Material(transparency=0.2)

# Shapes
coordsys = Modia3D.CoordinateSystem(0.7)
r1 = [4.5, 0.0, 0.0]
dr = [-1.5,0.0,0.0]


@assembly VisualObjects begin
   world   = Modia3D.Object3D()
   shape1  = Modia3D.Object3D(world  , Modia3D.Box(0.9,0.5,0.3    , material=mat1); r=r1, fixed=false)
   shape2  = Modia3D.Object3D(shape1 , Modia3D.Sphere(0.7         , material=mat1); r=dr, fixed=false)
   shape3  = Modia3D.Object3D(shape2 , Modia3D.Cylinder(0.5,0.8   , material=mat1); r=dr, fixed=false)
   shape4  = Modia3D.Object3D(shape3 , Modia3D.Cone(0.3,0.7       , material=mat1); r=dr, fixed=false)
   shape5  = Modia3D.Object3D(shape4 , Modia3D.Capsule(0.4,0.45   , material=mat1); r=dr, fixed=false)
   shape6  = Modia3D.Object3D(shape5 , Modia3D.Spring(0.3,0.7     , material=mat1); r=dr, fixed=false)
   shape7  = Modia3D.Object3D(shape1 , Modia3D.GearWheel(0.5,0.8  , material=mat1); r=[0.0,0.0,-2.0], fixed=false)
   shape8  = Modia3D.Object3D(shape7 , Modia3D.Pipe(0.5,0.8       , material=mat1); r=dr, fixed=false)
   shape9  = Modia3D.Object3D(shape8 , Modia3D.Beam(0.4,0.5,0.3   , material=mat1); r=dr, fixed=false)
   shape10 = Modia3D.Object3D(shape9 , Modia3D.Grid(1.0,0.6)                      ; r=dr, fixed=false)
   shape11 = Modia3D.Object3D(shape10, coordsys                                   ; r=dr, fixed=false)
   shape12 = Modia3D.Object3D(shape11, Modia3D.FileMesh(filename,4, material=mat2); r=dr, fixed=false)
end
visualObjects = VisualObjects(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true,
                                                                defaultFrameLength=0.7,
                                                                enableContactDetection=false))

# Kinematic simulation
Modia3D.initAnalysis!(visualObjects)

for time = range(0.0, stop=2.0, length=101)
  # update positional degrees of freedom
  r1[3] = 2*time
  Modia3D.set_r!(visualObjects.shape1,r1)
  Modia3D.updatePosition!(visualObjects)

  # Visualize shapes
  Modia3D.visualize!(visualObjects,time)
end

Modia3D.closeAnalysis!(visualObjects)

println("... success of Move_AllVisualObjects.jl!")

end
