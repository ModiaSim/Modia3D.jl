module Visualize_GeometriesWithoutMaterial

using Modia3D


filename = joinpath(Modia3D.path, "objects", "fish", "SiameseTiger0.3ds")
#filename = joinpath(Modia3D.path, "objects","engine", "crank", "crank.obj")
#file = FileShape.convexFile(filename; scaleFactor=MVector{3,Float64}(4.0,4.0,4.0))

# Properties
mat1  = Modia3D.Material(color="LightBlue", transparency=0.5)
mat2  = Modia3D.Material(transparency=0.2)
font  = Modia3D.Font(fontFamily="TimesNewRoman", bold=false, color="Black", charSize=0.2)
rtext = [0.0, 0.0, 1.0]


# Objects3D
@assembly GeometriesWithoutMaterial begin
   world            = Modia3D.Object3D(visualizeFrame=false)
   grid             = Modia3D.Object3D(world, Modia3D.Grid(1.0,0.6        ); r=[ 0.0,0.0, 0.0])
   coordinateSystem = Modia3D.Object3D(world, Modia3D.CoordinateSystem(0.7); r=[-1.5,0.0, 0.0])


   font1 = Modia3D.Font(charSize=0.2)
   font2 = Modia3D.Font(fontFamily="TimesNewRoman", bold=false, charSize=0.2, color="DarkBlue", transparency=0.0)
   text2 = Modia3D.TextShape("Text in the XY-plane"; font=font2, offset=[0.2, 0.0, 0.0],
                             axisAlignment=Modia3D.XY_Plane, alignment=Modia3D.Left)
   text3 = Modia3D.TextShape("Text in the XZ-plane"; font=font2, offset=[0.0, 0.0, 0.2],
                             axisAlignment=Modia3D.XZ_Plane, alignment=Modia3D.Left)
   text4 = Modia3D.TextShape("Text in the YZ-plane"; font=font2, offset=[0.0, 0.2, 0.0],
                             axisAlignment=Modia3D.YZ_Plane, alignment=Modia3D.Left)

   textShape2 = Modia3D.Object3D(world, text2; r=[1.2, 0.0, 0.0])
   textShape3 = Modia3D.Object3D(world, text3; r=[1.2, 0.0, 0.0])
   textShape4 = Modia3D.Object3D(world, text4; r=[1.2, 0.0, 0.0])

   # Place text above the shapes
   gridText             = Modia3D.Object3D(grid            , Modia3D.TextShape("Grid"            ; font=font); r=rtext, visualizeFrame=Modia3D.False)
   coordinateSystemText = Modia3D.Object3D(coordinateSystem, Modia3D.TextShape("CoordinateSystem"; font=font); r=rtext, visualizeFrame=Modia3D.False)
   TestUber = Modia3D.Object3D(textShape4, Modia3D.TextShape("Text"; font=font); r=rtext, visualizeFrame=Modia3D.False)


end

Modia3D.visualizeAssembly!( GeometriesWithoutMaterial(sceneOptions = Modia3D.SceneOptions(visualizeFrames=true,
                                                                                          defaultFrameLength=0.7,
                                                                                          enableContactDetection=false) ) )

println("... success of Visualize_GeometriesWithoutMaterial.jl!")

end
