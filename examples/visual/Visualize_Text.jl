module Visualize_Text

using Modia3D




@assembly TextDemo begin
   font1 = Modia3D.Font(charSize=0.2)
   font2 = Modia3D.Font(fontFamily="Arial", bold=true, charSize=0.2, color="Red", transparency=0.0)
   text1 = Modia3D.TextShape("This is a text line parallel to the screen"; font=font1)
   text2 = Modia3D.TextShape("This is text in the XY-plane"; font=font2, offset=[0.2, 0.0, 0.0],
                             axisAlignment=Modia3D.XY_Plane, alignment=Modia3D.Left)
   text3 = Modia3D.TextShape("This is text in the XZ-plane"; font=font2, offset=[0.0, 0.0, 0.2],
                             axisAlignment=Modia3D.XZ_Plane, alignment=Modia3D.Left)
   text4 = Modia3D.TextShape("This is text in the YZ-plane"; font=font2, offset=[0.0, 0.2, 0.0],
                             axisAlignment=Modia3D.YZ_Plane, alignment=Modia3D.Left)

   world      = Modia3D.Object3D(visualizeFrame=false)
   textShape1 = Modia3D.Object3D(world, text1)
   textShape2 = Modia3D.Object3D(world, text2; r=[1.0, 1.0, -1.0])
   textShape3 = Modia3D.Object3D(world, text3; r=[1.0, 1.0, -1.0])
   textShape4 = Modia3D.Object3D(world, text4; r=[1.0, 1.0, -1.0])
end

Modia3D.visualizeAssembly!( TextDemo(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.5) ) )


println("... success of Visualize_Text.jl!")

end
