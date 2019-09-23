module Simulate_YouBot

using  Modia3D
import Modia3D.ModiaMath
using  Modia3D.ModiaMath.Unitful

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.0)    # material of SolidFileMesh
vmat2 = Modia3D.Material(color="Grey"      , transparency=0.3)    # material of work piece
vmat3 = Modia3D.Material(color="LightBlue" , transparency=0.3)    # material of table

workpiece_Lx = 0.04
workpiece_Ly = 0.04
workpiece_Lz = 0.08

xPosTable = 0.71
tableX    = 0.4
tableY    = 0.3
tableZ    = 0.01
heigthLeg = 0.35
widthLeg  = 0.02
@assembly Table(world) begin
    solidPlate = Solid(SolidBox(tableX, tableY, tableZ, rsmall=0.0) , "DryWood", vmat1, contactMaterial="DryWood")
    plate = Object3D(world, solidPlate, fixed=true, r=[xPosTable , 0.0, heigthLeg], visualizeFrame=false)
    leg   = Solid(SolidBox(widthLeg, widthLeg, heigthLeg) , "DryWood", vmat3)
    leg1  = Object3D(plate, leg, fixed=true, r=[tableX/2 - widthLeg/2 , tableY/2 - widthLeg/2, -heigthLeg/2])
    leg2  = Object3D(plate, leg, fixed=true, r=[-tableX/2 + widthLeg/2 , tableY/2 - widthLeg/2, -heigthLeg/2])
    leg3  = Object3D(plate, leg, fixed=true, r=[tableX/2 - widthLeg/2 , -tableY/2 + widthLeg/2, -heigthLeg/2])
    leg4  = Object3D(plate, leg, fixed=true, r=[-tableX/2 + widthLeg/2 , -tableY/2 + widthLeg/2, -heigthLeg/2])
end


@assembly YouBot begin
    world           = Object3D(visualizeFrame=true)

    # Table + workpiece
    table     = Table(world)
    workpiece = Modia3D.ContactBox(table.plate, scale=[workpiece_Lx, workpiece_Ly, workpiece_Lz], massProperties="DryWood",
                                   material=vmat2, contactMaterial="DryWood",
                                   r=[-tableX/2+1.2*workpiece_Lx/2, 0.0, workpiece_Lz/2+tableZ/2], visualizeFrame=false, fixed=false)
end

gravField = UniformGravityField(g=9.81, n=[0,0,-1])
youBot    = YouBot(sceneOptions=SceneOptions(gravityField=gravField, visualizeFrames=false,
                                             defaultFrameLength=0.1, enableContactDetection=true, elasticContactReductionFactor=1e-4))

#Modia3D.visualizeAssembly!(youBot)
model  = Modia3D.SimulationModel( youBot )
result = ModiaMath.simulate!(model, stopTime=1.0, log=false, interval=0.001, tolerance=1e-5)

ModiaMath.plot(result, "workpiece.box.r[3]", figure=1)

println("... success of Simulate_YouBotBoxOnTable.jl!")

end
