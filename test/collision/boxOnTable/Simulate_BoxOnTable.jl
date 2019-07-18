module Simulate_BoxOnTable

using Modia3D
import Modia3D.ModiaMath

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of SolidFileMesh
vmat2 = Modia3D.Material(color="Green" , transparency=0.5)

#=
workpiece_Lx = 0.04
workpiece_Ly = 0.04
workpiece_Lz = 0.08
=#

zDim = 0.02
boxDim = 0.04
Lx = 0.04
Ly = 0.04
Lz = 0.08

rsmall = 0.001   # 0.03    # 0.03
@assembly BoxOnTable begin
  world = Object3D(visualizeFrame=true)
  box   = Object3D(world, Solid(SolidBox(Lx,Ly,Lz,rsmall=rsmall), "DryWood", vmat1, contactMaterial = "DryWood"),
                   r=[0.1, 0.1, Lz/2], fixed=false)
  table = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(0.6,0.6,zDim) , "DryWood", vmat2; contactMaterial = "DryWood"); r=[0.3, 0.3, -zDim/2], fixed=true )
end


gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
boxOnTable = BoxOnTable(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.2,nz_max = 100,
                        enableContactDetection=true, elasticContactReductionFactor=1e-4,
                        visualizeContactPoints=false, visualizeSupportPoints=false))

# Modia3D.visualizeAssembly!( threeD )


model = Modia3D.SimulationModel( boxOnTable, maxNumberOfSteps=2000 )
#ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=2.0, tolerance=1e-5, interval=0.001, log=true)

ModiaMath.plot(result, ["box.r[3]","box.v[3]"])

println("... success of Simulate_ContactBoxOnTable.jl!")

end
