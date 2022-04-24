# License for this file: MIT (expat)
# Copyright 2022, DLR Institute of System Dynamics and Control
# Author: Martin Otter (DLR)
#
# Execute this script via
#
#   include("create_Modia3D_sysimage.jl")
#
# to generate a sysimage of your current project and store it in your current working directory.
# Before generating the sysimage, the following packages are added to your current project (if not yet included):
#
#    Modia, Modia3D, ModiaPlot_PyPlot, PackageCompiler, Revise
#
module Create_Modia3D_sysimage

path = dirname(@__FILE__)
file = joinpath(path, "create_Modia3D_sysimage.jl")
Modia3D_sysimage_path = joinpath(pwd(), Sys.iswindows() ? "Modia3D_sysimage.dll" : "Modia3D_sysimage.so")

import Pkg
project     = Pkg.project()
projectPath = project.path
availablePackages = keys(project.dependencies)
addPackages       = setdiff!(["Modia", "Modia3D", "ModiaPlot_PyPlot", "PackageCompiler", "Revise"], availablePackages)
println("!!! Creating sysimage for Modia3D (executing: $file)")
println("!!! This will include all packages from project $projectPath")
if length(addPackages) > 0
    println("!!! Additionally, it will include the following packages")
    println("!!! $addPackages")
    println("!!! that are now added to the project ...")
    Pkg.add(addPackages)
end

# Create sysimage
using PackageCompiler
create_sysimage(sysimage_path = Modia3D_sysimage_path,
                precompile_statements_file = joinpath(path, "create_Modia3D_sysimage_precompile_statements_file.jl"))
                #precompile_execution_file  = joinpath(path, "create_Modia3D_sysimage_precompile_execution_file.jl"))

println("!!! Modia3D sysimage created. Use sysimage by starting julia with:")
println("  julia -J$Modia3D_sysimage_path")

end