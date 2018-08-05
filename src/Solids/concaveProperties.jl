# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Solids (Modia3D/Solids/_module.jl)
#


#=
All of these properties are used for function VHACD for decomposing a 3D surface
into a set of near convex solids.

outputDirectory ... Output directory
outputBasename ... Base name for output file (a number and file extension will be added automatically)
showLog ... show log file upon completion
resolution ... Maximum number of voxels generated during the voxelization stage (default=100,000, range=10,000-16,000,000)
depth ... Maximum number of clipping stages. During each split stage, solids with a concavity higher than the user defined threshold are clipped according the 'best' clipping plane (default=20, range=1-32)
concavity ... Maximum allowed concavity (default=0.0025, range=0.0-1.0)
planeDownsampling ... Controls the granularity of the search for the 'best' clipping plane (default=4, range=1-16)
convexhullDownsampling ... Controls the precision of the convex-hull generation process during the clipping plane selection stage (default=4, range=1-16)
alpha ... Controls the bias toward clipping along symmetry planes (default=0.05, range=0.0-1.0)
beta ... Controls the bias toward clipping along revolution axes (default=0.05, range=0.0-1.0)
gamma ... Controls the maximum allowed concavity during the merge stage (default=0.00125, range=0.0-1.0)
delta ... Controls the bias toward maximaxing local concavity (default=0.05, range=0.0-1.0)
pca ... Enable/disable normalizing the mesh before applying the convex decomposition (default=false)
mode ... false: voxel-based approximate convex decomposition, true: tetrahedron-based approximate convex decomposition
maxNumVerticesPerCH ... Controls the maximum number of triangles per convex-hull (default=64, range=4-1024)
minVolumePerCH ... Controls the adaptive sampling of the generated convex-hulls (default=0.0001, range=0.0-0.01)
convexhullApproximation ... Enable/disable approximation when computing convex-hulls
=#

struct ConcaveProperties
   outputDirectory::String
   outputBasename::String
   showLog::Bool
   resolution::Int
   depth::Int
   concavity::Float64
   planeDownsampling::Int
   convexhullDownsampling::Int
   alpha::Float64
   beta::Float64
   gamma::Float64
   delta::Float64
   pca::Bool
   mode::Bool
   convexhullApproximation::Bool
   maxNumVerticesPerCH::Int
   minVolumePerCH::Float64

   function ConcaveProperties(filename::String; showLog=false, resolution=100000, depth=20, concavity=0.0025, planeDownsampling=4, convexhullDownsampling=4,
            alpha=0.05, beta=0.05, gamma=0.00125, delta=0.05, pca=false, mode=false, convexhullApproximation=true,
            maxNumVerticesPerCH=64, minVolumePerCH=0.0001)
     @assert(isfile(filename))
     @assert(filename[end-3:end] == ".obj")
     @assert(isinteger(resolution))
     @assert(resolution>=10000 && resolution<=16000000)
     @assert(isinteger(depth))
     @assert(depth>=1 && depth<=32)
     @assert(concavity>=0.0 && concavity<=1.0)
     @assert(isinteger(planeDownsampling))
     @assert(planeDownsampling>=1 && planeDownsampling<=16)
     @assert(isinteger(convexhullDownsampling))
     @assert(convexhullDownsampling>=1 && convexhullDownsampling<=16)
     @assert(alpha>=0.0 && alpha<=1.0)
     @assert(beta>=0.0 && beta<=1.0)
     @assert(gamma>=0.0 && gamma<=1.0)
     @assert(delta>=0.0 && delta<=1.0)
     @assert(isinteger(maxNumVerticesPerCH))
     @assert(maxNumVerticesPerCH>=4 && maxNumVerticesPerCH<=1024)
     @assert(minVolumePerCH>=0.0 && minVolumePerCH<=0.01)

     outputDirectory = joinpath(dirname(filename),"convexSolids_" * basename(filename))
     outputBasename = rsplit(basename(filename), ".")[1]

     new(outputDirectory, outputBasename, showLog, resolution, depth, concavity, planeDownsampling, convexhullDownsampling,
        alpha, beta, gamma, delta, pca, mode, convexhullApproximation, maxNumVerticesPerCH, minVolumePerCH)
   end
end
