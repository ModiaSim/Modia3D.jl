# Utility functions for FileMesh.jl
# getObjInfos(): read file and returns: centroid, longestEdge and all objPoints
# Remark: at the moment it only works for .obj files
#
# VHACD(): decomposes a concave file mesh in convex sub parts
#          see: https://github.com/kmammou/v-hacd

function getObjInfos(filename::AbstractString, scaleFactor::MVector{3,Float64})
    objPoints = []
    facesIndizes = []
    areTriangles = true
    centroid = Modia3D.ZeroVector3D
    longestEdge = 0
    if filename[end-3:end] == ".obj"
        open(filename,"r") do file
            i = 0
            x_max::Float64 = 0.0
            x_min::Float64 = 0.0
            y_max::Float64 = 0.0
            y_min::Float64 = 0.0
            z_max::Float64 = 0.0
            z_min::Float64 = 0.0
            sum = Modia3D.ZeroVector3D

            for line in eachline(file; keep=true)
                if isequal(line[1],'v') && isequal(line[2],' ')
                    i += 1
                    tmp = rsplit(rstrip(line),' ')
                    push!(objPoints,[parse(Float64,tmp[end-2])*scaleFactor[1],parse(Float64,tmp[end-1])*scaleFactor[2],parse(Float64,tmp[end])*scaleFactor[3]])
                    if i == 1
                        x_max = objPoints[i][1]
                        x_min = objPoints[i][1]
                        y_max = objPoints[i][2]
                        y_min = objPoints[i][2]
                        z_max = objPoints[i][3]
                        z_min = objPoints[i][3]
                    else
                        (x_min, x_max) = check_MinMax(x_min, x_max, objPoints[i][1])
                        (y_min, y_max) = check_MinMax(y_min, y_max, objPoints[i][2])
                        (z_min, z_max) = check_MinMax(z_min, z_max, objPoints[i][3])
                    end
                    sum += objPoints[i]
                end
                if isequal(line[1],'f') && isequal(line[2],' ')
                    if areTriangles
                        tmp = rsplit(rstrip(line),' ')
                        if length(tmp) == 4
                            push!(facesIndizes,[parse(Int64,rsplit(tmp[2],"/")[1]), parse(Int64,rsplit(tmp[3],"/")[1]), parse(Int64,rsplit(tmp[4],"/")[1])])
                        else
                            areTriangles = false
            end; end; end; end

            if length(objPoints) != 0
                centroid = sum / length(objPoints)
            else
                centroid = Modia3D.ZeroVector3D
            end
            if !areTriangles
                empty!(facesIndizes)
            end
            longestEdge = max((x_max-x_min), (y_max-y_min), (z_max-z_min))
        end
        return (centroid, longestEdge, objPoints, facesIndizes)
    else
        error("Only .obj files are supported for solid FileMesh.")
        return (nothing, nothing, nothing, nothing)
    end
end

getObjInfos(filename::AbstractString, scaleFactor::AbstractVector) =
                            getObjInfos(filename, MVector{3,Float64}(scaleFactor))


# utility function for getObjInfos
function check_MinMax(act_min::Float64, act_max::Float64, value)
    if act_min >= value
        act_min = value
    end
    if act_max <= value
        act_max = value
    end
    return (act_min, act_max)
end


"""
VHACD(filename::String, outputDirectory::String, outputBasename::String;
              showLog=false, resolution=100000, depth=20, concavity=0.0025, planeDownsampling=4, convexhullDownsampling=4,
              alpha=0.05, beta=0.05, gamma=0.00125, delta=0.05, pca=false, mode=false, convexhullApproximation=true,
              maxNumVerticesPerCH=64, minVolumePerCH=0.0001)

Decomposes a 3D surface into a set of near convex solids.
Calls external executable V-HACD based on https://github.com/kmammou/v-hacd

execute this function once before usage

filename        ... Input file, OBJ Files (.obj)
outputDirectory ... Output directory
outputBasename  ... Base name for output file (a number and file extension will be added automati   cally)
showLog         ... show log file upon completion
resolution      ... Maximum number of voxels generated during the voxelization stage (default=100,000, range=10,000-16,000,000)
depth           ... Maximum number of clipping stages. During each split stage, solids with a concavity higher than the user defined threshold are clipped according the 'best' clipping plane (default=20, range=1-32)
concavity       ... Maximum allowed concavity (default=0.0025, range=0.0-1.0)
planeDownsampling ... Controls the granularity of the search for the 'best' clipping plane (default=4, range=1-16)
convexhullDownsampling ... Controls the precision of the convex-hull generation process during the clipping plane selection stage (default=4, range=1-16)
alpha     ... Controls the bias toward clipping along symmetry planes (default=0.05, range=0.0-1.0)
beta      ... Controls the bias toward clipping along revolution axes (default=0.05, range=0.0-1.0)
gamma     ... Controls the maximum allowed concavity during the merge stage (default=0.00125, range=0.0-1.0)
delta     ... Controls the bias toward maximaxing local concavity (default=0.05, range=0.0-1.0)
pca       ... Enable/disable normalizing the mesh before applying the convex decomposition (default=false)
mode      ... false: voxel-based approximate convex decomposition, true: tetrahedron-based approximate convex decomposition
maxNumVerticesPerCH ... Controls the maximum number of triangles per convex-hull (default=64, range=4-1024)
minVolumePerCH      ... Controls the adaptive sampling of the generated convex-hulls (default=0.0001, range=0.0-0.01)
convexhullApproximation ... Enable/disable approximation when computing convex-hulls
"""
function VHACD(filename::String, concaveProperties::ConcaveProperties)
    @assert(isfile(filename))
    @assert(filename[end-3:end] == ".obj")
    @assert(isinteger(concaveProperties.resolution))
    @assert(concaveProperties.resolution>=10000 && concaveProperties.resolution<=16000000)
    @assert(isinteger(concaveProperties.depth))
    @assert(concaveProperties.depth>=1 && concaveProperties.depth<=32)
    @assert(concaveProperties.concavity>=0.0 && concaveProperties.concavity<=1.0)
    @assert(isinteger(concaveProperties.planeDownsampling))
    @assert(concaveProperties.planeDownsampling>=1 && concaveProperties.planeDownsampling<=16)
    @assert(isinteger(concaveProperties.convexhullDownsampling))
    @assert(concaveProperties.convexhullDownsampling>=1 && concaveProperties.convexhullDownsampling<=16)
    @assert(concaveProperties.alpha>=0.0 && concaveProperties.alpha<=1.0)
    @assert(concaveProperties.beta>=0.0 && concaveProperties.beta<=1.0)
    @assert(concaveProperties.gamma>=0.0 && concaveProperties.gamma<=1.0)
    @assert(concaveProperties.delta>=0.0 && concaveProperties.delta<=1.0)
    @assert(isinteger(concaveProperties.maxNumVerticesPerCH))
    @assert(concaveProperties.maxNumVerticesPerCH>=4 && concaveProperties.maxNumVerticesPerCH<=1024)
    @assert(concaveProperties.minVolumePerCH>=0.0 && concaveProperties.minVolumePerCH<=0.01)

    logfilename = tempname()

    input = "--input \"" * filename * "\""
    output = "--output " * concaveProperties.outputBasename
    logfilename = "--log " * logfilename
    resolution = "--resolution " * string(concaveProperties.resolution)
    depth = "--depth " * string(concaveProperties.depth)
    concavity = "--concavity " * string(concaveProperties.concavity)
    planeDownsampling = "--planeDownsampling " * string(concaveProperties.planeDownsampling)
    convexhullDownsampling = "--convexhullDownsampling " * string(concaveProperties.convexhullDownsampling)
    alpha = "--alpha " * string(concaveProperties.alpha)
    beta = "--beta " * string(concaveProperties.beta)
    gamma = "--gamma " * string(concaveProperties.gamma)
    delta = "--detlta " * string(concaveProperties.delta)
    pca = "--pca " * (concaveProperties.pca ? "1" : "0")
    mode = "--mode " * (concaveProperties.mode ? "1" : "0")
    maxNumVerticesPerCH = "--maxNumVerticesPerCH " *string(concaveProperties.maxNumVerticesPerCH)
    minVolumePerCH = "--minVolumePerCH " * string(concaveProperties.minVolumePerCH)
    convexhullApproximation = "--convexhullApproximation " * (concaveProperties.convexhullApproximation ? "1" : "0")

    parameters = input * " " * output * " " * logfilename * " " * resolution * " " * depth * " " * concavity * " " * planeDownsampling * " " * convexhullDownsampling * " " * alpha * " " * beta * " " * gamma * " " * delta * " " * pca * " " * mode * " " * maxNumVerticesPerCH * " " * minVolumePerCH * " " * convexhullApproximation

    if isdir(concaveProperties.outputDirectory) != true
        mkdir(concaveProperties.outputDirectory)
    end

    if isempty(readdir(concaveProperties.outputDirectory))   # outputDirectory is empty
        println("From VHACD: output directory is empty! see ", concaveProperties.outputDirectory)
end; end
