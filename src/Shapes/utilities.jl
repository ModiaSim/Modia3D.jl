# add shape rotation so that `axis` becomes x-axis
function rotateAxis2x(axis, R_abs)
    if axis == 2
        Rabs = @SMatrix[0 1 0; 0 0 1; 1 0 0] * R_abs
    elseif axis == 3
        Rabs = @SMatrix[0 0 1; 1 0 0; 0 1 0] * R_abs
    else
        Rabs = R_abs
    end
    return Rabs
end

# add shape rotation so that `axis` becomes y-axis
function rotateAxis2y(axis, R_abs)
    if axis == 3
        Rabs = @SMatrix[0 1 0; 0 0 1; 1 0 0] * R_abs
    elseif axis == 1
        Rabs = @SMatrix[0 0 1; 1 0 0; 0 1 0] * R_abs
    else
        Rabs = R_abs
    end
    return Rabs
end

# add shape rotation so that `axis` becomes z-axis
function rotateAxis2z(axis, R_abs)
    if axis == 1
        Rabs = @SMatrix[0 1 0; 0 0 1; 1 0 0] * R_abs
    elseif axis == 2
        Rabs = @SMatrix[0 0 1; 1 0 0; 0 1 0] * R_abs
    else
        Rabs = R_abs
    end
    return Rabs
end


# Utility functions for FileMesh.jl
# getMeshInfos(): read file and return centroid, longestEdge and mesh data
#
# VHACD(): decomposes a concave file mesh in convex sub parts
#          see: https://github.com/kmammou/v-hacd

function getMeshInfos(filename::AbstractString, scaleFactor::SVector{3,Float64})
    objPoints = Vector{SVector{3,Float64}}()
    facesIndizes = Vector{SVector{3,Int64}}()
    vertices = Vector{SVector{3,Float64}}()
    faces = Vector{SVector{3,Int64}}()
    centroid::SVector{3,Float64} = Modia3D.ZeroVector3D(Float64)
    longestEdge::Float64 = 0.0

    mesh = FileIO.load(filename, pointtype=MeshIO.Point3{Float64}, facetype=MeshIO.TriangleFace{MeshIO.OneIndex{Int64}})
    for i in 1:length(MeshIO.coordinates(mesh))
        objPoint = SVector{3,Float64}(MeshIO.coordinates(mesh)[i][1]*scaleFactor[1],
                                      MeshIO.coordinates(mesh)[i][2]*scaleFactor[2],
                                      MeshIO.coordinates(mesh)[i][3]*scaleFactor[3])
        push!(objPoints, objPoint)
    end
    for i in 1:length(MeshIO.faces(mesh))
        push!(facesIndizes, MeshIO.faces(mesh)[i])
    end

    (vertices, faces) = cleanMesh(objPoints, facesIndizes)

    if length(vertices) != 0
        x_max::Float64 = 0.0
        x_min::Float64 = 0.0
        y_max::Float64 = 0.0
        y_min::Float64 = 0.0
        z_max::Float64 = 0.0
        z_min::Float64 = 0.0
        sum = Modia3D.ZeroVector3D(Float64)
        for i in 1:length(vertices)
            if i == 1
                x_max = vertices[i][1]
                x_min = vertices[i][1]
                y_max = vertices[i][2]
                y_min = vertices[i][2]
                z_max = vertices[i][3]
                z_min = vertices[i][3]
            else
                (x_min, x_max) = check_MinMax(x_min, x_max, vertices[i][1])
                (y_min, y_max) = check_MinMax(y_min, y_max, vertices[i][2])
                (z_min, z_max) = check_MinMax(z_min, z_max, vertices[i][3])
            end
            sum += vertices[i]
        end
        centroid = sum / length(vertices)
        longestEdge = max((x_max-x_min), (y_max-y_min), (z_max-z_min))
    end

    return (centroid, longestEdge, vertices, faces)
end

getMeshInfos(filename::AbstractString, scaleFactor::AbstractVector) =
getMeshInfos(filename, SVector{3,Float64}(scaleFactor))


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

# mesh cleaning: remove duplicate vertices
function cleanMesh(objPoints::Vector{SVector{3,Float64}}, facesIndizes::Vector{SVector{3,Int64}})
    vertices = Vector{SVector{3,Float64}}()
    faces = Vector{SVector{3,Int64}}()
    indexMap = Vector{Int64}()

    # generate cleaned vertex array
    ixNew = 0
    for ixOld in 1:length(objPoints)
        ixFound = 0
        for ixSearch in 1:ixNew
            if vertices[ixSearch] == objPoints[ixOld]
                ixFound = ixSearch
                continue
            end
        end
        if ixFound == 0
            # no dupe -> add to vertex array and index map
            push!(vertices, objPoints[ixOld])
            ixNew += 1
            push!(indexMap, ixNew)
        else
            # dupe -> add to index map
            push!(indexMap, ixFound)
        end
    end

    # generate updated face array
    for ixFace in 1:length(facesIndizes)
        if maximum(facesIndizes[ixFace]) <= length(indexMap)
            face = [indexMap[facesIndizes[ixFace][1]], indexMap[facesIndizes[ixFace][2]], indexMap[facesIndizes[ixFace][3]]]
            push!(faces, face)
        else
            @error("Mesh cleaning out of bounds error in face $ixFace.")
        end
    end

    #= check for equivalence
    for ixFace in 1:length(facesIndizes)
        for iVertex in 1:3
            if objPoints[facesIndizes[ixFace][iVertex]] != vertices[faces[ixFace][iVertex]]
                @error("Mesh cleaning equivalence error in face $ixFace.")
            end
        end
    end
    println("Mesh cleaning: Reduce number of vertices $(length(objPoints)) -> $(length(vertices)).")
    =#
    return (vertices, faces)
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
