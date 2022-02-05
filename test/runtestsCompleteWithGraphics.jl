module RuntestsCompleteWithGraphics

using  Modia3D
import Modia3D.Test

@enum TestsExtend  shortTests normalTests completeTests
const testsExtend = completeTests
const title = "Complete Modia3D (version=$(Modia3D.Version) with " * currentPlotPackage() * ")"

@time Test.@testset verbose=true "$title" begin
    include("includeTests.jl")
end

end
