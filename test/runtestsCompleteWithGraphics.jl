module RuntestsCompleteWithGraphics

using Modia3D
import Test

@enum TestsExtend  shortTests normalTests completeTests
const testsExtend = completeTests
const title = "Complete Modia3D (with " * currentPlotPackage() * ")"

@time Test.@testset verbose=true "$title" begin
    include("includeTests.jl")
end

end
