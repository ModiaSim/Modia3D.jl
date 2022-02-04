module RuntestsWithGraphics

using Modia3D
import Test

@enum TestsExtend  shortTests normalTests completeTests
const testsExtend = normalTests
const title = "Modia3D (with " * currentPlotPackage() * ")"

@time Test.@testset verbose=true "$title" begin
    include("includeTests.jl")
end

end
