module RuntestsWithGraphics

using  Modia3D
import Modia3D.Test

@enum TestsExtend  shortTests normalTests completeTests
const testsExtend = normalTests
const title = "Modia3D (version=$(Modia3D.Version) with " * currentPlotPackage() * ")"

@time Test.@testset verbose=true "$title" begin
    include("includeTests.jl")
end

end
