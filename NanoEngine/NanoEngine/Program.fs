﻿open System
open System.Collections.Generic
open System.Runtime.CompilerServices

module Tests =

    open NanoEngine.Engine

    exception TestFailureException of string

    module Assert =
        let IsTrue condition message = if not condition then raise (TestFailureException message)
        let IsFalse condition message = if condition then raise (TestFailureException message)
        let AreEqual (expected: 'T) (actual: 'T) (message: string) =
            if expected <> actual then
                let fullMessage = $"{message} | Expected: {expected}, Fact: {actual}"
                raise (TestFailureException fullMessage)
        let AreNotEqual (expected: 'T) (actual: 'T) (message: string) =
            if expected = actual then
                let fullMessage = $"{message} | Expected: {expected}, Fact: {actual}"
                raise (TestFailureException fullMessage)
        let AreClose (expected: double) (actual: double) (tolerance: double) (message: string) =
            if abs(expected - actual) > tolerance then
                let fullMessage = $"{message} | Expected: {expected:F4}, Fact: {actual:F4}, Tolerance: {tolerance:F4}"
                raise (TestFailureException fullMessage)
        let Fail message = raise (TestFailureException message)

    type TestLocation =
        | Center
        | BorderMinX
        | BorderMaxX
        | BorderMinY
        | BorderMaxY
        | CornerMinXMinY
        | CornerMaxXMinY
        | CornerMinXMaxY
        | CornerMaxXMaxY
        override this.ToString() =
            match this with
            | Center -> "Center of World"
            | BorderMinX -> "Border of MinX"
            | BorderMaxX -> "Border of MaxX"
            | BorderMinY -> "Border of MinY"
            | BorderMaxY -> "Border of MaxY"
            | CornerMinXMinY -> "Corner of MinX/MinY"
            | CornerMaxXMinY -> "Corner of MaxX/MinY"
            | CornerMinXMaxY -> "Corner of MinX/MaxY"
            | CornerMaxXMaxY -> "Corner of MaxX/MaxY"

    type TestResult = | Success of string | Failure of string | InProgress

    type TestState =
        {
            Engine : T
            mutable TestPhase: int
            Memo: Dictionary<string, obj>
        }
        member inline this.BodyRepo = this.Engine.Bodies
        member inline this.IslandRepo = this.Engine.Islands
        member inline this.FloraRepo = this.Engine.Flora
        member inline this.GeometryRepo = this.Engine.Geometry
        member inline this.LiquidRepo = this.Engine.Liquid
        member inline this.ActiveHash = this.Engine.ActiveHash
        member inline this.SleepingHash = this.Engine.SleepingHash
        member inline this.Buffers = this.Engine.Buffers
            
    type TestRunner(initialState: TestState) =
        let mutable _state = initialState

        member _.State with get() = _state and set v = _state <- v
        member _.GetBody id = &Body.getRef id _state.BodyRepo
        member _.GetIsland islandId = &Island.getIslandRef islandId _state.IslandRepo
        member _.GetIslandRefForBody bodyId = &Island.getIslandRefForBody bodyId _state.IslandRepo
        member _.Memoize(key, value) = _state.Memo[key] <- box value
        member _.Recall<'T>(key) = unbox<'T> _state.Memo[key]

        member private this.GetWrappedWorldPos (loc: TestLocation) (qOff, rOff) z =
            let anchorPos =
                match loc with
                | Center         -> Vector3(WorldLimits.X / 2.0, WorldLimits.Y / 2.0, 0.0)
                | BorderMinX     -> Vector3(0.0, WorldLimits.Y / 2.0, 0.0)
                | BorderMaxX     -> Vector3(WorldLimits.X, WorldLimits.Y / 2.0, 0.0)
                | BorderMinY     -> Vector3(WorldLimits.X / 2.0, 0.0, 0.0)
                | BorderMaxY     -> Vector3(WorldLimits.X / 2.0, WorldLimits.Y, 0.0)
                | CornerMinXMinY -> Vector3(0.0, 0.0, 0.0)
                | CornerMaxXMinY -> Vector3(WorldLimits.X, 0.0, 0.0)
                | CornerMinXMaxY -> Vector3(0.0, WorldLimits.Y, 0.0)
                | CornerMaxXMaxY -> Vector3(WorldLimits.X, WorldLimits.Y, 0.0)

            let originPoint = Grid.convertHexToWorld 0 0 0 0.0
            let offsetPoint = Grid.convertHexToWorld qOff rOff 0 0.0
            let worldOffsetVec = offsetPoint - originPoint

            let finalX = WorldLimits.wrapX (anchorPos.X + worldOffsetVec.X)
            let finalY = WorldLimits.wrapY (anchorPos.Y + worldOffsetVec.Y)
            Vector3(finalX, finalY, z)

        member this.GetSurfacePosForLocation (loc: TestLocation) (qOff, rOff, platZ) =
            let struct(anchorQ, anchorR, _) = this.Recall<struct(int * int * SubPrismCoords)>("platformAnchor")

            let targetQ = anchorQ + qOff
            let targetR = anchorR + rOff
            let targetZ = platZ

            let hexBasePos = Grid.convertHexToWorld targetQ targetR targetZ HEX_HEIGHT

            let surfaceZ_offset = HEX_HEIGHT
            Vector3(hexBasePos.X, hexBasePos.Y, hexBasePos.Z + surfaceZ_offset)
            
        member this.CreatePlatform(loc: TestLocation, z, size) =
            let anchorQ, anchorR =
                match loc with
                | Center         -> (GRID_WIDTH_Q / 2, GRID_DEPTH_R / 2)
                | BorderMinX     -> (0, GRID_DEPTH_R / 2)
                | BorderMaxX     -> (GRID_WIDTH_Q, GRID_DEPTH_R / 2)
                | BorderMinY     -> (GRID_WIDTH_Q / 2, 0)
                | BorderMaxY     -> (GRID_WIDTH_Q / 2, GRID_DEPTH_R)
                | CornerMinXMinY -> (0, 0)
                | CornerMaxXMinY -> (GRID_WIDTH_Q, 0)
                | CornerMinXMaxY -> (0, GRID_DEPTH_R)
                | CornerMaxXMaxY -> (GRID_WIDTH_Q, GRID_DEPTH_R)

            let inline wrap (value: int) (max: int) = (value % max + max) % max

            for dq = -size to size do
                for dr = -size to size do
                    let dc = -dq - dr

                    if abs dq <= size && abs dr <= size && abs dc <= size then
   
                        let targetQ = anchorQ + dq
                        let targetR = anchorR + dr

                        let finalQ = wrap targetQ GRID_WIDTH_Q
                        let finalR = wrap targetR GRID_DEPTH_R

                        let finalZ = Math.Clamp(z, 0, GRID_HEIGHT_Z - 1)

                        for i in 0..11 do
                            let finalCoords = SubPrismCoords(finalQ, finalR, finalZ, i)
                            _state.GeometryRepo |> Geometry.addPrism finalCoords Material.Static

            let anchorGridCoords = SubPrismCoords(wrap anchorQ GRID_WIDTH_Q, wrap anchorR GRID_DEPTH_R, z, 0)
            let rawAnchorCoords = struct(anchorQ, anchorR, anchorGridCoords)
            _state.Memo["platformAnchor"] <- box rawAnchorCoords
            rawAnchorCoords

        member this.CreateTree(q, r, z, height, thickness, mass, breakThreshold) =
            let supportCoords = SubPrismCoords.Normalize(q, r, z, 6)
            let baseCenterPos = supportCoords |> Grid.getTriangularPrismCenter
            let treePos = baseCenterPos + Vector3(0.0, 0.0, height / 2.0)
            
            let treeId = _state.Engine |> nextBodyId
            let finalTreePos = Vector3(WorldLimits.wrapX treePos.X, WorldLimits.wrapY treePos.Y, treePos.Z)
            let treeData = Flora.create treeId Body.BodyType.Tree mass finalTreePos height thickness supportCoords 0.5 breakThreshold
            _state.FloraRepo |> Flora.addTree treeData
            (treeId, finalTreePos)
            
        member _.CreateBody(pos, vel: Vector3, mass, dims, bType, friction) =
            Assert.IsTrue (vel.Magnitude() <= MAX_SPEED + 1e-5) "The initial velocity of the body exceeds the limit"
            let bodyId = _state.Engine |> nextBodyId
            let body = Body.T(bodyId, bType, mass, dims, Matrix3x3.Identity, pos, vel, SubPrismCoords.Zero, friction)
            Body.tryAdd &body _state.BodyRepo |> ignore
            bodyId

    type TestDefinition = {
        Setup: TestRunner -> TestLocation -> unit 
        Check: TestRunner -> TestDefinition -> TestLocation -> int -> TestResult
        MaxSteps: int
    }


open Tests
open NanoEngine.Engine
open System.Diagnostics

[<EntryPoint>]
let main _ =
    let dt = 0.025
    
    let logBodyState (state: TestState) (step: int) =
        if (state.BodyRepo |> Body.getAll).Count < 10 then
            printfn "[%3d]" step
            for kvp in state.BodyRepo |> Body.getAll do
                let b = kvp.Value
                printfn
                    "ID %d: Type: %A, P(%.3f, %.3f, %.3f), V(%.4f, %.4f, %.4f), D(%.4f, %.4f, %.4f), IsFalling: %b, FallProg: %.2f, Snapped: %b"
                    b.Id
                    b.BodyType
                    b.Position.X
                    b.Position.Y
                    b.Position.Z
                    b.Velocity.X
                    b.Velocity.Y
                    b.Velocity.Z
                    b.Dimensions.X
                    b.Dimensions.Y
                    b.Dimensions.Z
                    b.IsFallingOver
                    b.FallRotationProgress
                    b.IsSnappedToGrid
                
    let runTest (testDef: TestDefinition) (location: TestLocation) =
        printfn "Location: %s" (location.ToString())
        
        use engine = createEngine dt

        let initialState =
            {
                Engine = engine
                TestPhase = 0
                Memo = Dictionary<string, obj>()
            }
        
        let runner = TestRunner(initialState)
        
        
        let mutable i = 1
        try
            testDef.Setup runner location
            runner.State.IslandRepo |> Island.init
            runner.State.FloraRepo |> Flora.init

            let isProblematicTest = 
                match location with
                | _ -> false
                
            let mutable finalResult: TestResult = Failure $"The test did not complete within {testDef.MaxSteps} steps"
            let mutable continueLoop = true
            while i <= testDef.MaxSteps do
                if continueLoop then

                    engine |> Simulation.step 

                    let phases = [||]
                    if isProblematicTest && (phases |> Array.contains runner.State.TestPhase)then
                        logBodyState runner.State i
                        
                    match testDef.Check runner testDef location i with
                    | Success msg -> finalResult <- Success msg; continueLoop <- false
                    | Failure msg -> finalResult <- Failure msg; continueLoop <- false
                    | InProgress -> ()
            
                i <- i + 1

            match finalResult with
            | Success msg ->
                printfn "[PASS] %s" msg
                true
            | Failure msg ->
                printfn "[FAIL] %s" msg
                false
            | InProgress ->
                printfn "[FAIL] Test completed without result"
                false
        with
        | TestFailureException msg ->
            printfn "[FAIL]: %s" msg
            logBodyState runner.State i
            false
        | ex -> printfn "[FAIL]: %s\n%s" ex.Message ex.StackTrace; false

    let tests = ResizeArray<TestDefinition>()
    
    tests.Add({
        MaxSteps = 1500
        Setup = fun runner loc ->
            let platZ = 10
            let platformRadius = 8
            
            let struct(centerQ, centerR, centerGridCoords) = runner.CreatePlatform(loc, platZ, platformRadius)
                
            runner.Memoize("platformCenterGridCoords", centerGridCoords)

            let centerWorldPos = centerGridCoords |> Grid.getTriangularPrismCenter
            
            let surfaceZ = (double platZ + 1.0) * HEX_HEIGHT
            runner.Memoize("platformSurfaceZ", surfaceZ)
            
            let bodyStartPos = Vector3(centerWorldPos.X, centerWorldPos.Y, surfaceZ + 5.0) 
            let firstId = runner.CreateBody(bodyStartPos, Vector3.Zero, 50.0, Vector3.One, Body.BodyType.Generic, 0.2)
            runner.Memoize("firstId", firstId)

            let tree1Id, _ = runner.CreateTree(centerQ - 1, centerR - 4, platZ, 7.0, 0.4, 300.0, 80.0)
            runner.Memoize("tree1Id", tree1Id)
            let tree2Id, _ = runner.CreateTree(centerQ + 4, centerR + 3, platZ, 1.0, 0.01, 1.0, 2.0)
            runner.Memoize("tree2Id", tree2Id)

            runner.Memoize("secondId", -1)
            runner.Memoize("thirdId", -1)
            runner.Memoize("bulletId1", -1)
            runner.Memoize("bulletId2", -1)
            runner.Memoize("dominoId", -1)
            runner.Memoize("bulldozerId", -1)

        Check = fun runner testDef loc i ->
            match runner.State.TestPhase with
            | 0 -> 
                if runner.State.IslandRepo.SleepingIslandIds.Count = 1 then
                    runner.State.TestPhase <- 1
                elif i > 55 then Assert.Fail "STAGE 0 FAIL: The lower body was unable to sleep"
                InProgress
            | 1 ->
                let firstId = runner.Recall<int>("firstId")
                let bottomBody = runner.GetBody(firstId)
                let topPos1 = bottomBody.Position + Vector3(0.2, 0.0, 5.0)
                let secondId = runner.CreateBody(topPos1, Vector3.Zero, 20.0, Vector3.One, Body.BodyType.Generic, 0.2)
                runner.State.IslandRepo |> Island.addBody secondId
                runner.Memoize("secondId", secondId)
                runner.State.TestPhase <- 2
                InProgress
            | 2 ->
                if runner.State.IslandRepo.SleepingIslandIds.Count = 1 && runner.State.IslandRepo.ActiveIslandIds.Count = 0 then
                    runner.State.TestPhase <- 3
                elif i > 100 then Assert.Fail "STAGE 2 FAIL: A stack of two bodies couldn't sleep"
                InProgress
            | 3 ->
                let topBody1 = runner.GetBody(runner.Recall<int>("secondId"))
                let topPos2 = topBody1.Position + Vector3(0.2, 0.0, 3.0)
                let thirdId = runner.CreateBody(topPos2, Vector3.Zero, 30.0, Vector3.One, Body.BodyType.Generic, 0.2)
                runner.State.IslandRepo |> Island.addBody thirdId
                runner.Memoize("thirdId", thirdId)
                runner.State.TestPhase <- 4
                InProgress
            | 4 ->
                if runner.State.IslandRepo.SleepingIslandIds.Count = 1 && runner.State.IslandRepo.ActiveIslandIds.Count = 0 then
                    runner.State.TestPhase <- 5
                elif i > 150 then Assert.Fail "STAGE 4 FAIL: A stack of three bodies couldn't sleep"
                InProgress
            | 5 ->
                let thirdId = runner.Recall<int>("thirdId")
                let thirdBody = runner.GetBody(thirdId)
                let bulletStartPos = thirdBody.Position - Vector3(2.0, 0.0, -0.5)
                let bulletVel = Vector3(10.0, 0.0, 0.0)
                let bulletId1 = runner.CreateBody(bulletStartPos, bulletVel, 2.0, Vector3.One, Body.BodyType.Generic, 0.2)
                runner.State.IslandRepo |> Island.addBody bulletId1
                runner.Memoize("bulletId1", bulletId1)
                runner.State.TestPhase <- 6
                InProgress
            | 6 ->
                if runner.State.IslandRepo.SleepingIslandIds.Count = 2 && runner.State.IslandRepo.ActiveIslandIds.Count = 0 then
                    runner.State.TestPhase <- 7
                    InProgress
                elif i > 210 then Assert.Fail "STAGE 6 FAIL: The scene did not fall asleep after the first blow"
                else InProgress
            | 7 ->
                let targetId = runner.Recall<int>("thirdId")
                let targetBody = runner.GetBody(targetId)
                let bulletStartPos = targetBody.Position - Vector3(2.0, 0.0, -0.3) 
                let bulletVel = Vector3(10.0, 0.0, 0.0)
                let bulletId2 = runner.CreateBody(bulletStartPos, bulletVel, 2.0, Vector3.One, Body.BodyType.Generic, 0.1)
                runner.State.IslandRepo |> Island.addBody bulletId2
                runner.Memoize("bulletId2", bulletId2)

                runner.State.TestPhase <- 8
                InProgress

            | 8 | 9 ->
                if runner.State.TestPhase = 8 then
                    runner.State.TestPhase <- 9
                
                if runner.State.IslandRepo.ActiveIslandIds.Count > 0 then
                    if i > 280 then Assert.Fail "STAGE 8/9 FAIL: The bodies did not fall asleep after the second blow"
                    else InProgress
                else
                    Assert.AreEqual 2 runner.State.IslandRepo.SleepingIslandIds.Count "STAGE 9 FAIL: Incorrect number of sleeping islands. Expected 2"

                    let firstId   = runner.Recall<int>("firstId")
                    let secondId  = runner.Recall<int>("secondId")
                    let thirdId   = runner.Recall<int>("thirdId")
                    let bulletId1 = runner.Recall<int>("bulletId1")
                    let bulletId2 = runner.Recall<int>("bulletId2")

                    let main_tower_island_ref = runner.GetIslandRefForBody firstId

                    let broken_part_island_ref = runner.GetIslandRefForBody bulletId2

                    Assert.AreNotEqual main_tower_island_ref.Id broken_part_island_ref.Id "STAGE 9 FAIL: Both groups of bodies are mistakenly located on the same island"

                    let mainTowerExpectedBodies = set [firstId; secondId; thirdId]
                    let mainTowerActualBodies = set main_tower_island_ref.Bodies
                    Assert.AreEqual mainTowerExpectedBodies mainTowerActualBodies "STAGE 9 FAIL: Incorrect main tower composition"

                    let brokenPartExpectedBodies = set [bulletId1; bulletId2]
                    let brokenPartActualBodies = set broken_part_island_ref.Bodies
                    Assert.AreEqual brokenPartExpectedBodies brokenPartActualBodies "STAGE 9 FAIL: Incorrect composition of the broken off part"
                    runner.State.TestPhase <- 10
                    InProgress
            | 10 ->

                let targetId = runner.Recall<int>("thirdId")
                let targetBody = runner.GetBody(targetId)

                let struct(rawTargetQ, rawTargetR, platformZ_index) = 
                    Grid.convertWorldToRawGridCoords targetBody.Position

                let platformSurfaceZ = (double platformZ_index + 1.0) * HEX_HEIGHT

                let dominoGridR = rawTargetR - 1
                
                let dominoBaseWorldPos = Grid.convertHexToWorld rawTargetQ dominoGridR platformZ_index HEX_HEIGHT

                let dominoDims = Vector3(1.5, 0.3, 8.0) 
                let dominoStartPos = dominoBaseWorldPos + Vector3(0.0, 0.0, dominoDims.Z / 2.0)
                let wrappedStartPos = Vector3(
                    WorldLimits.wrapX dominoStartPos.X,
                    WorldLimits.wrapY dominoStartPos.Y,
                    dominoStartPos.Z
                )

                let dominoId = runner.CreateBody(wrappedStartPos, Vector3.Zero, 1000.0, dominoDims, Body.BodyType.Generic, 0.5)
                runner.State.IslandRepo |> Island.addBody dominoId
                runner.Memoize("dominoId", dominoId)
                runner.Memoize("platformSurfaceZ_ForDomino", platformSurfaceZ)

                let wallHexOffsets = [| (-1, 3); (0, 3); (1, 3) |]
                
                for dq, dr in wallHexOffsets do
                    let finalRawQ = rawTargetQ + dq
                    let finalRawR = rawTargetR + dr
                    
                    for z_layer in platformZ_index-3 .. platformZ_index do
                        for i in 0..11 do
                            let wallPartCoords = SubPrismCoords.Normalize(finalRawQ, finalRawR, z_layer, i)
                            runner.State.GeometryRepo |> Geometry.addPrism wallPartCoords Material.Static
                runner.State.TestPhase <- 11
                InProgress

            | 11 ->
                let dominoId = runner.Recall<int>("dominoId")
                let domino = &runner.GetBody(dominoId)
                let surfaceZ = runner.Recall<double>("platformSurfaceZ_ForDomino")

                let pivotEdgeSign = -1.0

                let pivotOffsetLocal = Vector3(0.0, domino.Dimensions.Y / 2.0 * pivotEdgeSign, -domino.Dimensions.Z / 2.0)
  
                let pivotOffsetWorld = domino.Orientation * pivotOffsetLocal

                let pivotX = WorldLimits.wrapX (domino.Position.X + pivotOffsetWorld.X)
                let pivotY = WorldLimits.wrapY (domino.Position.Y + pivotOffsetWorld.Y)
                domino.FallPivotPoint <- Vector3(pivotX, pivotY, surfaceZ)

                let pivot = domino.FallPivotPoint
                let pos = domino.Position
                let mutable offset = Vector3.Zero
                offset.X <- pos.X - pivot.X
                if offset.X > WorldLimits.X * 0.5 then offset.X <- offset.X - WorldLimits.X
                elif offset.X < -WorldLimits.X * 0.5 then offset.X <- offset.X + WorldLimits.X

                offset.Y <- pos.Y - pivot.Y
                if offset.Y > WorldLimits.Y * 0.5 then offset.Y <- offset.Y - WorldLimits.Y
                elif offset.Y < -WorldLimits.Y * 0.5 then offset.Y <- offset.Y + WorldLimits.Y

                offset.Z <- pos.Z - pivot.Z
                domino.InitialCenterOffsetFromPivot <- offset

                domino.FallRotationAxis <- domino.Orientation.R0 * pivotEdgeSign

                domino.FallDuration <- 2.0 
                domino.FallInitialOrientation <- domino.Orientation
                domino.IsForceFalling <- true
                runner.State.TestPhase <- 12
                InProgress

            | 12 ->
                if runner.State.IslandRepo.ActiveIslandIds.Count = 0 then
                    runner.State.TestPhase <- 13
                    InProgress
                elif i >= 900 then
                    Assert.Fail "STAGE 13 FAIL: The scene failed to stabilize after the dominoes fell"
                else
                    InProgress

            | 13 ->
                Assert.AreEqual 3 runner.State.IslandRepo.SleepingIslandIds.Count "STAGE 13 FAIL: Incorrect number of sleeping islands. Expected 3"

                let firstId   = runner.Recall<int>("firstId")
                let secondId  = runner.Recall<int>("secondId")
                let thirdId   = runner.Recall<int>("thirdId")
                let bulletId1 = runner.Recall<int>("bulletId1")
                let bulletId2 = runner.Recall<int>("bulletId2")
                let dominoId  = runner.Recall<int>("dominoId")

                let islandOfTripleStack = runner.GetIslandRefForBody dominoId
                let islandOfDoubleStack = runner.GetIslandRefForBody bulletId1
                let islandOfLoneCube    = runner.GetIslandRefForBody thirdId

                let allActualIslandIds = set [islandOfTripleStack.Id; islandOfDoubleStack.Id; islandOfLoneCube.Id]
                Assert.AreEqual 3 allActualIslandIds.Count "STAGE 13 FAIL: 3 unique islands were expected"

                Assert.AreEqual (set [firstId; secondId; dominoId]) (set islandOfTripleStack.Bodies) "STAGE 13 FAIL: Incorrect composition of the island with a triple stack"
                Assert.AreEqual (set [bulletId1; bulletId2]) (set islandOfDoubleStack.Bodies) "STAGE 13 FAIL: Incorrect composition of the island with a double stack"
                Assert.AreEqual (set [thirdId]) (set islandOfLoneCube.Bodies) "STAGE 13 FAIL: Incorrect composition of the island with a single cube"

                let tolerance = 0.185

                let body1_ref   = runner.GetBody(firstId)
                let body4_ref   = runner.GetBody(secondId)
                let domino_ref  = runner.GetBody(dominoId)

                let topOfDomino   = domino_ref.Position.Z + domino_ref.Dimensions.Z / 2.0 - tolerance
                let bottomOfBody1 = body1_ref.Position.Z - body1_ref.Dimensions.Z / 2.0
                Assert.AreClose topOfDomino bottomOfBody1 tolerance "STAGE 13 FAIL: Triple stack geometry. Body 1 must lie on domino (8)"
                
                let topOfBody1    = body1_ref.Position.Z + body1_ref.Dimensions.Z / 2.0
                let bottomOfBody4 = body4_ref.Position.Z - body4_ref.Dimensions.Z / 2.0
                Assert.AreClose topOfBody1 bottomOfBody4 tolerance "STAGE 13 FAIL: Geometry of a triple stack. Body 4 must lie on body 1"

                let body6_ref   = runner.GetBody(bulletId1)
                let body7_ref   = runner.GetBody(bulletId2)

                let topOfBody6    = body6_ref.Position.Z + body6_ref.Dimensions.Z / 2.0
                let bottomOfBody7 = body7_ref.Position.Z - body7_ref.Dimensions.Z / 2.0
                Assert.AreClose topOfBody6 bottomOfBody7 tolerance "STAGE 13 FAIL: Double stack geometry. Body 7 must lie on body 6"

                let body5_ref = runner.GetBody(thirdId)
                let platformZ_by_domino = domino_ref.Position.Z - domino_ref.Dimensions.Z / 2.0

                let bottomOfBody5 = body5_ref.Position.Z - body5_ref.Dimensions.Z / 2.0
                Assert.AreClose platformZ_by_domino bottomOfBody5 tolerance "STAGE 13 FAIL: Platform geometry: The lone cube (5) must lie on the same level as the domino (8)"
                
                let bottomOfBody6 = body6_ref.Position.Z - body6_ref.Dimensions.Z / 2.0
                Assert.AreClose platformZ_by_domino bottomOfBody6 tolerance "STAGE 13 FAIL: Platform geometry: The base of the double stack (6) must lie at the same level as the domino (8)"

                runner.State.TestPhase <- 14
                InProgress
                
            | 14 ->

                let northernTreeId = runner.Recall<int>("tree2Id")
                let northernTree = runner.State.FloraRepo.AllTrees[northernTreeId]
                let dominoId = runner.Recall<int>("dominoId")
                let domino = runner.GetBody(dominoId)

                let swiperWidth = 6.0
                let swiperLength = 1.0
                let swiperHeight = 3.0
                let swiperDims = Vector3(swiperLength, swiperWidth, swiperHeight)
                let gapFromTree = 0.5
                let clearanceFromDomino = 0.1
                let swiperNorthEdgeY = northernTree.Position.Y - gapFromTree
                let swiperCenterY_unwrapped = swiperNorthEdgeY - (swiperWidth / 2.0) - 0.1
                let swiperCenterY = WorldLimits.wrapY swiperCenterY_unwrapped
                let dominoTopZ = domino.Position.Z + domino.Dimensions.Z / 2.0
                let swiperBottomZ = dominoTopZ + clearanceFromDomino
                let swiperCenterZ = swiperBottomZ + (swiperHeight / 2.0)

                let struct(_, _, centerGridCoords) = runner.Recall<struct(int*int*SubPrismCoords)>("platformAnchor")

                let platformCenterX =
                    Grid.convertHexToWorld centerGridCoords.Q centerGridCoords.R 0 HEX_HEIGHT
                    |> fun v -> v.X
                
                let safeMarginLeft = 6.0
                let safeMarginRight = 8.0

                let startX_unwrapped = platformCenterX + safeMarginLeft
                let stopX_unwrapped = platformCenterX - safeMarginRight

                let startX = WorldLimits.wrapX startX_unwrapped
                let stopX = WorldLimits.wrapX stopX_unwrapped

                runner.Memoize("startX", startX)
                runner.Memoize("stopX", stopX)

                let startPos = Vector3(startX, swiperCenterY, swiperCenterZ)
                let swiperVel = Vector3(-MAX_SPEED, 0.0, 0.0)

                let bulldozerId = runner.CreateBody(startPos, swiperVel, 50000.0, swiperDims, Body.BodyType.Generic, 0.8)
                runner.Memoize("bulldozerId", bulldozerId)
                
                runner.GetBody(bulldozerId).IsGravityEnabled <- false
                runner.State.IslandRepo |> Island.addBody bulldozerId

                runner.State.TestPhase <- 15
                InProgress

            | 15 ->
                if runner.State.IslandRepo.ActiveIslandIds.Count = 0 then
                    runner.State.TestPhase <- 16
                    InProgress
                elif i >= 1500 then
                    Assert.Fail "STAGE 15 FAIL: The scene did not stabilize after being cleared by a bulldozer"
                else
                    let bulldozerId = runner.Recall<int>("bulldozerId")
                    let swiper = &runner.GetBody(bulldozerId)
                    
                    if not <| Unsafe.IsNullRef &swiper && not swiper.IsGravityEnabled then
                        
                        let stopX = runner.Recall<double>("stopX")
                        let currentX = swiper.Position.X

                        let deltaX = currentX - stopX

                        let toroidalDist = WorldLimits.relativeX deltaX

                        let hasReachedTarget = toroidalDist <= 0.5

                        let emergencyStop = swiper.Velocity.X > 0.0

                        if hasReachedTarget || emergencyStop then
                            swiper.Velocity <- Vector3.Zero
                            swiper.IsGravityEnabled <- true

                    InProgress
                    
            | 16 ->
                Assert.AreEqual 0 runner.State.IslandRepo.ActiveIslandIds.Count "FINISH: All the islands were supposed to be asleep"

                let sleepingIslandCount = runner.State.IslandRepo.SleepingIslandIds.Count
                let platformZ = (double 10 + 1.0) * HEX_HEIGHT
                let floorZ = 0.0
                let tolerance = 0.51

                let checkIsOnSurface (bodyId: int) (surfaceZ: double) (bodyName: string) =
                    let body = runner.GetBody(bodyId)
                    let bodyBottomZ = body.Position.Z - body.Dimensions.Z / 2.0
                    Assert.AreClose surfaceZ bodyBottomZ tolerance $"{bodyName} (ID {bodyId}) must lie at a height {surfaceZ}"

                if sleepingIslandCount = 8 then

                    let body1Id = runner.Recall<int>("firstId")
                    let treeId = runner.Recall<int>("tree1Id")
                    let body4Id = runner.Recall<int>("secondId")
                    let body5Id = runner.Recall<int>("thirdId")
                    let body6Id = runner.Recall<int>("bulletId1")
                    let body7Id = runner.Recall<int>("bulletId2")
                    let dominoId = runner.Recall<int>("dominoId")
                    let bulldozerId = runner.Recall<int>("bulldozerId")

                    let allBodies = [body1Id; treeId; body4Id; body5Id; body6Id; body7Id; dominoId; bulldozerId]
                    for bodyId in allBodies do
                        let island = runner.GetIslandRefForBody bodyId
                        Assert.AreEqual 1 island.Bodies.Count $"The body {bodyId} must be in a single island"

                    checkIsOnSurface body5Id platformZ "Cube 5"
                    checkIsOnSurface dominoId platformZ "Domino 8"
                    checkIsOnSurface bulldozerId platformZ "Bulldozer 9"

                    checkIsOnSurface body1Id floorZ "Cube 1"
                    checkIsOnSurface treeId floorZ "Broken tree 2"
                    checkIsOnSurface body4Id floorZ "Cube 4"
                    checkIsOnSurface body6Id floorZ "Cube 6"
                    checkIsOnSurface body7Id floorZ "Cube 7"
                    Success $"{loc} COMPLETED"

                elif sleepingIslandCount = 5 then
                    let body1Id = runner.Recall<int>("firstId")
                    let treeId = runner.Recall<int>("tree1Id")
                    let body4Id = runner.Recall<int>("secondId")
                    let body5Id = runner.Recall<int>("thirdId")
                    let body6Id = runner.Recall<int>("bulletId1")
                    let body7Id = runner.Recall<int>("bulletId2")
                    let dominoId = runner.Recall<int>("dominoId")
                    let bulldozerId = runner.Recall<int>("bulldozerId")

                    let island_cross_world_pile = runner.GetIslandRefForBody body1Id
                    Assert.AreEqual (set [body1Id; body4Id; body6Id]) (set island_cross_world_pile.Bodies) "Incorrect composition [1, 4, 6]"
                    
                    let island_lone_floor_cube = runner.GetIslandRefForBody body7Id
                    Assert.AreEqual (set [body7Id]) (set island_lone_floor_cube.Bodies) "Incorrect composition of a lonely cube on the floor [7]"

                    let island_lone_platform_cube = runner.GetIslandRefForBody body5Id
                    Assert.AreEqual (set [body5Id]) (set island_lone_platform_cube.Bodies) "Cube [5] should be alone on the platform"

                    let island_domino = runner.GetIslandRefForBody dominoId
                    Assert.AreEqual (set [dominoId]) (set island_domino.Bodies) "Domino [8] must be one on the platform"

                    let island_swiper = runner.GetIslandRefForBody bulldozerId
                    Assert.AreEqual (set [bulldozerId]) (set island_swiper.Bodies) "There should be one bulldozer [9] on the platform"

                    checkIsOnSurface body5Id platformZ "Cube 5"
                    checkIsOnSurface dominoId platformZ "Domino 8"
                    checkIsOnSurface bulldozerId platformZ "Bulldozer 9"
                    checkIsOnSurface body1Id floorZ "Cube 1"
                    checkIsOnSurface body7Id floorZ "Cube 7"

                    let body1 = runner.GetBody(body1Id)
                    let body4 = runner.GetBody(body4Id)
                    let topOfBody1 = body1.Position.Z + body1.Dimensions.Z / 2.0
                    let bottomOfBody4 = body4.Position.Z - body4.Dimensions.Z / 2.0
                    Assert.AreClose topOfBody1 bottomOfBody4 tolerance "Body 4 must stand on body 1"

                    Assert.IsTrue (runner.State.FloraRepo.AllTrees.ContainsKey(treeId)) "The southern tree should have survived"
                    Success $"[{loc}] COMPLETED"
                else
                    Assert.Fail $"[{loc}] Unexpected number of sleeping islands: {sleepingIslandCount}. Expected 8 or 5"
            | _ -> InProgress
    })
    
    let mutable passedCount = 0
    let mutable totalRuns = 0
    let stopwatch = Stopwatch.StartNew()
    let locationsToTest = [
        TestLocation.Center
        TestLocation.BorderMinX
        TestLocation.BorderMaxX
        TestLocation.BorderMinY
        TestLocation.BorderMaxY
        TestLocation.CornerMinXMinY
        TestLocation.CornerMaxXMinY
        TestLocation.CornerMinXMaxY
        TestLocation.CornerMaxXMaxY
    ]
    
    for testDef in tests do
        for location in locationsToTest do
            totalRuns <- totalRuns + 1
            if runTest testDef location then passedCount <- passedCount + 1
            
    stopwatch.Stop()
    printfn "Completed: %d / %d" passedCount totalRuns
    printfn "Duration: %d ms" stopwatch.ElapsedMilliseconds
    if passedCount = totalRuns then
        printfn "SUCCESS"
    else
        printfn "FAILURE"
    0