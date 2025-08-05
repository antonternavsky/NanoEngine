namespace NanoEngine
#nowarn "9"
#nowarn "3391"

open System.Runtime.CompilerServices
open System.Runtime.InteropServices
open Collections.Pooled
open System
open Microsoft.FSharp.NativeInterop
open System.Collections.Generic
open System.Threading
open Serilog

module Engine =
    let [<Literal>] PENETRATION_SLOP = 0.01
    let [<Literal>] MAX_PENETRATION = PENETRATION_SLOP * 4.0
    let [<Literal>] CORRECTION_PERCENT = 0.65
    let [<Literal>] RESTITUTION_THRESHOLD = 2.0
    let [<Literal>] EPSILON = 1e-6
    let [<Literal>] EPSILON_X2 = EPSILON * EPSILON
    let [<Literal>] SLEEP_VELOCITY_THRESHOLD_SQ = 0.01
    let [<Literal>] CONTACT_TTL = 3
    let [<Literal>] FRAMES_TO_SLEEP = CONTACT_TTL + 2
    let [<Literal>] PI = Math.PI
    let SQRT3 = Math.Sqrt 3.0
    let [<Literal>] MIN_DIMENSION_THRESHOLD = 0.001
    let [<Literal>] MAX_DIMENSION = 40.0
    let [<Literal>] GRID_WIDTH_Q = 65536
    let [<Literal>] GRID_DEPTH_R = 65536
    let [<Literal>] GRID_HEIGHT_Z = 248
    let [<Literal>] HEX_WIDTH = 1.0
    let HEX_RADIUS = HEX_WIDTH / SQRT3
    let [<Literal>] HEX_HEIGHT = 1.0
    let [<Literal>] MAX_SPEED = 10.0
    let MAX_SPEED_SQ = pown MAX_SPEED 2
    let WORLD_HEIGHT_IN_METERS = double GRID_HEIGHT_Z * HEX_HEIGHT
    let [<Literal>] FRAMES_TO_UNROOT = 2
    
    [<RequireQualifiedAccess>]
    module Dispose =
        let inline action(a: #IDisposable) =
            use a = a
            ()

    [<RequireQualifiedAccess>]    
    module Stack =
        let inline alloc size = Span<'a>(size |> NativePtr.stackalloc<'a> |> NativePtr.toVoidPtr, size)

    [<Struct; StructLayout(LayoutKind.Explicit, Size = 24)>]
    type Vector3 =
        new(x, y, z) = { X = x; Y = y; Z = z }
        [<FieldOffset(0)>]
        val mutable X: double

        [<FieldOffset(8)>]
        val mutable Y: double

        [<FieldOffset(16)>]
        val mutable Z: double
        override this.ToString() = $"Vector3[{this.X:F3};{this.Y:F3};{this.Z:F3}]"

        static member inline Zero = Vector3(0.0, 0.0, 0.0)
        static member inline One = Vector3(1.0, 1.0, 1.0)
        static member inline Up = Vector3(0.0, 0.0, 1.0)
        static member inline Down = Vector3(0.0, 0.0, -1.0)
        static member inline UnitX = Vector3(1.0, 0.0, 0.0)
        static member inline UnitY = Vector3(0.0, 1.0, 0.0)
        static member inline (+)(a: Vector3, b: Vector3) =
            Vector3(a.X + b.X, a.Y + b.Y, a.Z + b.Z)

        static member inline (~-) (v: Vector3) = Vector3(-v.X, -v.Y, -v.Z)
        static member inline (-)(a: Vector3, b: Vector3) =
            Vector3(a.X - b.X, a.Y - b.Y, a.Z - b.Z)
        
        static member inline Mul(v: Vector3, s: double) = Vector3(v.X * s, v.Y * s, v.Z * s)
        static member inline (*)(v: Vector3, s: double) = Vector3(v.X * s, v.Y * s, v.Z * s)
        static member inline (*)(s: double, v: Vector3) = Vector3(v.X * s, v.Y * s, v.Z * s)
        static member inline (/)(v: Vector3, s: double) = Vector3(v.X / s, v.Y / s, v.Z / s)

        static member inline Dot(a: Vector3, b: Vector3) = a.X * b.X + a.Y * b.Y + a.Z * b.Z

        static member inline Cross(a: Vector3, b: Vector3) =
            Vector3(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X)

        member inline this.MagnitudeSq() = Vector3.Dot(this, this)
        member inline this.Magnitude() = sqrt (this.MagnitudeSq())
        member inline this.Normalize() =
            let mSq = this.MagnitudeSq()
            if mSq < EPSILON_X2 then
                Vector3.Zero
            else
                let m = sqrt mSq
                this / m

    
    let GRAVITY = Vector3(0.0, 0.0, -9.8)

    [<Struct; IsReadOnly; StructLayout(LayoutKind.Explicit, Size = 72)>]
    type Matrix3x3 =
        [<FieldOffset(0)>]
        val R0: Vector3

        [<FieldOffset(24)>]
        val R1: Vector3

        [<FieldOffset(48)>]
        val R2: Vector3

        new(r0, r1, r2) = { R0 = r0; R1 = r1; R2 = r2 }

        static member inline Identity =
            Matrix3x3(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0))

        member inline this.Transpose() =
            Matrix3x3(
                Vector3(this.R0.X, this.R1.X, this.R2.X),
                Vector3(this.R0.Y, this.R1.Y, this.R2.Y),
                Vector3(this.R0.Z, this.R1.Z, this.R2.Z)
            )

        member inline this.Transform(v) =
            Vector3(Vector3.Dot(this.R0, v), Vector3.Dot(this.R1, v), Vector3.Dot(this.R2, v))

        static member inline (*)(m: Matrix3x3, v) = m.Transform(v)

        static member inline (*)(m1: Matrix3x3, m2: Matrix3x3) =
            let r0x = m1.R0.X * m2.R0.X + m1.R0.Y * m2.R1.X + m1.R0.Z * m2.R2.X
            let r0y = m1.R0.X * m2.R0.Y + m1.R0.Y * m2.R1.Y + m1.R0.Z * m2.R2.Y
            let r0z = m1.R0.X * m2.R0.Z + m1.R0.Y * m2.R1.Z + m1.R0.Z * m2.R2.Z

            let r1x = m1.R1.X * m2.R0.X + m1.R1.Y * m2.R1.X + m1.R1.Z * m2.R2.X
            let r1y = m1.R1.X * m2.R0.Y + m1.R1.Y * m2.R1.Y + m1.R1.Z * m2.R2.Y
            let r1z = m1.R1.X * m2.R0.Z + m1.R1.Y * m2.R1.Z + m1.R1.Z * m2.R2.Z

            let r2x = m1.R2.X * m2.R0.X + m1.R2.Y * m2.R1.X + m1.R2.Z * m2.R2.X
            let r2y = m1.R2.X * m2.R0.Y + m1.R2.Y * m2.R1.Y + m1.R2.Z * m2.R2.Y
            let r2z = m1.R2.X * m2.R0.Z + m1.R2.Y * m2.R1.Z + m1.R2.Z * m2.R2.Z

            Matrix3x3(
                Vector3(r0x, r0y, r0z),
                Vector3(r1x, r1y, r1z),
                Vector3(r2x, r2y, r2z)
            )

        static member inline CreateRotation(axis: Vector3, angle: double) =
            if Double.IsNaN angle then
                Matrix3x3.Identity
            else
                if axis.MagnitudeSq() < EPSILON_X2 then
                    Matrix3x3.Identity
                else
                    let axis = axis.Normalize()
                    let x, y, z = axis.X, axis.Y, axis.Z
                    let c = Math.Cos angle
                    let s = Math.Sin angle
                    let omc = 1.0 - c

                    let xx_omc = x * x * omc
                    let yy_omc = y * y * omc
                    let zz_omc = z * z * omc
                    let xy_omc = x * y * omc
                    let xz_omc = x * z * omc
                    let yz_omc = y * z * omc

                    let xs = x * s
                    let ys = y * s
                    let zs = z * s

                    Matrix3x3(
                        Vector3(c + xx_omc, xy_omc - zs, xz_omc + ys),
                        Vector3(xy_omc + zs, c + yy_omc, yz_omc - xs),
                        Vector3(xz_omc - ys, yz_omc + xs, c + zz_omc)
                    )

    type Material =
        | Static = 0uy
        | GravityEnabled = 1uy
        | Liquid = 2uy

    [<RequireQualifiedAccess>]
    module Utils =
        let inline removeBySwapBack<'a when 'a :> IEquatable<'a>> (item: 'a) (list: PooledList<'a>)=
            let idx = list.Span.IndexOf item
            if idx > -1 then
                let lastIndex = list.Count - 1
                list[idx] <- list[lastIndex]
                list.RemoveAt lastIndex
                true
            else
                false

    [<Struct; IsReadOnly; StructLayout(LayoutKind.Explicit, Size = 16)>]
    type SubPrismCoords =
        new(q, r, z, s) = { Q = q; R = r; Z = z; SubIndex = s }

        [<FieldOffset(0)>]
        val Q: int

        [<FieldOffset(4)>]
        val R: int

        [<FieldOffset(8)>]
        val Z: int

        [<FieldOffset(12)>]
        val SubIndex: int

        override this.ToString() = $"SubPrismCoords({this.Q};{this.R};{this.Z};{this.SubIndex})"

        static member inline Create(q: int, r: int, z: int, i: int) = SubPrismCoords(q, r, z, i)

        static member inline Zero = SubPrismCoords(0, 0, 0, 0)

        static member inline Normalize(q: int, r: int, z: int, i: int) =
            let inline wrap (value: int) (max: int) = (value % max + max) % max
            SubPrismCoords(
                wrap q GRID_WIDTH_Q,
                wrap r GRID_DEPTH_R,
                Math.Clamp(z, 0, GRID_HEIGHT_Z - 1),
                i)

    [<RequireQualifiedAccess>]
    module SubPrismKey =
        [<Literal>]
        let private Z_SHIFT = 4
        [<Literal>]
        let private R_SHIFT = 12
        [<Literal>]
        let private Q_SHIFT = 28

        [<Literal>]
        let private Q_MASK = 0xFFFFUL
        [<Literal>]
        let private R_MASK = 0xFFFFUL
        [<Literal>]
        let private Z_MASK = 0xFFUL
        [<Literal>]
        let private S_MASK = 0xFUL

        let inline pack (coords: SubPrismCoords) =
            let q_part = (uint64 coords.Q &&& Q_MASK) <<< Q_SHIFT
            let r_part = (uint64 coords.R &&& R_MASK) <<< R_SHIFT
            let z_part = (uint64 coords.Z &&& Z_MASK) <<< Z_SHIFT
            let s_part = (uint64 coords.SubIndex &&& S_MASK)
            q_part ||| r_part ||| z_part ||| s_part

        let inline unpack (key: uint64) =
            let q = int32 ((key >>> Q_SHIFT) &&& Q_MASK)
            let r = int32 ((key >>> R_SHIFT) &&& R_MASK)
            let z = int32 ((key >>> Z_SHIFT) &&& Z_MASK)
            let s = int32 (key &&& S_MASK)
            SubPrismCoords(q, r, z, s)

    [<RequireQualifiedAccess>]
    module ContactKey =
        let inline key (id1: int) (id2: int) =
            // We guarantee the order so that (1,2) and (2,1) give one key, otherwise the legs are guaranteed to be shot off
            let minId = int64 (min id1 id2)
            let maxId = int64 (max id1 id2)
            (minId <<< 32) ||| maxId
            
        let inline unpack (key: int64) =
            let id1 = int (key >>> 32)
            let id2 = int (key &&& 0xFFFFFFFFL)
            struct(id1, id2)

    [<Struct; IsReadOnly; StructLayout(LayoutKind.Explicit, Size = 24)>]
    type CollisionResult =
        new(p) = { PenetrationVector = p }
        member inline this.AreColliding = this.PenetrationVector <> Vector3.Zero
        member inline this.Normalize() = 
            let originalPenetrationVector = this.PenetrationVector
            if originalPenetrationVector.MagnitudeSq() < EPSILON_X2 then
                struct(Vector3.Zero, 0.0)
            else
                let penetrationDepth = originalPenetrationVector.Magnitude()
                let finalNormal = originalPenetrationVector / penetrationDepth
                struct(finalNormal, penetrationDepth)
        
        [<FieldOffset(0)>]
        val PenetrationVector: Vector3

    [<RequireQualifiedAccess>]
    module WorldLimits =
        let X = HEX_RADIUS * Math.Sqrt(3.0) * double GRID_WIDTH_Q
        let Y = HEX_RADIUS * 1.5 * double GRID_DEPTH_R
        let _halfX = X / 2.0
        let _halfY = Y / 2.0
            
        let inline private wrap value range =
            if value >= 0.0 && value < range then
                value
            else
                let result = value % range
                if result < 0.0 then result + range else result
                
        let inline wrapX value = wrap value X
       
        let inline wrapY value = wrap value Y
        
        let inline wrapPosition(p: byref<Vector3>) =
            p.X <- wrapX p.X
            p.Y <- wrapY p.Y
        
        let inline relativeX value =
            if value > _halfX then value - X
            elif value < -_halfX then value + X
            else value
            
        let inline relativeY value =
            if value > _halfY then value - Y
            elif value < -_halfY then value + Y
            else value
            
        let inline relative (delta: byref<Vector3>) =
            delta.X <- relativeX delta.X
            delta.Y <- relativeY delta.Y
    
    [<RequireQualifiedAccess>]
    module Collision =

        let getAABB (position: Vector3) (dimensions: Vector3) (orientation: Matrix3x3) =
            
            let h = dimensions / 2.0
            let corners : Span<Vector3> = Stack.alloc 8
            corners[0] <- Vector3(-h.X, -h.Y, -h.Z)
            corners[1] <- Vector3(h.X, -h.Y, -h.Z)
            corners[2] <- Vector3(h.X, h.Y, -h.Z)
            corners[3] <- Vector3(-h.X, h.Y, -h.Z)
            corners[4] <- Vector3(-h.X, -h.Y, h.Z)
            corners[5] <- Vector3(h.X, -h.Y, h.Z)
            corners[6] <- Vector3(h.X, h.Y, h.Z)
            corners[7] <- Vector3(-h.X, h.Y, h.Z)

            let firstWorldCorner = position + orientation * corners[0]
            let mutable minCorner = firstWorldCorner
            let mutable maxCorner = firstWorldCorner

            for i = 1 to 7 do
                let worldCorner = position + orientation * corners[i]
                minCorner.X <- min minCorner.X worldCorner.X
                minCorner.Y <- min minCorner.Y worldCorner.Y
                minCorner.Z <- min minCorner.Z worldCorner.Z
                maxCorner.X <- max maxCorner.X worldCorner.X
                maxCorner.Y <- max maxCorner.Y worldCorner.Y
                maxCorner.Z <- max maxCorner.Z worldCorner.Z
            
            struct(minCorner, maxCorner)
        
        let inline checkCollisionAABB(minA: Vector3) (maxA: Vector3) (minB: Vector3) (maxB: Vector3) =

            let inline axisOverlaps minValA maxValA minValB maxValB worldSize =
                let centerA = (minValA + maxValA) * 0.5
                let extentA = (maxValA - minValA) * 0.5
                let centerB = (minValB + maxValB) * 0.5
                let extentB = (maxValB - minValB) * 0.5

                let mutable delta = centerA - centerB
                let halfWorldSize = worldSize * 0.5

                if delta > halfWorldSize then delta <- delta - worldSize
                elif delta < -halfWorldSize then delta <- delta + worldSize

                abs(delta) <= (extentA + extentB)

            let zOverlaps = maxA.Z >= minB.Z && minA.Z <= maxB.Z

            if not <| zOverlaps then
                false
            else
                let xOverlaps = axisOverlaps minA.X maxA.X minB.X maxB.X WorldLimits.X
                if not <| xOverlaps then
                    false
                else
                    axisOverlaps minA.Y maxA.Y minB.Y maxB.Y WorldLimits.Y

        let checkCollisionSATWithCachedAxis
            (p1: Vector3)
            (d1: Vector3)
            (o1: Matrix3x3)
            (p2: Vector3)
            (d2: Vector3)
            (o2: Matrix3x3)
            (cachedAxisIndex: int) =
            let inline getAxisByIndex index =
                match index with
                | 0 -> o1.R0
                | 1 -> o1.R1
                | 2 -> o1.R2
                | 3 -> o2.R0
                | 4 -> o2.R1
                | 5 -> o2.R2
                | 6 -> Vector3.Cross(o1.R0, o2.R0)
                | 7 -> Vector3.Cross(o1.R0, o2.R1)
                | 8 -> Vector3.Cross(o1.R0, o2.R2)
                | 9 -> Vector3.Cross(o1.R1, o2.R0)
                | 10 -> Vector3.Cross(o1.R1, o2.R1)
                | 11 -> Vector3.Cross(o1.R1, o2.R2)
                | 12 -> Vector3.Cross(o1.R2, o2.R0)
                | 13 -> Vector3.Cross(o1.R2, o2.R1)
                | 14 -> Vector3.Cross(o1.R2, o2.R2)
                | _ -> Vector3.Zero
            
            let h1 = d1 / 2.0
            let h2 = d2 / 2.0

            let mutable delta = p1 - p2
            WorldLimits.relative &delta
            
            let mutable minPenetration = Double.MaxValue
            let mutable mtvAxis = Vector3.Zero
            let mutable colliding = true
            let mutable winningAxisIndex = -1

            let inline testAxis (axis: Vector3) (axisIndex: int) =
                if axis.MagnitudeSq() > EPSILON then
                    let radius1 =
                        abs (Vector3.Dot(o1.R0 * h1.X, axis))
                        + abs (Vector3.Dot(o1.R1 * h1.Y, axis))
                        + abs (Vector3.Dot(o1.R2 * h1.Z, axis))
            
                    let radius2 =
                        abs (Vector3.Dot(o2.R0 * h2.X, axis))
                        + abs (Vector3.Dot(o2.R1 * h2.Y, axis))
                        + abs (Vector3.Dot(o2.R2 * h2.Z, axis))

                    let p1CenterProj = Vector3.Dot(delta, axis)

                    let p1Min = p1CenterProj - radius1
                    let p1Max = p1CenterProj + radius1
                    let p2Min = -radius2
                    let p2Max = radius2

                    let overlap = (min p1Max p2Max) - (max p1Min p2Min)

                    if overlap < 0.0 then
                        colliding <- false
                        winningAxisIndex <- axisIndex 
                    else
                        if overlap < minPenetration then
                            minPenetration <- overlap
                            mtvAxis <- axis
                            winningAxisIndex <- axisIndex

            if cachedAxisIndex <> -1 then
                let cachedAxis = getAxisByIndex cachedAxisIndex
                testAxis cachedAxis cachedAxisIndex

            if not <| colliding then
                struct(CollisionResult(Vector3.Zero), cachedAxisIndex)
            else
                testAxis o1.R0 0
                
                if colliding then testAxis o1.R1 1
                if colliding then testAxis o1.R2 2
                if colliding then testAxis o2.R0 3
                if colliding then testAxis o2.R1 4
                if colliding then testAxis o2.R2 5

                let inline testCrossProduct(axisA: Vector3) (axisB: Vector3) (axisIndex: int) =
                    if colliding && abs(Vector3.Dot(axisA, axisB)) < 1.0 - EPSILON then
                        let crossAxis = Vector3.Cross(axisA, axisB)
                        testAxis (crossAxis.Normalize()) axisIndex
                
                if colliding then
                    testCrossProduct o1.R0 o2.R0 6
                    testCrossProduct o1.R0 o2.R1 7
                    testCrossProduct o1.R0 o2.R2 8
                    testCrossProduct o1.R1 o2.R0 9
                    testCrossProduct o1.R1 o2.R1 10
                    testCrossProduct o1.R1 o2.R2 11
                    testCrossProduct o1.R2 o2.R0 12
                    testCrossProduct o1.R2 o2.R1 13
                    testCrossProduct o1.R2 o2.R2 14

                if colliding then
                    let mutable normal = mtvAxis.Normalize()

                    if Vector3.Dot(normal, delta) < 0.0 then
                        normal <- -normal
                        
                    let penetrationVector = normal * minPenetration
                    struct(CollisionResult(penetrationVector), winningAxisIndex)
                else
                    struct(CollisionResult(Vector3.Zero), winningAxisIndex)
                    
        let inline checkCollisionSAT (p1: Vector3) (d1: Vector3) (o1: Matrix3x3) (p2: Vector3) (d2: Vector3) (o2: Matrix3x3) =
            let struct(result, _) = checkCollisionSATWithCachedAxis p1 d1 o1 p2 d2 o2 -1
            result
    
    
    [<RequireQualifiedAccess>]
    module Body =
        type BodyType =
            | Generic = 0uy
            | SolidPrism = 1uy
            | Tree = 2uy
            | Bush = 3uy

        [<Struct; StructLayout(LayoutKind.Sequential)>]
        type T =
            new(
                id,
                bodyType,
                mass,
                dimensions : Vector3,
                orientation,
                position,
                velocity,
                supportCoords,
                friction) =
                {
                    Id = id
                    BodyType = bodyType
                    Mass = mass
                    InvMass = if mass <= EPSILON then 0.0 else 1.0 / mass
                    _dimensions =
                        Vector3(
                            min MAX_DIMENSION (max dimensions.X MIN_DIMENSION_THRESHOLD),
                            min MAX_DIMENSION (max dimensions.Y MIN_DIMENSION_THRESHOLD),
                            min MAX_DIMENSION (max dimensions.Z MIN_DIMENSION_THRESHOLD))
                    Orientation = orientation
                    Position = position
                    Velocity = velocity
                    SupportCoords = supportCoords
                    FrictionCoefficient = friction
                    IsFallingOver = false
                    FallRotationProgress = 0.0
                    FallDuration = 0.0
                    FallRotationAxis = Vector3.Zero
                    FallInitialOrientation = Matrix3x3.Identity
                    FallPivotPoint = Vector3.Zero
                    InitialCenterOffsetFromPivot = Vector3.Zero
                    IsForceFalling = false
                    IsSnappedToGrid = false
                    IsGravityEnabled = true
                    MinAABB = Vector3(Double.MaxValue, Double.MaxValue, Double.MaxValue)
                    MaxAABB = Vector3(Double.MinValue, Double.MinValue, Double.MinValue)
                }

            val Id: int
            val BodyType: BodyType
            val Mass: double
            val mutable InvMass: double
            val mutable Position: Vector3
            val mutable Velocity: Vector3
            val mutable private _dimensions: Vector3
            member this.Dimensions
                with get() = this._dimensions
                and set (value: Vector3) =
                    this._dimensions <-
                        Vector3(
                            min MAX_DIMENSION (max value.X MIN_DIMENSION_THRESHOLD),
                            min MAX_DIMENSION (max value.Y MIN_DIMENSION_THRESHOLD),
                            min MAX_DIMENSION (max value.Z MIN_DIMENSION_THRESHOLD))
                        
            val mutable Orientation: Matrix3x3
            val SupportCoords: SubPrismCoords
            val FrictionCoefficient: double
            val mutable IsFallingOver: bool
            val mutable FallRotationProgress: double
            val mutable FallDuration: double
            val mutable FallRotationAxis: Vector3
            val mutable FallInitialOrientation: Matrix3x3
            val mutable FallPivotPoint: Vector3
            val mutable InitialCenterOffsetFromPivot: Vector3
            val mutable IsForceFalling: bool
            val mutable IsSnappedToGrid : bool
            val mutable IsGravityEnabled : bool
            val mutable MinAABB: Vector3
            val mutable MaxAABB: Vector3
    
        let inline updateAABB (body: byref<T>) =
            let struct(minCorner, maxCorner) =
                Collision.getAABB
                    body.Position
                    body.Dimensions
                    body.Orientation
            body.MinAABB <- minCorner
            body.MaxAABB <- maxCorner
        
        type Repo =
            private
                {
                    _bodies : Dictionary<int, T>
                    mutable _currentBodyId: int
                }
        
        let nextId r = Interlocked.Increment &r._currentBodyId
        let getAll r  : IReadOnlyDictionary<_, _> = r._bodies 
        let createRepo() =
            {
                _bodies = Dictionary<_, _>()
                _currentBodyId = 0
            }
        
        let getRef id r = &CollectionsMarshal.GetValueRefOrNullRef(r._bodies, id)
        let getKeys r = r._bodies.Keys
        let tryAdd (body: inref<T>) r = r._bodies.TryAdd(body.Id, body)
        let remove id r = r._bodies.Remove id
    
    [<RequireQualifiedAccess>]
    module Grid =
        let private _precomputedPrismShapes =
            let precomputePrismShape (sector: int) =
                let inline calcRelativeHexVertex (index: int) =
                    let angle = double index * PI / 3.0
                    Vector3(HEX_RADIUS * cos angle, HEX_RADIUS * sin angle, 0.0)

                let nextSector = (sector + 1) % 6

                let v1_rel = Vector3.Zero
                let v2_rel = calcRelativeHexVertex sector
                let v3_rel = calcRelativeHexVertex nextSector

                let xAxis = ((v2_rel + v3_rel) / 2.0 - v1_rel).Normalize()
                let zAxis = Vector3.Up
                let yAxis = Vector3.Cross(zAxis, xAxis).Normalize()
                let orientation = Matrix3x3(xAxis, yAxis, zAxis)

                let baseCenter_rel = (v1_rel + v2_rel + v3_rel) / 3.0
                let invOrientation = orientation.Transpose()
                let localV1 = invOrientation * (v1_rel - baseCenter_rel)
                let localV2 = invOrientation * (v2_rel - baseCenter_rel)
                let localV3 = invOrientation * (v3_rel - baseCenter_rel)

                let minLocalX = min localV1.X (min localV2.X localV3.X)
                let maxLocalX = max localV1.X (max localV2.X localV3.X)
                let minLocalY = min localV1.Y (min localV2.Y localV3.Y)
                let maxLocalY = max localV1.Y (max localV2.Y localV3.Y)
                
                let hx = (maxLocalX - minLocalX) / 2.0
                let hy = (maxLocalY - minLocalY) / 2.0
                let hz = HEX_HEIGHT / 4.0

                let dimensions = Vector3(hx, hy, hz) * 2.0

                struct (orientation, dimensions)

            [| for i in 0..5 -> precomputePrismShape i |]
        
        let private _precomputedPrismAABBs =
            [|
                for i in 0..11 ->
                    let struct(orient, dims) = _precomputedPrismShapes[i % 6]
                    Collision.getAABB Vector3.Zero dims orient
            |]
            
        let private _vertexes =
            [|
                Vector3(HEX_RADIUS * cos (double 0 * PI / 3.0), HEX_RADIUS * sin (double 0 * PI / 3.0), 0.0)
                Vector3(HEX_RADIUS * cos (double 1 * PI / 3.0), HEX_RADIUS * sin (double 1 * PI / 3.0), 0.0)
                Vector3(HEX_RADIUS * cos (double 2 * PI / 3.0), HEX_RADIUS * sin (double 2 * PI / 3.0), 0.0)
                Vector3(HEX_RADIUS * cos (double 3 * PI / 3.0), HEX_RADIUS * sin (double 3 * PI / 3.0), 0.0)
                Vector3(HEX_RADIUS * cos (double 4 * PI / 3.0), HEX_RADIUS * sin (double 4 * PI / 3.0), 0.0)
                Vector3(HEX_RADIUS * cos (double 5 * PI / 3.0), HEX_RADIUS * sin (double 5 * PI / 3.0), 0.0)
            |]
    
        let inline convertWorldToFractionalAxial worldX worldY =
            let r_approx = worldY / (HEX_RADIUS * 1.5)
            let r = r_approx
            let q_approx = worldX / (HEX_RADIUS * SQRT3) - 0.5 * (double (int r_approx &&& 1))
            let q = q_approx

            struct (q, r)

        let inline cubeRound (fracX: double) (fracY: double) (fracZ: double) =
            let roundedX = Math.Round fracX
            let roundedY = Math.Round fracY
            let roundedZ = Math.Round fracZ

            let diffX = abs (roundedX - fracX)
            let diffY = abs (roundedY - fracY)
            let diffZ = abs (roundedZ - fracZ)

            // Rounding correction: the axis with the largest change due to rounding is
            // recalculated from the other two to preserve the x+y+z=0 invariant
            if diffX > diffY && diffX > diffZ then
                struct (-int roundedY - int roundedZ, int roundedZ)
            elif diffY > diffZ then
                struct (int roundedX, int roundedZ)
            else
                let finalR = -int roundedX - int roundedY
                struct (int roundedX, finalR)
         
        let inline convertHexToWorld q r z hexHeight =
            let x = HEX_RADIUS * SQRT3 * (double q + 0.5 * double (r &&& 1))
            let y = HEX_RADIUS * 1.5 * double r
            let zPos = (double z + 0.5) * hexHeight
            Vector3(x, y, zPos)

        let inline convertWorldToRawGridCoords (pos: Vector3) =
            let struct (fracQ, fracR) = convertWorldToFractionalAxial pos.X pos.Y
            let struct (initialGuessQ, initialGuessR) = cubeRound fracQ (-fracQ - fracR) fracR

            let guessHexCenter = convertHexToWorld initialGuessQ initialGuessR 0 0.0

            let mutable relativePos = pos - guessHexCenter
            WorldLimits.relative &relativePos
            
            let struct (fracQ_stable, fracR_stable) = convertWorldToFractionalAxial relativePos.X relativePos.Y
            let struct (offsetQ, offsetR) = cubeRound fracQ_stable (-fracQ_stable - fracR_stable) fracR_stable

            let finalQ = initialGuessQ + offsetQ    
            let finalR = initialGuessR + offsetR
            let finalZIndex = Math.Floor(pos.Z / HEX_HEIGHT) |> int

            struct(finalQ, finalR, finalZIndex)

        let convertWorldToSubPrismCoords (pos: Vector3)=
            let mutable pos = pos
            WorldLimits.wrapPosition &pos

            let struct (finalQ, finalR, finalZIndex) = convertWorldToRawGridCoords pos

            let rawHexCenter = convertHexToWorld finalQ finalR 0 0.0

            let mutable relativePos = pos - rawHexCenter
            WorldLimits.relative &relativePos

            let angle = Math.Atan2(relativePos.Y, relativePos.X)
            let normalizedAngle = if angle < 0.0 then angle + 2.0 * PI else angle
            let sector = int (Math.Floor(normalizedAngle / (PI / 3.0))) % 6
            let verticalLayer = if (pos.Z - (double finalZIndex + 0.5) * HEX_HEIGHT) < 0.0 then 0 else 1
            let finalSubIndex = verticalLayer * 6 + sector

            SubPrismCoords.Normalize(finalQ, finalR, finalZIndex, finalSubIndex)

        let getTriangularPrismCenter (coords: SubPrismCoords)=
            let hexCenter = convertHexToWorld coords.Q coords.R 0 0.0
            let sector = coords.SubIndex % 6

            let v1 = hexCenter
            let v2 = hexCenter + _vertexes[sector]
            let v3 = hexCenter + _vertexes[(sector + 1) % 6]

            let baseCenter = (v1 + v2 + v3) / 3.0
            let zPos = HEX_HEIGHT * (double coords.Z + (if coords.SubIndex < 6 then 0.25 else 0.75))

            Vector3(baseCenter.X, baseCenter.Y, zPos)

        let createPrismSpace (coords: SubPrismCoords) =
            let absolutePrismPos = coords |> getTriangularPrismCenter
            let struct (orientation, dimensions) = _precomputedPrismShapes[coords.SubIndex % 6]
            struct (absolutePrismPos, dimensions, orientation)

        let inline getPrismSpace coords  = coords |> createPrismSpace
        
        let inline getPrismSpaceByKey key  = key |> SubPrismKey.unpack |> getPrismSpace
        
        let inline private getTriangularPrismAABB (coords: SubPrismCoords) =
            let prismCenter = coords |> getTriangularPrismCenter
            let struct(minLocal, maxLocal) = _precomputedPrismAABBs[coords.SubIndex]
            struct(minLocal + prismCenter, maxLocal + prismCenter)

        let inline private fillGridFromCorners
            (corners: Span<Vector3>)
            (minAABB: Vector3)
            (maxAABB: Vector3)
            (p: Vector3)
            (d: Vector3)
            (o: Matrix3x3) 
            (filter: HashSet<uint64>) =
            let mutable rawMinQ, rawMaxQ = Int32.MaxValue, Int32.MinValue
            let mutable rawMinR, rawMaxR = Int32.MaxValue, Int32.MinValue
            let mutable rawMinZ, rawMaxZ = Int32.MaxValue, Int32.MinValue
            
            for i = 0 to corners.Length - 1 do
                let struct(rawQ, rawR, rawZ) = corners[i] |> convertWorldToRawGridCoords 
                rawMinQ <- min rawMinQ rawQ
                rawMaxQ <- max rawMaxQ rawQ
                rawMinR <- min rawMinR rawR
                rawMaxR <- max rawMaxR rawR
                rawMinZ <- min rawMinZ rawZ
                rawMaxZ <- max rawMaxZ rawZ

            for q = rawMinQ to rawMaxQ do
                for r = rawMinR to rawMaxR do
                    for z = rawMinZ to rawMaxZ do
                        let hexBottomZ = double z * HEX_HEIGHT
                        let hexTopZ = hexBottomZ + HEX_HEIGHT
                        if maxAABB.Z >= hexBottomZ && minAABB.Z <= hexTopZ then
                            let hexCenterXY = convertHexToWorld q r 0 0.0
                            let minHexAABB = Vector3(hexCenterXY.X - HEX_RADIUS, hexCenterXY.Y - HEX_RADIUS, hexBottomZ)
                            let maxHexAABB = Vector3(hexCenterXY.X + HEX_RADIUS, hexCenterXY.Y + HEX_RADIUS, hexTopZ)

                            if Collision.checkCollisionAABB minAABB maxAABB minHexAABB maxHexAABB then
                                let overlapsBottomHalf = maxAABB.Z >= hexBottomZ && minAABB.Z < (hexBottomZ + 0.5 * HEX_HEIGHT)
                                let overlapsTopHalf = maxAABB.Z >= (hexBottomZ + 0.5 * HEX_HEIGHT) && minAABB.Z < hexTopZ

                                let inline checkAndAddSubPrisms (startIndex: int) (endIndex: int) =
                                    for sub_idx = startIndex to endIndex do
                                        let subPrismCoords = SubPrismCoords.Normalize(q, r, z, sub_idx)
                                        let struct(minPrism, maxPrism) = subPrismCoords |> getTriangularPrismAABB
                                        if Collision.checkCollisionAABB minAABB maxAABB minPrism maxPrism then
                                            let struct(prismPos, prismDims, prismOrient) = subPrismCoords |> getPrismSpace
                                            let result =
                                                Collision.checkCollisionSAT
                                                    p
                                                    d
                                                    o
                                                    prismPos
                                                    prismDims
                                                    prismOrient
                                                        
                                            if result.AreColliding then
                                                let key = subPrismCoords |> SubPrismKey.pack
                                                filter.Add key |> ignore
                                
                                if overlapsBottomHalf then
                                    checkAndAddSubPrisms 0 5
                                if overlapsTopHalf then
                                    checkAndAddSubPrisms 6 11

        let inline private fillForShift
            (shift: Vector3)
            (p: Vector3)
            (d: Vector3)
            (o: Matrix3x3)
            (worldCorners: Span<Vector3>)
            (shiftBuffer: Span<Vector3>)
            (filter: HashSet<uint64>) =
            
            shiftBuffer[0] <- worldCorners[0] + shift
            shiftBuffer[1] <- worldCorners[1] + shift
            shiftBuffer[2] <- worldCorners[2] + shift
            shiftBuffer[3] <- worldCorners[3] + shift
            shiftBuffer[4] <- worldCorners[4] + shift
            shiftBuffer[5] <- worldCorners[5] + shift
            shiftBuffer[6] <- worldCorners[6] + shift
            shiftBuffer[7] <- worldCorners[7] + shift
                
            let shiftedPos = p + shift
            let struct(shiftedMin, shiftedMax) = Collision.getAABB shiftedPos d o

            fillGridFromCorners shiftBuffer shiftedMin shiftedMax shiftedPos d o filter
            
        let fillOverlappingSubPrismsAABB
            (p: Vector3)
            (d: Vector3)
            (o: Matrix3x3)
            (filter: HashSet<uint64>)
            (buffer: PooledList<uint64>) =
            
            filter.Clear()
            buffer.Clear()

            let h = d / 2.0
            let worldCorners : Span<Vector3> = Stack.alloc 8
            worldCorners[0] <- p + o * Vector3(-h.X, -h.Y, -h.Z)
            worldCorners[1] <- p + o * Vector3(h.X, -h.Y, -h.Z)
            worldCorners[2] <- p + o * Vector3(h.X, h.Y, -h.Z)
            worldCorners[3] <- p + o * Vector3(-h.X, h.Y, -h.Z)
            worldCorners[4] <- p + o * Vector3(-h.X, -h.Y, h.Z)
            worldCorners[5] <- p + o * Vector3(h.X, -h.Y, h.Z)
            worldCorners[6] <- p + o * Vector3(h.X, h.Y, h.Z)
            worldCorners[7] <- p + o * Vector3(-h.X, h.Y, h.Z)
                
            let struct(minAABB, maxAABB) = Collision.getAABB p d o
            
            fillGridFromCorners
                worldCorners
                minAABB
                maxAABB
                p
                d
                o
                filter

            let margin = (max d.X d.Y) * 1.5
            let nearMinX = minAABB.X < margin
            let nearMaxX = maxAABB.X > (WorldLimits.X - margin)
            let nearMinY = minAABB.Y < margin
            let nearMaxY = maxAABB.Y > (WorldLimits.Y - margin)

            if nearMinX || nearMaxX || nearMinY || nearMaxY then
                let shiftBuffer : Span<Vector3> = Stack.alloc 8
                if nearMinX then fillForShift (Vector3(WorldLimits.X, 0.0, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMaxX then fillForShift (Vector3(-WorldLimits.X, 0.0, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMinY then fillForShift (Vector3(0.0, WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMaxY then fillForShift (Vector3(0.0, -WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMinX && nearMinY then fillForShift (Vector3(WorldLimits.X, WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMaxX && nearMinY then fillForShift (Vector3(-WorldLimits.X, WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMinX && nearMaxY then fillForShift (Vector3(WorldLimits.X, -WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMaxX && nearMaxY then fillForShift (Vector3(-WorldLimits.X, -WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter

            buffer.AddRange filter
    
    [<RequireQualifiedAccess>]
    module Geometry =
        type Repo =
            private
                {
                    _logger : ILogger
                    _prisms : Dictionary<uint64, Material>
                    mutable _idGenerator: int
                }
            
            member this.Prisms : IReadOnlyDictionary<_, _> = this._prisms
        
        let createRepo() =
            {
                _logger = Log.ForContext<Repo>()
                _prisms = Dictionary()
                _idGenerator = -1
            }
            
        let removePrism key r = r._prisms.Remove key
        
        let addPrism(coords: SubPrismCoords)  (material: Material) r =
            let coords = coords |> SubPrismKey.pack
            let mutable isFound = false
            let newPrism = &CollectionsMarshal.GetValueRefOrAddDefault (r._prisms, coords, &isFound)
            newPrism <- material

        let isSolid coords r = r._prisms.ContainsKey coords

        let inline hasSupportBeneath (point: Vector3) r =
            let checkPos = point - Vector3(0.0, 0.0, 0.01)
            if checkPos.Z < EPSILON then true
            else
                let supportCoords = checkPos |> Grid.convertWorldToSubPrismCoords |> SubPrismKey.pack
                r |> isSolid supportCoords

        let updatePhysics(fallingPrisms: PooledList<Body.T>) (prismsToRemove: PooledList<uint64>) r=
            for kvp in r._prisms do
                if kvp.Value = Material.GravityEnabled then
                    let coordsKey = kvp.Key
                    let coords = coordsKey |> SubPrismKey.unpack
                    let pos = coords |> Grid.getTriangularPrismCenter 
                    let checkPos = pos - Vector3(0.0, 0.0, HEX_HEIGHT / 2.0 + 0.01)

                    let supportCoords = checkPos |> Grid.convertWorldToSubPrismCoords |> SubPrismKey.pack

                    if not <| (r |> isSolid supportCoords) then
                        let struct (position, dimension, orientation) = coords |> Grid.createPrismSpace
                        let body =
                            Body.T(
                                r._idGenerator,
                                Body.BodyType.SolidPrism,
                                10000.0, 
                                dimension,
                                orientation,
                                position,
                                Vector3.Zero,
                                coords,
                                0.5)

                        r._idGenerator <- r._idGenerator - 1

                        fallingPrisms.Add body

                        prismsToRemove.Add coordsKey

    [<RequireQualifiedAccess>]
    module Flora =
        
        [<Struct; StructLayout(LayoutKind.Sequential)>]
        type T =
            {
                Id: int
                BodyType: Body.BodyType
                Mass: double
                SupportCoordsKey: uint64
                FrictionCoefficient: double
                BreakThreshold: double
                mutable Dimensions: Vector3
                mutable Position: Vector3
                mutable IsDestroyed: bool
                mutable FramesWithoutSupport: int
            }
    
        let inline createBodyFromFlora id (flora: T) =
            Body.T(
                id,
                flora.BodyType,
                flora.Mass,
                flora.Dimensions,
                Matrix3x3.Identity,
                flora.Position,
                Vector3.Zero,
                flora.SupportCoordsKey |> SubPrismKey.unpack,
                flora.FrictionCoefficient
            )
        
        let inline create id bType mass pos h t sup friction breakThreshold =
            {
                Id = id
                BodyType = bType
                Mass = mass
                Dimensions = Vector3(t, t, h)
                Position = pos
                SupportCoordsKey = sup |> SubPrismKey.pack
                FrictionCoefficient = friction
                BreakThreshold = breakThreshold
                IsDestroyed = false
                FramesWithoutSupport = 0
            }

        type Repo =
            private
                {
                    _geometryRepo: Geometry.Repo
                    _trees: Dictionary<int, T>
                    _bushes : Dictionary<int, T>
                    _treeHash : Dictionary<uint64, PooledList<int>>
                    _bushHash : Dictionary<uint64, PooledList<int>>
                    _filterBuffer: HashSet<uint64>
                    _buffer: PooledList<uint64>
                }
            member this.AllTrees : IReadOnlyDictionary<_, _> = this._trees
            member this.AllBushes : IReadOnlyDictionary<_, _> = this._bushes
            member this.Dispose() =
                this._buffer |> Dispose.action
            interface IDisposable with
                member this.Dispose() = this.Dispose()
        
        let createRepo geometryRepo =
            {
                _geometryRepo = geometryRepo
                _trees = Dictionary()
                _bushes = Dictionary()
                _treeHash = Dictionary()
                _bushHash = Dictionary()
                _filterBuffer = HashSet()
                _buffer = new PooledList<_>()
            }
            
        let private fillPrismCoords (flora: T) r =
            Grid.fillOverlappingSubPrismsAABB
                flora.Position
                flora.Dimensions
                Matrix3x3.Identity
                r._filterBuffer
                r._buffer
        
        let addTree(flora: T) r = r._trees.Add(flora.Id, flora)
        let addBush(flora: T) r = r._bushes.Add(flora.Id, flora)
        let removeTree id r = r._trees.Remove id |> ignore
        let removeBush id r = r._bushes.Remove id |> ignore

        let init r =
            r._treeHash.Clear()
            r._bushHash.Clear()

            for kvp in r._trees do
                let floraId = kvp.Key
                let floraData = kvp.Value
                r |> fillPrismCoords floraData

                for prismCoords in r._buffer.Span do
                    match r._treeHash.TryGetValue prismCoords with
                    | true, idList -> idList.Add floraId
                    | false, _ ->
                        let newList = new PooledList<int>()
                        newList.Add floraId
                        r._treeHash.Add(prismCoords, newList)

            for kvp in r._bushes do
                let floraId = kvp.Key
                let floraData = kvp.Value
                r |> fillPrismCoords floraData

                for prismCoords in r._buffer.Span do
                    match r._bushHash.TryGetValue prismCoords with
                    | true, idList -> idList.Add floraId
                    | false, _ ->
                        let newList = new PooledList<int>()
                        newList.Add floraId
                        r._bushHash.Add(prismCoords, newList)

        let addToHash(flora: T) r =

            let hash = if flora.BodyType = Body.BodyType.Tree then r._treeHash else r._bushHash

            r |> fillPrismCoords flora

            for prismCoords in r._buffer.Span do
                match hash.TryGetValue prismCoords with
                | true, idList ->
                    if not <| idList.Contains flora.Id then
                        idList.Add flora.Id
                | false, _ ->
                    let newList = new PooledList<int>()
                    newList.Add flora.Id
                    hash.Add(prismCoords, newList)
                 
        let updateHashesForRemovedFlora(removedFloraIds: PooledList<int>) r =

            for floraId in removedFloraIds.Span do
                let floraData =
                    if r._trees.ContainsKey floraId then
                        ValueSome r._trees[floraId]
                    elif r._bushes.ContainsKey floraId then
                        ValueSome r._bushes[floraId]
                    else
                        ValueNone

                match floraData with
                | ValueSome data ->
                    let hash = if data.BodyType = Body.BodyType.Tree then r._treeHash else r._bushHash
                    r |> fillPrismCoords data

                    for prismCoords in r._buffer.Span do
                        match hash.TryGetValue prismCoords with
                        | true, idList ->
                            idList |> Utils.removeBySwapBack floraId |> ignore

                            if idList.Count = 0 then
                                hash.Remove prismCoords |> ignore
                                idList |> Dispose.action
                        | false, _ -> ()
                | ValueNone -> ()

        let growFlora floraId (newDimensions: Vector3) r=
            let treeData = &CollectionsMarshal.GetValueRefOrNullRef(r._trees, floraId)

            let floraDataRef =
                if Unsafe.IsNullRef &treeData then
                    &CollectionsMarshal.GetValueRefOrNullRef(r._bushes, floraId)
                else
                    &treeData

            match Unsafe.IsNullRef &floraDataRef with
            | true -> ()
            | false ->
                let oldDimensions = floraDataRef.Dimensions
                let oldHeight = oldDimensions.Z
                let newHeight = newDimensions.Z

                r |> fillPrismCoords({ floraDataRef with Dimensions = oldDimensions })
                use oldOccupiedCells = new PooledList<uint64>()
                oldOccupiedCells.AddRange r._buffer

                floraDataRef.Dimensions <- newDimensions
                let heightChange = newHeight - oldHeight
                floraDataRef.Position <- floraDataRef.Position + Vector3(0.0, 0.0, heightChange / 2.0)
                floraDataRef.Position.X <- WorldLimits.wrapX floraDataRef.Position.X
                floraDataRef.Position.Y <- WorldLimits.wrapY floraDataRef.Position.Y

                r |> fillPrismCoords floraDataRef
                use newOccupiedCells = new PooledList<uint64>()
                newOccupiedCells.AddRange r._buffer

                let hash =
                    if floraDataRef.BodyType = Body.BodyType.Tree then
                        r._treeHash
                    else
                        r._bushHash

                use newCellsSet = new PooledSet<uint64>(newOccupiedCells.Span)

                for oldCell in oldOccupiedCells.Span do

                    if not <| newCellsSet.Remove oldCell then

                        match hash.TryGetValue oldCell with
                        | true, idList ->
                            idList |> Utils.removeBySwapBack floraId |> ignore

                            if idList.Count = 0 then
                                hash.Remove oldCell |> ignore
                                idList |> Dispose.action
                        | false, _ -> ()

                for newCell in newCellsSet do
                    match hash.TryGetValue newCell with
                    | true, idList ->
                        if not <| idList.Contains floraId then
                            idList.Add floraId
                    | false, _ ->
                        let newList = new PooledList<int>()
                        newList.Add floraId
                        hash.Add(newCell, newList)

        let tryGetTreesInCell coords r = r._treeHash.TryGetValue coords

        let tryGetBushesInCell coords r= r._bushHash.TryGetValue coords
        let tryGetTreeDataRef id r = &CollectionsMarshal.GetValueRefOrNullRef(r._trees, id)

        let applyBushFriction(dynamicBody: byref<Body.T>) bushId dt r =
            if r._bushes.ContainsKey bushId then
                let bushData = r._bushes[bushId]
                let result =
                    Collision.checkCollisionSAT
                        bushData.Position
                        bushData.Dimensions
                        dynamicBody.Orientation
                        bushData.Position
                        bushData.Dimensions
                        Matrix3x3.Identity

                if result.AreColliding then
                    let velocity = dynamicBody.Velocity

                    if velocity.MagnitudeSq() > EPSILON then
                        let frictionFactor = 0.5
                        let thickness = bushData.Dimensions.X
                        let height = bushData.Dimensions.Z
                        let k = frictionFactor * thickness * height
                        let dampingFactor =
                            if dynamicBody.InvMass > EPSILON then
                                1.0 - (k * dt * dynamicBody.InvMass)
                            else
                                1.0

                        dynamicBody.Velocity <- velocity * (max 0.0 (min 1.0 dampingFactor))

        let updatePhysics(unrootedFlora: PooledList<T>) (floraToRemoveIds: PooledList<int>) r =
            let updateCollection (flora: Dictionary<int, T>) =
                use currentKeys = new PooledList<int>(flora.Keys)

                for id in currentKeys.Span do
                    let floraData = &CollectionsMarshal.GetValueRefOrNullRef(flora, id)
                    if not <| Unsafe.IsNullRef &floraData && not floraData.IsDestroyed then
                        let hasSupport = r._geometryRepo |> Geometry.isSolid floraData.SupportCoordsKey
                        if hasSupport then
                            floraData.FramesWithoutSupport <- 0
                        else
                            floraData.FramesWithoutSupport <- floraData.FramesWithoutSupport + 1

                        if floraData.Mass > EPSILON && floraData.FramesWithoutSupport >= FRAMES_TO_UNROOT then
                            unrootedFlora.Add(floraData)
                            floraToRemoveIds.Add(id)

            updateCollection r._trees
            updateCollection r._bushes

    [<Struct>]
    type Buffers =
        {
            UniquePrismsFilterBuffer: HashSet<uint64>
            UniquePrismsBuffer: PooledList<uint64>
            UnrootedFlora: PooledList<Flora.T>
            FallingPrisms: PooledList<Body.T>
            SnapPointsBuffer: PooledList<Vector3>
            CollisionCheckedBodyPairs : HashSet<int64>
            CollisionIslandPairs: HashSet<int64>
            CollisionActiveIslandIds: PooledList<int>
            CollisionSleepingIslandIds: PooledList<int>
            CollisionDestroyedFlora: HashSet<int>
            CollisionProcessedStatic: HashSet<uint64>
            CollisionProcessedFlora: HashSet<int>

            BodiesToAdd: PooledList<Body.T>
            BodiesToRemoveIds: PooledList<int>
            FloraToRemoveIds: PooledList<int>
            PrismsToRemoveCoords: PooledList<uint64>
        } 
        member this.Dispose() =
            this.UniquePrismsBuffer |> Dispose.action
            this.UnrootedFlora |> Dispose.action
            this.FallingPrisms |> Dispose.action
            this.SnapPointsBuffer |> Dispose.action
            
            this.CollisionActiveIslandIds |> Dispose.action
            this.CollisionSleepingIslandIds |> Dispose.action
            
            this.BodiesToAdd |> Dispose.action
            this.BodiesToRemoveIds |> Dispose.action
            this.FloraToRemoveIds |> Dispose.action
            this.PrismsToRemoveCoords |> Dispose.action
        interface IDisposable with
            member this.Dispose() = this.Dispose()
            
        member this.Clear() =
            this.UniquePrismsFilterBuffer.Clear()
            this.UniquePrismsBuffer.Clear()
            this.UnrootedFlora.Clear()
            this.FallingPrisms.Clear()
            this.SnapPointsBuffer.Clear()
            
            this.CollisionCheckedBodyPairs.Clear()
            this.CollisionIslandPairs.Clear()
            this.CollisionActiveIslandIds.Clear()
            this.CollisionSleepingIslandIds.Clear()
            this.CollisionDestroyedFlora.Clear()
            this.CollisionProcessedStatic.Clear()
            this.CollisionProcessedFlora.Clear()

            this.BodiesToAdd.Clear()
            this.BodiesToRemoveIds.Clear()
            this.FloraToRemoveIds.Clear()
            this.PrismsToRemoveCoords.Clear()

        static member Create() =
            {
                UniquePrismsFilterBuffer = HashSet()
                UniquePrismsBuffer = new PooledList<_>()
                UnrootedFlora = new PooledList<_>()
                FallingPrisms = new PooledList<_>()
                SnapPointsBuffer = new PooledList<_>()

                CollisionCheckedBodyPairs = HashSet()
                CollisionActiveIslandIds = new PooledList<_>()
                CollisionSleepingIslandIds = new PooledList<_>()
                CollisionIslandPairs = HashSet()
                CollisionDestroyedFlora = HashSet()
                CollisionProcessedStatic = HashSet()
                CollisionProcessedFlora = HashSet()

                BodiesToAdd = new PooledList<_>()
                BodiesToRemoveIds = new PooledList<_>()
                FloraToRemoveIds = new PooledList<_>()
                PrismsToRemoveCoords = new PooledList<_>()
            }

    [<RequireQualifiedAccess>]
    module SpatialHash =
        
        [<Struct; IsReadOnly>]
        type BodyCacheEntry =
            val Position: Vector3
            val Dimensions: Vector3
            val Orientation: Matrix3x3
            val OccupiedCells: PooledList<uint64>
            new(body: inref<Body.T>, cells) = 
                { 
                    Position = body.Position; 
                    Dimensions = body.Dimensions;
                    Orientation = body.Orientation;
                    OccupiedCells = cells 
                }
        
        let private areTransformsEqual (b1: inref<Body.T>) (b2: inref<BodyCacheEntry>) =
            let inline areVectorsClose (v1: Vector3) (v2: Vector3) =
                abs(v1.X - v2.X) < EPSILON
                && abs(v1.Y - v2.Y) < EPSILON
                && abs(v1.Z - v2.Z) < EPSILON

            let inline areMatricesClose (m1: Matrix3x3) (m2: Matrix3x3) =
                areVectorsClose m1.R0 m2.R0
                && areVectorsClose m1.R1 m2.R1
                && areVectorsClose m1.R2 m2.R2

            areVectorsClose b1.Position b2.Position
            && areVectorsClose b1.Dimensions b2.Dimensions
            && areMatricesClose b1.Orientation b2.Orientation
        
        type Repo =
            private
                {
                    _logger : ILogger
                    _hash : Dictionary<uint64, PooledList<int>>
                    _bodyStateCache : Dictionary<int, BodyCacheEntry>
                }
            member this.Dispose() =
                this._hash.Values |> Seq.iter Dispose.action
                this._bodyStateCache.Values |> Seq.iter(fun entry -> entry.OccupiedCells |> Dispose.action)
            interface IDisposable with
                member this.Dispose() = this.Dispose()
                
        let createRepo() =
            {
                _logger = Log.ForContext<Repo>()
                _hash = Dictionary<_, _>()
                _bodyStateCache = Dictionary<_, _>()
            }
        
        let getOccupiedCells bodyId r : ReadOnlySpan<uint64> =
            match r._bodyStateCache.TryGetValue bodyId with
            | true, entry -> entry.OccupiedCells.Span
            | false, _ -> ReadOnlySpan<uint64>.Empty

        let inline private addBodyToCell bodyId cellKey r =
            match r._hash.TryGetValue cellKey with
            | true, idList -> 
                if not <| idList.Contains bodyId then
                    idList.Add bodyId
            | false, _ ->
                let newList = new PooledList<int>()
                newList.Add bodyId
                r._hash.Add(cellKey, newList)

        let inline private removeBodyFromCell bodyId cellKey r =
            match r._hash.TryGetValue cellKey with
            | true, idList ->
                if idList |> Utils.removeBySwapBack bodyId && idList.Count = 0 then
                    r._hash.Remove cellKey |> ignore
                    idList |> Dispose.action
            | false, _ -> ()

        let clear r =
            r._hash.Values |> Seq.iter Dispose.action
            r._bodyStateCache.Values |> Seq.iter(fun entry -> entry.OccupiedCells |> Dispose.action)
           
            r._hash.Clear()
            r._bodyStateCache.Clear()

        let query cellKey r =
            match r._hash.TryGetValue cellKey with
            | true, idList -> idList.Span
            | false, _ -> Span<int>.Empty

        let add(body: inref<Body.T>) buffers r =
            if r._bodyStateCache.ContainsKey body.Id then
                r._logger.Warning("Body {BodyId} was already added into cache", body.Id)
            else
                let occupiedCells = new PooledList<uint64>()
                Grid.fillOverlappingSubPrismsAABB
                    body.Position
                    body.Dimensions
                    body.Orientation
                    buffers.UniquePrismsFilterBuffer
                    occupiedCells
                
                for cellKey in occupiedCells.Span do
                    r |> addBodyToCell body.Id cellKey

                let newEntry = BodyCacheEntry(&body, occupiedCells)
                r._bodyStateCache.Add(body.Id, newEntry)

        let remove bodyId r =
            match r._bodyStateCache.TryGetValue bodyId with
            | true, entry ->
                for cellKey in entry.OccupiedCells.Span do
                    r |> removeBodyFromCell bodyId cellKey
                
                entry.OccupiedCells |> Dispose.action
                r._bodyStateCache.Remove bodyId |> ignore
            | false, _ -> ()

        let update (body: inref<Body.T>) buffers r=
            let cachedEntry = &CollectionsMarshal.GetValueRefOrNullRef(r._bodyStateCache, body.Id)
            match Unsafe.IsNullRef &cachedEntry with
            | true -> add &body buffers r
            | false ->
                if not <| areTransformsEqual &body &cachedEntry then
                    let newOccupiedCells = new PooledList<uint64>()
                    Grid.fillOverlappingSubPrismsAABB
                        body.Position
                        body.Dimensions
                        body.Orientation
                        buffers.UniquePrismsFilterBuffer
                        newOccupiedCells
                    
                    let oldOccupiedCells = cachedEntry.OccupiedCells
                    use newCellsSet = new PooledSet<uint64>(newOccupiedCells.Span)

                    for oldCell in oldOccupiedCells.Span do
                        if not <| newCellsSet.Remove oldCell then
                            r |> removeBodyFromCell body.Id oldCell

                    for newCell in newCellsSet do
                        r |> addBodyToCell body.Id newCell

                    oldOccupiedCells |> Dispose.action
                    
                    cachedEntry <- BodyCacheEntry(&body, newOccupiedCells)
    
    [<RequireQualifiedAccess>]    
    module SnapGrid =
        let private generateLayerPoints (basePos: Vector3) (layerZ: double) (points: PooledList<Vector3>) =
            let hexCenter = Vector3(basePos.X, basePos.Y, layerZ)
            points.Add hexCenter
            let inline calcHexVertex (index: int) =
                let angle = double index * PI / 3.0
                hexCenter + Vector3(HEX_RADIUS * cos angle, HEX_RADIUS * sin angle, 0.0)
            for i = 0 to 5 do
                let currentVertex = calcHexVertex i
                let nextVertex = calcHexVertex (i + 1)
                points.Add currentVertex
                let triCenter = (hexCenter + currentVertex + nextVertex) / 3.0
                points.Add(Vector3(triCenter.X, triCenter.Y, layerZ))

        let private getSnapPointsForHex (coords: SubPrismCoords) (buffer: PooledList<Vector3>) =
            buffer.Clear()
            let hexBasePos = Grid.convertHexToWorld coords.Q coords.R 0 0.0
            generateLayerPoints hexBasePos (double coords.Z * HEX_HEIGHT) buffer
            generateLayerPoints hexBasePos (double coords.Z * HEX_HEIGHT + 0.5 * HEX_HEIGHT) buffer

        let private checkCollisions
            bodyToSnapId
            candidatePosition
            bodyDimensions
            bodyOrientation
            bodyRepo
            spatialHash
            (occupiedCells: Span<uint64>) =
            let mutable isFree = true
            let mutable i = 0
            while isFree && i < occupiedCells.Length do
                let cellKey = &occupiedCells[i]
                for otherId in SpatialHash.query cellKey spatialHash do
                    if isFree && otherId <> bodyToSnapId then
                        let otherBody = &Body.getRef otherId bodyRepo
                        if not <| Unsafe.IsNullRef &otherBody then
                            let result =
                                Collision.checkCollisionSAT
                                    candidatePosition
                                    bodyDimensions
                                    bodyOrientation
                                    otherBody.Position
                                    otherBody.Dimensions
                                    otherBody.Orientation
                            
                            if result.AreColliding then
                                isFree <- false
                i <- i + 1
            isFree
            
        let private isTargetPositionFree
            bodyToSnapId
            candidatePosition
            bodyDimensions
            bodyOrientation
            bodyRepo
            activeHash
            sleepingHash
            geometryRepo
            filterBuffer
            buffer =
                Grid.fillOverlappingSubPrismsAABB
                    candidatePosition
                    bodyDimensions
                    bodyOrientation
                    filterBuffer
                    buffer
                    
                let occupiedCells = buffer.Span

                let mutable isFree =
                    checkCollisions
                        bodyToSnapId
                        candidatePosition
                        bodyDimensions
                        bodyOrientation
                        bodyRepo
                        activeHash
                        occupiedCells
                
                if isFree then
                    isFree <-
                        checkCollisions
                            bodyToSnapId
                            candidatePosition
                            bodyDimensions
                            bodyOrientation
                            bodyRepo
                            sleepingHash
                            occupiedCells
                if isFree then
                    let mutable i = 0
                    while isFree && i < occupiedCells.Length do
                        let cellKey = &occupiedCells[i]
                        if geometryRepo |> Geometry.isSolid cellKey then
                            let struct (staticPos, staticDims, staticOrient) = cellKey |> Grid.getPrismSpaceByKey 
                            let result =
                                Collision.checkCollisionSAT
                                    candidatePosition
                                    bodyDimensions
                                    bodyOrientation
                                    staticPos
                                    staticDims
                                    staticOrient
                                    
                            if result.AreColliding && result.PenetrationVector.Magnitude() > PENETRATION_SLOP then
                                isFree <- false
                        i <- i + 1
                                
                isFree

        [<Struct>]
        type private DistanceComparer =
            interface IComparer<struct(Vector3 * double)> with
                member _.Compare(x, y) =
                    let struct(_, distanceX) = x
                    let struct(_, distanceY) = y
                    distanceX.CompareTo distanceY
            
        let trySnapToGridCollisionAware
            (body: byref<Body.T>)
            bodyRepo
            activeHash
            sleepingHash
            geometryRepo
            buffers =
            
            let offsetFromCenterToBottom = body.Orientation * Vector3(0.0, 0.0, -body.Dimensions.Z / 2.0)
            let bodyPos = body.Position
            let bodyHeight = body.Dimensions.Z

            let halfBodyHeightInCells = int (Math.Ceiling((bodyHeight / 2.0) / HEX_HEIGHT))
            let minZOffset = -halfBodyHeightInCells - 1
            
            let struct(rawQ, rawR, rawZ) = bodyPos |> Grid.convertWorldToRawGridCoords

            use allCandidatePoints = new PooledList<struct(Vector3 * double)>()
            
            for zOffset = minZOffset to 0 do
                let searchCoords = SubPrismCoords.Normalize(rawQ, rawR, rawZ + zOffset, 0)
                getSnapPointsForHex searchCoords buffers.SnapPointsBuffer

                for point in buffers.SnapPointsBuffer.Span do
                    let wrappedPointX = WorldLimits.wrapX point.X
                    let wrappedPointY = WorldLimits.wrapY point.Y
                    let wrappedPoint = Vector3(wrappedPointX, wrappedPointY, point.Z)

                    let deltaX = WorldLimits.relativeX (wrappedPoint.X - bodyPos.X)
                    let deltaY = WorldLimits.relativeY (wrappedPoint.Y - bodyPos.Y)
                    let deltaZ = wrappedPoint.Z - bodyPos.Z
                    
                    let distSq = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ
                    
                    allCandidatePoints.Add(struct(wrappedPoint, distSq))

            allCandidatePoints.Sort(DistanceComparer())
            
            let mutable isFoundSnapPoint = false
            let mutable i = 0
            let allCandidatePoints = allCandidatePoints.Span
            while not isFoundSnapPoint && i < allCandidatePoints.Length do
                let struct(point, _) = allCandidatePoints[i]
                
                if geometryRepo |> Geometry.hasSupportBeneath point then
                    let candidateBodyCenter = point - offsetFromCenterToBottom
                    
                    if isTargetPositionFree
                        body.Id
                        candidateBodyCenter
                        body.Dimensions
                        body.Orientation
                        bodyRepo
                        activeHash
                        sleepingHash
                        geometryRepo
                        buffers.UniquePrismsFilterBuffer
                        buffers.UniquePrismsBuffer 
                    then
                        body.Position <- candidateBodyCenter
                        body.Velocity <- Vector3.Zero
                        body.IsSnappedToGrid <- true
                        isFoundSnapPoint <- true

                i <- i + 1

            isFoundSnapPoint

    [<RequireQualifiedAccess>]
    module Island =
        
        [<Struct; IsReadOnly; StructLayout(LayoutKind.Sequential, Pack = 1)>]
        type ContactData =
            new(slotIndex, itemIndex, cachedSeparatingAxis) =
                {
                    SlotIndex = slotIndex
                    ItemIndexInList = itemIndex
                    CachedSeparatingAxis = cachedSeparatingAxis
                }
                
            val SlotIndex: int
            val ItemIndexInList: int
            val CachedSeparatingAxis: int
            
        [<Struct; IsReadOnly>]
        type ExpiringContacts =
            private new (slots, lookup, currentSlot) =
                {
                    _slots = slots
                    _lookup = lookup
                    _currentSlot = currentSlot
                }
            
            val private _slots: PooledList<int64>[]
            val private _lookup: PooledDictionary<int64, ContactData>
            val private _currentSlot: int

            static member Create() =
                let slots = Array.init (CONTACT_TTL + 1) (fun _ -> new PooledList<_>())
                let lookup = new PooledDictionary<_, _>()
                new ExpiringContacts(slots, lookup, 0)
            
            member this.Count = this._lookup.Count
            member this.Clear() =
                this._lookup.Clear()
                this._slots |> Seq.iter _.Clear()

            member this.TransferTo(target: byref<ExpiringContacts>) =
                for slotIndex = 0 to this._slots.Length - 1 do
                let sourceSlot = this._slots[slotIndex]
                if sourceSlot.Count > 0 then
                    let ttl = (slotIndex - this._currentSlot + this._slots.Length) % this._slots.Length
                    if ttl > 0 then
                        for key in sourceSlot.Span do
                            let axis = this._lookup[key].CachedSeparatingAxis
                            target.AddOrUpdate(key, ttl, axis)
                this.Clear()
                
            member this.AddOrUpdate(key, ttl, newCachedAxis) =
                let newTtl = min CONTACT_TTL (max 1 ttl)
                let newSlotIndex = (this._currentSlot + newTtl) % this._slots.Length

                match this._lookup.TryGetValue key with
                | true, oldData ->
                    if oldData.SlotIndex <> newSlotIndex then
                        this.Remove key |> ignore
                        let targetSlotList = this._slots[newSlotIndex]
                        targetSlotList.Add key
                        this._lookup.Add(key, ContactData(newSlotIndex, targetSlotList.Count - 1, newCachedAxis ))
                    else
                        this._lookup[key] <- ContactData(oldData.SlotIndex, oldData.ItemIndexInList, newCachedAxis)
                | false, _ ->
                    let targetSlotList = this._slots[newSlotIndex]
                    targetSlotList.Add key
                    let newData = ContactData(newSlotIndex, targetSlotList.Count - 1, newCachedAxis)
                    this._lookup.Add(key, newData)
            
            member this.AddOrUpdate(key, ttl) =
                let currentAxis = 
                    match this._lookup.TryGetValue key with
                    | true, data -> data.CachedSeparatingAxis
                    | false, _ -> -1
                this.AddOrUpdate(key, ttl, currentAxis)
            
            member this.TryGetCachedAxis(key, axis: byref<int>) =
                match this._lookup.TryGetValue key with
                | true, data ->
                    axis <- data.CachedSeparatingAxis
                    true
                | false, _ ->
                    axis <- -1
                    false
                
            member this.Remove key =
                match this._lookup.TryGetValue key with
                | true, data ->
                    let slotIndex = data.SlotIndex
                    let itemIndexInList = data.ItemIndexInList

                    this._lookup.Remove key |> ignore

                    let list = this._slots[slotIndex]
                    let lastIndex = list.Count - 1

                    if itemIndexInList < lastIndex then
                        let lastItemKey = list[lastIndex]
                        list[itemIndexInList] <- lastItemKey
                        let item = this._lookup[lastItemKey]
                        this._lookup[lastItemKey] <- ContactData(item.SlotIndex, itemIndexInList, item.CachedSeparatingAxis)

                    list.RemoveAt lastIndex
                    true
                | _ -> false

            member this.NextStep() =
                let newCurrentSlot = (this._currentSlot + 1) % this._slots.Length
                let expiringNow = this._slots[newCurrentSlot]
                let contactsWereRemoved = expiringNow.Count > 0
                if contactsWereRemoved then
                    for key in expiringNow.Span do
                        this._lookup.Remove key |> ignore
                    expiringNow.Clear()

                struct(new ExpiringContacts(this._slots, this._lookup, newCurrentSlot), contactsWereRemoved)
                
            member this.GetKeys() = this._lookup.Keys :> ICollection<_>

            member this.Dispose() =
                this._lookup |> Dispose.action
                this._slots |> Seq.iter Dispose.action

            interface IDisposable with
                member this.Dispose() = this.Dispose()

        [<StructLayout(LayoutKind.Sequential, Pack = 1)>]
        type ContactsHolder =
            {
                mutable Contacts: ExpiringContacts
            }
            
        [<Struct>]
        type T =
            new(id) =
                {
                    Id = id
                    Bodies = new PooledSet<_>()
                    _contactsHolder = null
                    IsAwake = true
                    FramesResting = 0
                    CantSleepFrames = 2
                    IsSlow = false
                    IsGrounded = false
                    HadContactsRemovedThisStep = false
                    MaxPenetrationThisStep = 0.0
                    MinAABB = Vector3(Double.MaxValue, Double.MaxValue, Double.MaxValue)
                    MaxAABB = Vector3(Double.MinValue, Double.MinValue, Double.MinValue)
                }
            val Id: int
            val Bodies: PooledSet<int>
            val mutable _contactsHolder : ContactsHolder | null
            val mutable IsAwake : bool
            val mutable FramesResting : int
            val mutable CantSleepFrames : int
            val mutable IsSlow : bool
            val mutable IsGrounded : bool
            val mutable HadContactsRemovedThisStep : bool
            val mutable MaxPenetrationThisStep : double
            val mutable MinAABB: Vector3
            val mutable MaxAABB: Vector3
    
            member inline this.UpdateMaxPenetration depth =
                if depth > this.MaxPenetrationThisStep then
                    this.MaxPenetrationThisStep <- depth
                    
            member inline this.EnsureContactsExist() =
                if isNull this._contactsHolder then
                    this._contactsHolder <- { Contacts = ExpiringContacts.Create() }
                    
            member inline this.AddOrUpdateContact(key, ttl, newCachedAxis) =
                this.EnsureContactsExist()
                this._contactsHolder.Contacts.AddOrUpdate(key, ttl, newCachedAxis)

            member inline this.TryGetCachedAxis(key, axis: byref<int>) =
                if isNull this._contactsHolder then
                    axis <- -1
                    false
                else
                    this._contactsHolder.Contacts.TryGetCachedAxis(key, &axis)

            member inline this.RemoveContact(key) =
                if isNull this._contactsHolder then
                    false
                else
                    this._contactsHolder.Contacts.Remove(key)

            member this.GetContactKeys() : ICollection<int64> =
                if isNull this._contactsHolder then
                    Array.Empty() :> ICollection<int64>
                else
                    this._contactsHolder.Contacts.GetKeys()
                    
            member this.ContactCount =
                if isNull this._contactsHolder then 0 else this._contactsHolder.Contacts.Count

            member inline this.MergeContactsFrom(sourceIsland: byref<T>) =
                if not <| isNull sourceIsland._contactsHolder then
                    this.EnsureContactsExist()
                    sourceIsland._contactsHolder.Contacts.TransferTo &this._contactsHolder.Contacts
                    
            member this.NextStep() =
                this.MaxPenetrationThisStep <- 0.0
                if this.IsSlow && not <| isNull this._contactsHolder then
                    let struct(newContacts, contactsWereRemoved) = this._contactsHolder.Contacts.NextStep()
                    this._contactsHolder.Contacts <- newContacts
                    this.HadContactsRemovedThisStep <- contactsWereRemoved
                else
                    this.HadContactsRemovedThisStep <- false
                    
            member this.Dispose() =
                let contactsHolder = this._contactsHolder
                if not <| isNull contactsHolder then
                    this._contactsHolder <- null
                    this.Bodies |> Dispose.action
                    contactsHolder.Contacts |> Dispose.action
                    
            interface IDisposable with
                member this.Dispose() = this.Dispose()
        
        let recalculateAABB (island: byref<T>) bodyRepo =
            island.MinAABB <- Vector3(Double.MaxValue, Double.MaxValue, Double.MaxValue)
            island.MaxAABB <- Vector3(Double.MinValue, Double.MinValue, Double.MinValue)

            for bodyId in island.Bodies do
                let body = &Body.getRef bodyId bodyRepo
                if not <| Unsafe.IsNullRef &body then
                    island.MinAABB.X <- min island.MinAABB.X body.MinAABB.X
                    island.MinAABB.Y <- min island.MinAABB.Y body.MinAABB.Y
                    island.MinAABB.Z <- min island.MinAABB.Z body.MinAABB.Z
                    
                    island.MaxAABB.X <- max island.MaxAABB.X body.MaxAABB.X
                    island.MaxAABB.Y <- max island.MaxAABB.Y body.MaxAABB.Y
                    island.MaxAABB.Z <- max island.MaxAABB.Z body.MaxAABB.Z
            
        type Repo =
            private
                {
                    _bodyRepo: Body.Repo
                    _activeHash: SpatialHash.Repo
                    _sleepingHash: SpatialHash.Repo
                    _mergeRedirects : Dictionary<int, int>
                    _activeIslandIds : HashSet<int>
                    _sleepingIslandIds : HashSet<int>
                    _allIslands : Dictionary<int, T>
                    _bodyToIslandMap : Dictionary<int, int>
                    _removeIslandsBuffer : List<int>
                    _islandsToMerge : Queue<struct (int * int)>
                    _islandsToMergePairs : HashSet<struct(int * int)>
                    _islandsToWake: HashSet<int>
                    _islandsToSleep : HashSet<int>
                    _islandsMarkedForSplit : HashSet<int>
                    _islandsInvolvedInMerge: HashSet<int>
                    _staticSupportCache: Dictionary<int, bool>
                    _freeIds : List<int>
                    _logger : ILogger
                    mutable _currentId: int
                }
            member this.ActiveIslandIds = this._activeIslandIds
            member this.SleepingIslandIds = this._sleepingIslandIds

            member this.Dispose() = this._allIslands.Values |> Seq.iter Dispose.action
                
            interface IDisposable with
                member this.Dispose() = this.Dispose()
        
        let getActiveIslandIds r = r._activeIslandIds
        let getSleepingIslandIds r = r._sleepingIslandIds
        
        let createRepo bodyRepo activeHash sleepingHash =
            {
                _bodyRepo = bodyRepo
                _activeHash = activeHash
                _sleepingHash = sleepingHash
                _mergeRedirects = Dictionary()
                _activeIslandIds = HashSet()
                _sleepingIslandIds = HashSet()
                _allIslands = Dictionary()
                _bodyToIslandMap = Dictionary()
                _removeIslandsBuffer = List()
                _islandsToMerge = Queue()
                _islandsToMergePairs = HashSet()
                _islandsToWake = HashSet()
                _islandsToSleep = HashSet()
                _islandsMarkedForSplit = HashSet()
                _islandsInvolvedInMerge = HashSet()
                _staticSupportCache = Dictionary()
                _freeIds = List()
                _logger = Log.ForContext<Repo>()
                _currentId = -1
            }

        let inline private newIsland r =
            let newId =
                match r._freeIds.Count with
                | 0 -> Interlocked.Increment &r._currentId
                | count ->
                    let lastIndex = count - 1
                    let newId = r._freeIds[lastIndex]
                    r._freeIds.RemoveAt lastIndex
                    newId
                    
            new T(newId)
            
        let init r =   
            for bodyId in r._bodyRepo |> Body.getKeys do
                let newIsland = r |> newIsland
                newIsland.Bodies.Add bodyId |> ignore
                r._bodyToIslandMap.Add(bodyId, newIsland.Id)
                r._allIslands.Add(newIsland.Id, newIsland)
                r._activeIslandIds.Add newIsland.Id |> ignore

        let private buildAdjacencyMap
            (bodies: ICollection<int>)
            (contactKeys: ICollection<int64>)
            =
            
            let adjacencyMap = new PooledDictionary<int, PooledList<int>>()
            for bodyId in bodies do
                adjacencyMap.Add(bodyId, new PooledList<int>())

            for contactKey in contactKeys do
                let struct(id1, id2) = contactKey |> ContactKey.unpack
                match adjacencyMap.TryGetValue id1 with
                | true, a1 ->
                    match adjacencyMap.TryGetValue id2 with
                    | true, a2 ->
                        a1.Add id2
                        a2.Add id1
                    | _ -> ()
                | _ -> ()
                    
            adjacencyMap
        
        let private findSingleConnectedComponent
            (startNode: int)
            (adjacencyMap: IDictionary<int, PooledList<int>>)
            (visited: PooledSet<int>) =
            
            let newComponent = new PooledList<int>()
            use queue = new PooledQueue<int>()

            if not <| visited.Contains startNode then
                queue.Enqueue startNode
                visited.Add startNode |> ignore

                while queue.Count > 0 do
                    let currentId = queue.Dequeue()
                    newComponent.Add currentId

                    for neighborId in adjacencyMap[currentId].Span do
                        if visited.Add neighborId then
                            queue.Enqueue neighborId
            
            newComponent
            
        let private findConnectedComponents
            (bodiesToAnalyze: ICollection<int>)
            (island: inref<T>) =
            use adjacencyMap = buildAdjacencyMap bodiesToAnalyze (island.GetContactKeys())
            let result = new PooledList<PooledList<int>>()
            use visited = new PooledSet<int>()

            for bodyId in bodiesToAnalyze do
                let newComponent = findSingleConnectedComponent bodyId adjacencyMap visited
                if newComponent.Count > 0 then
                    result.Add newComponent
                else
                    newComponent |> Dispose.action
                    
            adjacencyMap.Values |> Seq.iter Dispose.action
            result
        
        let private hasStaticSupport
            (body: inref<Body.T>)
            (occupiedCells: ReadOnlySpan<uint64>)
            geometryRepo
            r =
            
            let cache = r._staticSupportCache
            match cache.TryGetValue body.Id with
            | true, result -> result
            | false, _ ->
                
                let mutable hasFoundSupport = false

                let h = body.Dimensions / 2.0
                let mutable lowestPointZ = Double.MaxValue
                for i = 0 to 7 do
                    let signX = if (i &&& 1) = 0 then -1.0 else 1.0
                    let signY = if (i &&& 2) = 0 then -1.0 else 1.0
                    let signZ = if (i &&& 4) = 0 then -1.0 else 1.0
                    let localCorner = Vector3(h.X * signX, h.Y * signY, h.Z * signZ)
                    let worldCorner = body.Position + body.Orientation * localCorner
                    lowestPointZ <- min lowestPointZ worldCorner.Z
                
                if lowestPointZ < PENETRATION_SLOP then
                    hasFoundSupport <- true
      
                if not <| hasFoundSupport then
                    let mutable i = 0
                    while not hasFoundSupport && i < occupiedCells.Length do
                        let cellKey = &occupiedCells[i]
                        if geometryRepo |> Geometry.isSolid cellKey then
                            let struct (staticPos, staticDims, staticOrient) = cellKey |> Grid.getPrismSpaceByKey
                            let result =
                                Collision.checkCollisionSAT
                                    body.Position
                                    body.Dimensions
                                    body.Orientation
                                    staticPos
                                    staticDims
                                    staticOrient
                                    
                            if result.AreColliding then
                                hasFoundSupport <- true
                        i <- i + 1
                
                cache.Add(body.Id, hasFoundSupport)
                
                hasFoundSupport
        
        let isGrounded
            bodyRepo
            activeHash
            geometryRepo
            (island: inref<T>)
            r =
            
            if island.Bodies.Count = 0 then
                true
            elif island.Bodies.Count = 1 then
                let bodyId = island.Bodies.GetEnumerator().Current
                let body = &Body.getRef bodyId bodyRepo
                if Unsafe.IsNullRef &body then
                    false
                else
                    let occupiedCells = SpatialHash.getOccupiedCells bodyId activeHash
                    hasStaticSupport &body occupiedCells geometryRepo r
            else
                use anchors = new PooledList<int>()
                for bodyId in island.Bodies do
                    let body = &Body.getRef bodyId bodyRepo
                    if not <| Unsafe.IsNullRef &body then
                        let occupiedCells = SpatialHash.getOccupiedCells bodyId activeHash
                        if hasStaticSupport &body occupiedCells geometryRepo r then
                            anchors.Add bodyId

                if anchors.Count = 0 then
                    false
                elif anchors.Count = island.Bodies.Count then
                    true
                else
                    use adjacencyMap = buildAdjacencyMap island.Bodies (island.GetContactKeys())
                    use visited = new PooledSet<int>()
                    use queue = new PooledQueue<int>()
                    
                    for anchorId in anchors.Span do
                        if visited.Add anchorId then
                            queue.Enqueue anchorId

                    while queue.Count > 0 do
                        let currentId = queue.Dequeue()
                        match adjacencyMap.TryGetValue currentId with
                        | true, a ->
                            for neighborId in a.Span do
                                if visited.Add neighborId then
                                    queue.Enqueue neighborId
                        | _ -> ()
                        
                    let isGrounded = visited.Count = island.Bodies.Count
                    
                    adjacencyMap.Values |> Seq.iter Dispose.action
                    
                    isGrounded
                
                
        let getIslandRefForBody bodyId r = &CollectionsMarshal.GetValueRefOrNullRef(r._allIslands, r._bodyToIslandMap[bodyId])
        let getIslandIdForBody bodyId r = r._bodyToIslandMap[bodyId]
        let getIslandRef islandId r = &CollectionsMarshal.GetValueRefOrNullRef(r._allIslands, islandId)

        let addBody bodyId r=
            let mutable isFound = false
            let islandId = &CollectionsMarshal.GetValueRefOrAddDefault(r._bodyToIslandMap, bodyId, &isFound)
            if not <| isFound then
                let newIsland = r |> newIsland
                r._allIslands.Add(newIsland.Id, newIsland)
                islandId <- newIsland.Id
                newIsland.Bodies.Add bodyId |> ignore
                r._activeIslandIds.Add newIsland.Id |> ignore
                
        let removeBody bodyId r =
            let mutable islandId = 0
            if r._bodyToIslandMap.TryGetValue(bodyId, &islandId) then
                r._bodyToIslandMap.Remove bodyId |> ignore
                
                let island = &getIslandRef islandId r
                island.Bodies.Remove bodyId |> ignore
                if island.Bodies.Count = 0 then
                    if not <| r._removeIslandsBuffer.Contains islandId then
                       r._removeIslandsBuffer.Add islandId
                       
            r._staticSupportCache.Remove bodyId |> ignore
            
        let requestWakeIsland (island: byref<T>) r =
            island.FramesResting <- 0
            if not <| island.IsAwake && r._islandsToWake.Add island.Id then
                r._logger.Debug("Request wake island: {IslandId}", island.Id)
            
        let islandMerge sourceId targetId r =
            if sourceId <> targetId then
                let mutable sourceIsland = &getIslandRef sourceId r
                let mutable targetIsland = &getIslandRef targetId r
                if not <| Unsafe.IsNullRef &sourceIsland && not <| Unsafe.IsNullRef &targetIsland then
                    targetIsland.CantSleepFrames <- max sourceIsland.CantSleepFrames targetIsland.CantSleepFrames
                    targetIsland.FramesResting <- 0
                    targetIsland.IsGrounded <- sourceIsland.IsGrounded || targetIsland.IsGrounded
                    targetIsland.UpdateMaxPenetration sourceIsland.MaxPenetrationThisStep
                    
                    use bodiesToMove = new PooledList<int>(sourceIsland.Bodies)
                    for bodyId in bodiesToMove.Span do
                        targetIsland.Bodies.Add bodyId |> ignore
                        r._bodyToIslandMap[bodyId] <- targetId
                    
                    targetIsland.MergeContactsFrom &sourceIsland
   
                    targetIsland.MinAABB.X <- min targetIsland.MinAABB.X sourceIsland.MinAABB.X
                    targetIsland.MinAABB.Y <- min targetIsland.MinAABB.Y sourceIsland.MinAABB.Y
                    targetIsland.MinAABB.Z <- min targetIsland.MinAABB.Z sourceIsland.MinAABB.Z
                    
                    targetIsland.MaxAABB.X <- max targetIsland.MaxAABB.X sourceIsland.MaxAABB.X
                    targetIsland.MaxAABB.Y <- max targetIsland.MaxAABB.Y sourceIsland.MaxAABB.Y
                    targetIsland.MaxAABB.Z <- max targetIsland.MaxAABB.Z sourceIsland.MaxAABB.Z
                    
                    r._freeIds.Add sourceIsland.Id
                    
                    sourceIsland |> Dispose.action
                    if not <| r._removeIslandsBuffer.Contains sourceId then
                       r._removeIslandsBuffer.Add sourceId
                
        let requestSleep (island: byref<T>) r =
            if island.IsAwake && r._islandsToSleep.Add island.Id then
                r._logger.Debug("Request sleep island: {IslandId}", island.Id)
            
        let requestMerge id1 id2 r =
            if id1 <> id2 then
                let inline findRoot (islandId: int) =
                    let mutable currentId = islandId
                    use path = new PooledList<int>()

                    while r._mergeRedirects.ContainsKey currentId do
                        path.Add currentId
                        currentId <- r._mergeRedirects[currentId]
                    
                    let rootId = currentId
                    for nodeOnPath in path.Span do
                        r._mergeRedirects[nodeOnPath] <- rootId

                    rootId

                let root1 = findRoot id1
                let root2 = findRoot id2

                if root1 <> root2 then
                    // Merge is deferred if a split has already been requested for one of the root islands
                    // in the same frame. This gives split requests priority if they arrive first
                    if r._islandsMarkedForSplit.Contains root1 || r._islandsMarkedForSplit.Contains root2 then
                        r._logger.Debug("Merge deferred for ({Id1}, {Id2}) because one root is marked for split", id1, id2)
                    else
                        let mutable island1 = &getIslandRef root1 r
                        let mutable island2 = &getIslandRef root2 r
                        if not <| Unsafe.IsNullRef &island1 && not <| Unsafe.IsNullRef &island2 then
                            requestWakeIsland &island1 r
                            requestWakeIsland &island2 r
                        
                            let struct(sourceRoot, targetRoot) =
                                if island1.Bodies.Count < island2.Bodies.Count then
                                    (root1, root2)
                                else
                                    (root2, root1)
                                    
                            let pair = struct (sourceRoot, targetRoot)
                            if r._islandsToMergePairs.Add pair then
                                r._logger.Debug(
                                    "Redirected Merge: original ({Id1}, {Id2}) -> final ({SourceRoot}, {TargetRoot})",
                                    id1,
                                    id2,
                                    sourceRoot,
                                    targetRoot)
                                
                                r._islandsToMerge.Enqueue pair

                                r._mergeRedirects[sourceRoot] <- targetRoot
                                
                                r._islandsInvolvedInMerge.Add root1 |> ignore
                                r._islandsInvolvedInMerge.Add root2 |> ignore
                    
        let requestSplit islandId r =
            if r._islandsInvolvedInMerge.Contains islandId then
                r._logger.Debug("Split request for island {IslandId} ignored due to a pending merge", islandId)
            elif r._islandsMarkedForSplit.Add islandId then
                r._logger.Debug("Request split island: {IslandId}", islandId)
        
        let private removeIslands r =
            let mergedIdsToRemove = CollectionsMarshal.AsSpan r._removeIslandsBuffer
            for removedId in mergedIdsToRemove do
                r._allIslands.Remove removedId |> ignore
                r._activeIslandIds.Remove removedId |> ignore
                r._sleepingIslandIds.Remove removedId |> ignore
            r._removeIslandsBuffer.Clear()
            
        let processIslandChanges geometryRepo buffers r =
            
            r._staticSupportCache.Clear()
            r._mergeRedirects.Clear()
            r._islandsInvolvedInMerge.Clear()
            
            for islandId in r._islandsToWake do
                let mutable island = &getIslandRef islandId r
                if Unsafe.IsNullRef &island then
                    r._removeIslandsBuffer.Add islandId
                elif island.Bodies.Count = 0 then
                        r._removeIslandsBuffer.Add island.Id
                        island |> Dispose.action
                elif not island.IsAwake then
                    island.IsAwake <- true
                    island.FramesResting <- 0

                    // It is strictly forbidden to erase ContactTTL,
                    // because any awakened island must remember all its contacts, so as not to
                    // start the tedious and expensive procedure of assembling back into the island again
                    // island.ContactTTL.Clear()
                    
                    for bodyId in island.Bodies do
                        r._sleepingHash |> SpatialHash.remove bodyId
                        let body = &Body.getRef bodyId r._bodyRepo
                        if not <| Unsafe.IsNullRef &body then
                            SpatialHash.add &body buffers r._activeHash
                        r._staticSupportCache.Remove bodyId |> ignore
                        
                    r._sleepingIslandIds.Remove island.Id |> ignore
                    r._activeIslandIds.Add island.Id |> ignore
            r._islandsToWake.Clear()

            while r._islandsToMerge.Count > 0 do
                let struct(sourceId, targetId) = r._islandsToMerge.Dequeue()
                r |> islandMerge sourceId targetId
            r._islandsToMergePairs.Clear()

            r |> removeIslands
    
            for islandIdToSplit in r._islandsMarkedForSplit do
                let mutable originalIsland = &getIslandRef islandIdToSplit r
                if not <| Unsafe.IsNullRef &originalIsland then
                    use components = findConnectedComponents originalIsland.Bodies &originalIsland

                    if components.Count <= 1 then
                        components |> Seq.iter Dispose.action
                    else

                        r._logger.Debug("Splitting {IslandId} into {ComponentCount} components",  originalIsland.Id, components.Count)

                        let originalMaxPenetration = originalIsland.MaxPenetrationThisStep
                        
                        let mutable largestComponentIndex = -1
                        let mutable maxBodyCount = -1
                        for i = 0 to components.Count - 1 do
                            if components[i].Count > maxBodyCount then
                                maxBodyCount <- components[i].Count
                                largestComponentIndex <- i

                        let originalContactKeys = originalIsland.GetContactKeys()
                        for i = 0 to components.Count - 1 do
                            if i <> largestComponentIndex then
                                let fragmentComponent = components[i]
                                let mutable newFragmentIsland = r |> newIsland
                                r._allIslands.Add(newFragmentIsland.Id, newFragmentIsland)
                                r._activeIslandIds.Add newFragmentIsland.Id |> ignore
                                newFragmentIsland.MaxPenetrationThisStep <- originalMaxPenetration
                                
                                r._logger.Debug("New island fragment {IslandId} with {ComponentCount} was created", newFragmentIsland.Id, fragmentComponent.Count)

                                for bodyId in fragmentComponent.Span do
                                    newFragmentIsland.Bodies.Add bodyId |> ignore
                                    originalIsland.Bodies.Remove bodyId |> ignore
                                    r._bodyToIslandMap[bodyId] <- newFragmentIsland.Id

                                for contactKey in originalContactKeys do
                                    let struct(id1, id2) = ContactKey.unpack contactKey
                                    if newFragmentIsland.Bodies.Contains id1 && newFragmentIsland.Bodies.Contains id2 then
                                       let mutable cachedAxis = -1
                                       originalIsland.TryGetCachedAxis(contactKey, &cachedAxis) |> ignore
                                       newFragmentIsland.AddOrUpdateContact(contactKey, CONTACT_TTL, cachedAxis)
                                
                                newFragmentIsland.IsGrounded <-
                                    isGrounded
                                        r._bodyRepo
                                        r._activeHash
                                        geometryRepo
                                        &newFragmentIsland
                                        r
                                        
                                recalculateAABB &newFragmentIsland r._bodyRepo
                                
                        originalIsland.IsGrounded <-
                            isGrounded
                                r._bodyRepo
                                r._activeHash
                                geometryRepo
                                &originalIsland
                                r
                        
                        recalculateAABB &originalIsland r._bodyRepo
                        
                        use contactsToRemove = new PooledList<int64>()
                        for contactKey in originalContactKeys do
                            let struct(id1, id2) = ContactKey.unpack contactKey
                            if not <| originalIsland.Bodies.Contains id1 || not <| originalIsland.Bodies.Contains id2 then
                                contactsToRemove.Add contactKey
                        
                        for key in contactsToRemove.Span do
                            originalIsland.RemoveContact key |> ignore

                        components |> Seq.iter Dispose.action
                
            r._islandsMarkedForSplit.Clear()

            for islandId in r._islandsToSleep do
                if r._activeIslandIds.Contains islandId then
                    let island = &getIslandRef islandId r
                    if Unsafe.IsNullRef &island then
                        if not <| r._removeIslandsBuffer.Contains islandId then
                            r._removeIslandsBuffer.Add islandId
                    elif island.Bodies.Count = 0 then
                        if not <| r._removeIslandsBuffer.Contains islandId then
                            r._removeIslandsBuffer.Add islandId
                        island |> Dispose.action                       
                    elif island.IsAwake then
                        let mutable isStillEligibleForSleep = true
                        let mutable enumerator = island.Bodies.GetEnumerator()
                        while isStillEligibleForSleep && enumerator.MoveNext() do
                            let bodyId = enumerator.Current
                            let body = &Body.getRef bodyId r._bodyRepo
                            if not <| Unsafe.IsNullRef &body && body.Velocity.MagnitudeSq() >= SLEEP_VELOCITY_THRESHOLD_SQ then
                                isStillEligibleForSleep <- false
                        
                        if isStillEligibleForSleep then
                            island.IsAwake <- false
                            island.MaxPenetrationThisStep <- 0.0
                            for bodyId in island.Bodies do
                                r._activeHash |> SpatialHash.remove bodyId
                                let body = &Body.getRef bodyId r._bodyRepo
                                if not <| Unsafe.IsNullRef &body then
                                    body.Velocity <- Vector3.Zero
                                    SpatialHash.add &body buffers r._sleepingHash
                            
                            recalculateAABB &island r._bodyRepo
                            
                            r._activeIslandIds.Remove island.Id |> ignore
                            r._sleepingIslandIds.Add island.Id |> ignore
                        else
                            island.FramesResting <- 0
          
            r._islandsToSleep.Clear()

            for island in r._allIslands.Values do
                if island.Bodies.Count = 0 then
                    r._removeIslandsBuffer.Add island.Id
                    
            r |> removeIslands

    [<RequireQualifiedAccess>]
    module Liquid =
        
        let private _directions = [| struct(1, 0); (0, 1); (-1, 1); (-1, 0); (0, -1); (1, -1) |]
        
        type Repo =
            private
                {
                    _liquidCells : HashSet<uint64>
                    _bufferForEmpty : PooledList<uint64>
                    _bufferForFill : PooledList<uint64>
                    _geometryRepo : Geometry.Repo
                }
                member this.Dispose() =
                    this._bufferForEmpty |> Dispose.action
                    this._bufferForFill |> Dispose.action
                    
                interface IDisposable with
                    member this.Dispose() = this.Dispose()
        let createRepo geometryRepo =
            {
                _liquidCells = HashSet<uint64>()
                _bufferForEmpty = new PooledList<uint64>()
                _bufferForFill = new PooledList<uint64>()
                _geometryRepo = geometryRepo
            }
 
        let isSpreadable targetCoords r=
            not <| (r._geometryRepo |> Geometry.isSolid targetCoords)
            && not <| (r._liquidCells.Contains targetCoords)

        let addLiquid(coords: SubPrismCoords) r = coords |> SubPrismKey.pack |> r._liquidCells.Add |> ignore

        let isLiquid(coords: uint64) r= r._liquidCells.Contains coords

        let updatePhysics r =
            r._bufferForEmpty.Clear()
            r._bufferForFill.Clear()

            for cell in r._liquidCells do
                let unpackedCell = cell |> SubPrismKey.unpack
                let belowCoords = SubPrismCoords.Normalize(unpackedCell.Q, unpackedCell.R, unpackedCell.Z - 1, unpackedCell.SubIndex)

                let packedBelowCoords = belowCoords |> SubPrismKey.pack
                if r |> isSpreadable packedBelowCoords then
                    r._bufferForFill.Add packedBelowCoords
                    r._bufferForEmpty.Add cell
                else
                    let mutable hasSpread = false
                    for dq, dr in _directions do
                        let neighbor = SubPrismCoords.Normalize(unpackedCell.Q + dq, unpackedCell.R + dr, unpackedCell.Z, unpackedCell.SubIndex)
                        
                        let packedNeighbor = neighbor |> SubPrismKey.pack
                        
                        if r |> isSpreadable packedNeighbor then
                            r._bufferForFill.Add packedNeighbor
                            hasSpread <- true

                    if hasSpread then
                        r._bufferForEmpty.Add cell

            r._bufferForEmpty |> Seq.iter(fun c -> r._liquidCells.Remove c |> ignore)
            r._bufferForFill |> Seq.iter(fun c -> r._liquidCells.Add c |> ignore)
    
               
    
    type T =
        private
            {
                _geometryRepo: Geometry.Repo
                _bodyRepo : Body.Repo
                _activeHash : SpatialHash.Repo
                _sleepingHash: SpatialHash.Repo
                _islandRepo : Island.Repo
                _liquidRepo : Liquid.Repo
                _floraRepo : Flora.Repo
                _buffers : Buffers
                _random : Random
                _dt : double
            }
        
        member this.Geometry = this._geometryRepo
        member this.Bodies = this._bodyRepo
        member this.ActiveHash = this._activeHash
        member this.SleepingHash = this._sleepingHash
        member this.Islands = this._islandRepo
        member this.Liquid = this._liquidRepo
        member this.Flora = this._floraRepo
        member this.Buffers = this._buffers
        
        member this.Dispose() =
            this._activeHash |> Dispose.action
            this._sleepingHash |> Dispose.action
            this._islandRepo |> Dispose.action
            this._liquidRepo |> Dispose.action
            this._floraRepo |> Dispose.action
            this._buffers |> Dispose.action
        interface IDisposable with
            member this.Dispose() = this.Dispose()
    let createEngine dt =
        
        let logger =
            LoggerConfiguration()
                .WriteTo.Async(fun c -> c.Console() |> ignore)
                .MinimumLevel.Information()
                .CreateLogger();

        Log.CloseAndFlush()

        Log.Logger <- logger

        let geometryRepo = Geometry.createRepo()
        let bodyRepo = Body.createRepo()

        use activeHash = SpatialHash.createRepo()
        use sleepingHash = SpatialHash.createRepo()

        {
            _bodyRepo = bodyRepo
            _islandRepo = Island.createRepo bodyRepo activeHash sleepingHash
            _floraRepo = Flora.createRepo geometryRepo
            _geometryRepo = geometryRepo
            _liquidRepo = Liquid.createRepo geometryRepo
            _activeHash = activeHash
            _sleepingHash = sleepingHash
            _buffers = Buffers.Create()
            _random = Random(Guid.NewGuid().GetHashCode())
            _dt = dt
        }
        
    let nextBodyId engine = engine._bodyRepo |> Body.nextId
    
    [<RequireQualifiedAccess>]
    module Simulation =          
        let inline private isPointInsideOBB (point: Vector3) (obbPos: Vector3) (obbDims: Vector3) (obbOrient: Matrix3x3) =

            let h = obbDims / 2.0
            let mutable delta = point - obbPos
            WorldLimits.relative &delta
            let localPoint = obbOrient.Transpose() * delta
            abs(localPoint.X) <= h.X + PENETRATION_SLOP  &&
            abs(localPoint.Y) <= h.Y + PENETRATION_SLOP  &&
            abs(localPoint.Z) <= h.Z + PENETRATION_SLOP 

        let checkPartnersInHash
            checkPoint
            bodyRepo
            checkPointCellKey
            (body: inref<Body.T>)
            hash =
                
            let mutable isFound = false
            for otherId in SpatialHash.query checkPointCellKey hash do
                if not <| isFound && otherId <> body.Id then
                    let otherBody = &Body.getRef otherId bodyRepo
                    if not <| Unsafe.IsNullRef &otherBody && not <| otherBody.IsFallingOver then
                        if isPointInsideOBB checkPoint otherBody.Position otherBody.Dimensions otherBody.Orientation then
                            isFound <- true
            isFound

        let private checkWorldPosForSupport
            (p: Vector3)
            geometryRepo
            bodyRepo
            (body: inref<Body.T>)
            activeHash
            sleepingHash =

            let checkPos = Vector3(p.X, p.Y, p.Z - PENETRATION_SLOP)
            let key = checkPos |> Grid.convertWorldToSubPrismCoords |> SubPrismKey.pack

            geometryRepo |> Geometry.isSolid key
            || checkPartnersInHash p bodyRepo key &body activeHash
            || checkPartnersInHash p bodyRepo key &body sleepingHash

        let private checkSinglePoint
            geometryRepo
            bodyRepo
            (body: inref<Body.T>)
            activeHash
            sleepingHash
            (point: Vector3) =
        
            if point.Z <= PENETRATION_SLOP then
                true
            else
                if checkWorldPosForSupport point geometryRepo bodyRepo &body activeHash sleepingHash then
                    true
                else
                    let radius = HEX_RADIUS * 0.1
                    if checkWorldPosForSupport (point + Vector3(radius, 0.0, 0.0)) geometryRepo bodyRepo &body activeHash sleepingHash then true
                    elif checkWorldPosForSupport (point - Vector3(radius, 0.0, 0.0)) geometryRepo bodyRepo &body activeHash sleepingHash then true
                    elif checkWorldPosForSupport (point + Vector3(0.0, radius, 0.0)) geometryRepo bodyRepo &body activeHash sleepingHash then true
                    elif checkWorldPosForSupport (point - Vector3(0.0, -radius, 0.0)) geometryRepo bodyRepo &body activeHash sleepingHash then true
                    else false
        
        let isPointSupported
            geometryRepo
            bodyRepo
            (body: inref<Body.T>)
            activeHash
            sleepingHash
            (checkPoint: Vector3) =
            if checkSinglePoint geometryRepo bodyRepo &body activeHash sleepingHash checkPoint then
                true
            else
                let bodyHalfExtent = (max body.Dimensions.X body.Dimensions.Y) / 2.0
                let maxSupporterHalfExtent = MAX_DIMENSION / 2.0
                let safeMargin = bodyHalfExtent + maxSupporterHalfExtent + PENETRATION_SLOP

                let nearMinX = checkPoint.X < safeMargin
                let nearMaxX = checkPoint.X > (WorldLimits.X - safeMargin)
                let nearMinY = checkPoint.Y < safeMargin
                let nearMaxY = checkPoint.Y > (WorldLimits.Y - safeMargin)

                let mutable foundSupportInGhost = false

                if nearMinX then
                    foundSupportInGhost <- foundSupportInGhost || checkSinglePoint geometryRepo bodyRepo &body activeHash sleepingHash (checkPoint + Vector3(WorldLimits.X, 0.0, 0.0))
                if not <| foundSupportInGhost && nearMaxX then
                    foundSupportInGhost <- foundSupportInGhost || checkSinglePoint geometryRepo bodyRepo &body activeHash sleepingHash (checkPoint - Vector3(WorldLimits.X, 0.0, 0.0))
                if not <| foundSupportInGhost && nearMinY then
                    foundSupportInGhost <- foundSupportInGhost || checkSinglePoint geometryRepo bodyRepo &body activeHash sleepingHash (checkPoint + Vector3(0.0, WorldLimits.Y, 0.0))
                if not <| foundSupportInGhost && nearMaxY then
                    foundSupportInGhost <- foundSupportInGhost || checkSinglePoint geometryRepo bodyRepo &body activeHash sleepingHash (checkPoint - Vector3(0.0, WorldLimits.Y, 0.0))
                if not <| foundSupportInGhost && nearMinX && nearMinY then
                    foundSupportInGhost <- foundSupportInGhost || checkSinglePoint geometryRepo bodyRepo &body activeHash sleepingHash (checkPoint + Vector3(WorldLimits.X, WorldLimits.Y, 0.0))
                if not <| foundSupportInGhost && nearMaxX && nearMinY then
                    foundSupportInGhost <- foundSupportInGhost || checkSinglePoint geometryRepo bodyRepo &body activeHash sleepingHash (checkPoint + Vector3(-WorldLimits.X, WorldLimits.Y, 0.0))
                if not <| foundSupportInGhost && nearMinX && nearMaxY then
                    foundSupportInGhost <- foundSupportInGhost || checkSinglePoint geometryRepo bodyRepo &body activeHash sleepingHash (checkPoint + Vector3(WorldLimits.X, -WorldLimits.Y, 0.0))
                if not <| foundSupportInGhost && nearMaxX && nearMaxY then
                    foundSupportInGhost <- foundSupportInGhost || checkSinglePoint geometryRepo bodyRepo &body activeHash sleepingHash (checkPoint + Vector3(-WorldLimits.X, -WorldLimits.Y, 0.0))

                foundSupportInGhost
        
        let private tryInitiateFall
            (body: inref<Body.T>)
             geometryRepo
             bodyRepo
             activeHash
             sleepingHash =
            if body.IsFallingOver || body.InvMass < EPSILON then
                ValueNone
            else
                let h = body.Dimensions / 2.0
                let bottomCenterOffset = body.Orientation * Vector3(0.0, 0.0, -h.Z)
                let mutable projectedCenterOfMass = body.Position + bottomCenterOffset
                WorldLimits.wrapPosition &projectedCenterOfMass
                
                let isCenterOfMassSupported = isPointSupported geometryRepo bodyRepo &body activeHash sleepingHash projectedCenterOfMass
                if isCenterOfMassSupported then
                    ValueNone
                else
                    let localBaseCenter = Vector3(0.0, 0.0, -h.Z)
                    let worldBaseCenter = body.Position + body.Orientation * localBaseCenter
                    let mutable instabilityVector = body.Position - worldBaseCenter
                    WorldLimits.relative &instabilityVector
                    
                    instabilityVector.Z <- 0.0

                    let corners = [| Vector3(-h.X, -h.Y, -h.Z); Vector3(h.X, -h.Y, -h.Z); 
                                     Vector3(h.X, h.Y, -h.Z); Vector3(-h.X, h.Y, -h.Z) |]
                    
                    let mutable pivotCornerLocal = Vector3.Zero
                    let mutable minProjection = Double.MaxValue

                    for localCorner in corners do
                        let worldCornerVector = body.Orientation * localCorner
                        let projection = Vector3.Dot(instabilityVector, worldCornerVector)
                        
                        if projection < minProjection then
                            minProjection <- projection
                            pivotCornerLocal <- localCorner
                    
                    let pivotPoint = body.Position + body.Orientation * pivotCornerLocal
                    let mutable fallDirection = body.Position - pivotPoint
                    WorldLimits.relative &fallDirection
                    fallDirection.Z <- 0.0 
                    let rotationAxis = Vector3.Cross(fallDirection, Vector3.Up)
                    let finalAxis = if rotationAxis.MagnitudeSq() < EPSILON then Vector3.UnitX else rotationAxis.Normalize()
                    ValueSome(struct(pivotPoint, finalAxis))

        let private updateSpatialHashIncrementally
            bodyId
            (newlyOccupiedCells: PooledList<int64>)
            (bodyGridOccupationCache: Dictionary<int, PooledList<int64>>)
            (spatialHash: Dictionary<int64, PooledList<int>>)
            =
            match bodyGridOccupationCache.TryGetValue bodyId with
            | false, _ -> 
                for newCell in newlyOccupiedCells.Span do
                    match spatialHash.TryGetValue newCell with
                    | true, idList -> idList.Add bodyId
                    | false, _ ->
                        let newList = new PooledList<int>()
                        newList.Add bodyId
                        spatialHash.Add(newCell, newList)
                bodyGridOccupationCache.Add(bodyId, newlyOccupiedCells)
            | true, oldOccupiedCells -> 
                use newCellsSet = new PooledSet<int64>(newlyOccupiedCells.Span)
                for oldCell in oldOccupiedCells.Span do
                    if not <| newCellsSet.Remove oldCell then
                        match spatialHash.TryGetValue oldCell with
                        | true, idList ->
                            idList |> Utils.removeBySwapBack bodyId |> ignore
                            if idList.Count = 0 then
                                spatialHash.Remove oldCell |> ignore
                                idList |> Dispose.action
                        | false, _ -> ()

                for newCell in newCellsSet do
                    match spatialHash.TryGetValue newCell with
                    | true, idList -> idList.Add bodyId
                    | false, _ ->
                        let newList = new PooledList<int>()
                        newList.Add bodyId
                        spatialHash.Add(newCell, newList)
                
                oldOccupiedCells |> Dispose.action
                bodyGridOccupationCache[bodyId] <- newlyOccupiedCells
        
        let private removeBodyFromSpatialCache
            (bodyId: int)
            (bodyGridOccupationCache: Dictionary<int, PooledList<int64>>)
            (spatialHash: Dictionary<int64, PooledList<int>>)
            =
            match bodyGridOccupationCache.TryGetValue bodyId with
            | true, occupiedCells ->
                for cell in occupiedCells.Span do
                    match spatialHash.TryGetValue cell with
                    | true, idList ->
                        idList |> Utils.removeBySwapBack bodyId |> ignore
                        if idList.Count = 0 then
                            spatialHash.Remove cell |> ignore
                            idList |> Dispose.action
                    | false, _ -> ()
                occupiedCells |> Dispose.action
                bodyGridOccupationCache.Remove bodyId |> ignore
            | false, _ -> ()

        let inline private getStableFrictionNormal (normal: Vector3) (b1: inref<Body.T>) (b2: inref<Body.T>) =
            if abs(Vector3.Dot(b1.Orientation.R2, b2.Orientation.R2)) > 0.98 && abs(normal.Z) > 0.9 then
                if normal.Z > 0.0 then Vector3.Up else Vector3.Down
            else
                normal
                
        let inline private resolveDynamicCollision
            (b1: byref<Body.T>)
            (b2: byref<Body.T>)
            (result: CollisionResult)
            invMass1
            invMass2 =
                
            let finalInvMass1 = if b1.IsFallingOver then 0.0 else invMass1
            let finalInvMass2 = if b2.IsFallingOver then 0.0 else invMass2
            let totalInvMass = finalInvMass1 + finalInvMass2

            if totalInvMass <= EPSILON then
                0.0
            else

                let struct(finalNormal, penetrationDepth) = result.Normalize() 

                let relativeVelocity = b1.Velocity - b2.Velocity
                let velAlongNormal = Vector3.Dot(relativeVelocity, finalNormal)
                let mutable totalImpulseScalar = 0.0

                if velAlongNormal < 0.0 then
                    let e = if abs velAlongNormal < RESTITUTION_THRESHOLD then 0.0 else 0.1
                    let j = -(1.0 + e) * velAlongNormal / totalInvMass
                    let impulse = j * finalNormal
                    
                    b1.Velocity <- b1.Velocity + impulse * finalInvMass1
                    b2.Velocity <- b2.Velocity - impulse * finalInvMass2
                    
                    totalImpulseScalar <- j

                if penetrationDepth > PENETRATION_SLOP then
                    let correctionMagnitude = penetrationDepth * CORRECTION_PERCENT
                    let correctionVector = finalNormal * correctionMagnitude

                    b1.Position <- b1.Position + correctionVector * finalInvMass1
                    b2.Position <- b2.Position - correctionVector * finalInvMass2

                    WorldLimits.wrapPosition &b1.Position
                    WorldLimits.wrapPosition &b2.Position

                totalImpulseScalar

        let inline private resolveDynamicFriction
            (b1: byref<Body.T>)
            (b2: byref<Body.T>)
            (normal: Vector3)
            normalImpulseSum
            invMass1
            invMass2
            dt =
            let finalInvMass1 = if b1.IsFallingOver then 0.0 else invMass1
            let finalInvMass2 = if b2.IsFallingOver then 0.0 else invMass2
            let totalInvMass = finalInvMass1 + finalInvMass2
        
            if totalInvMass > EPSILON then
                let gravityForceB1 =
                    if finalInvMass1 > 0.0 then
                        (1.0 / finalInvMass1) * Vector3.Dot(-GRAVITY, normal)
                    else
                        0.0
                let gravityForceB2 =
                    if finalInvMass2 > 0.0 then
                        (1.0 / finalInvMass2) * Vector3.Dot(GRAVITY, normal)
                    else
                        0.0
                        
                let gravityImpulse = (max gravityForceB1 gravityForceB2) * dt

                let effectiveNormalImpulse = normalImpulseSum + gravityImpulse
        
                if effectiveNormalImpulse > 0.0 then
                    let frictionNormal = getStableFrictionNormal normal &b1 &b2

                    let relativeVelocity = b2.Velocity - b1.Velocity
                    let tangentVelocity = relativeVelocity - (Vector3.Dot(relativeVelocity, frictionNormal) * frictionNormal)

                    if tangentVelocity.MagnitudeSq() > EPSILON_X2 then
                        let tangentDirection = tangentVelocity.Normalize()
                        let j_t_scalar = -Vector3.Dot(relativeVelocity, tangentDirection) / totalInvMass
                        let combinedFriction = (b1.FrictionCoefficient + b2.FrictionCoefficient) * 0.5

                        let maxFrictionImpulse = combinedFriction * effectiveNormalImpulse
                        let frictionImpulseScalar = Math.Clamp(j_t_scalar, -maxFrictionImpulse, maxFrictionImpulse)
                        let frictionImpulse = frictionImpulseScalar * tangentDirection

                        b1.Velocity <- b1.Velocity - frictionImpulse * finalInvMass1
                        b2.Velocity <- b2.Velocity + frictionImpulse * finalInvMass2
                        
        let inline private resolveStaticCollision
            (b1: byref<Body.T>)
            (result: CollisionResult) =
            if b1.IsFallingOver then
                0.0
            else
                let struct(finalNormal, penetrationDepth) = result.Normalize()
                let velAlongNormal = Vector3.Dot(b1.Velocity, finalNormal)
                let mutable totalImpulseScalar = 0.0

                if velAlongNormal < 0.0 then
                    let e = if abs velAlongNormal < RESTITUTION_THRESHOLD then 0.0 else 0.1
                    
                    let velocityToCancel = finalNormal * velAlongNormal
                    b1.Velocity <- b1.Velocity - velocityToCancel * (1.0 + e)

                    let m1 = if b1.InvMass > EPSILON then 1.0 / b1.InvMass else 1e12
                    totalImpulseScalar <- m1 * (velocityToCancel * (1.0 + e)).Magnitude()
                    
                let invMass1 = b1.InvMass
                if invMass1 > EPSILON && penetrationDepth > PENETRATION_SLOP then
                    let correctionMagnitude = penetrationDepth * CORRECTION_PERCENT
                    let correctionVector = finalNormal * correctionMagnitude
                    b1.Position <- b1.Position + correctionVector
                    WorldLimits.wrapPosition &b1.Position
                    
                totalImpulseScalar
        
        let inline private resolveStaticFriction
            (b1: byref<Body.T>) 
            (normal: Vector3)
            normalImpulseSum
            invMass1
            dt =
            if invMass1 > EPSILON then
                let gravityForce = if invMass1 > 0.0 then (1.0 / invMass1) * Vector3.Dot(-GRAVITY, normal) else 0.0
                let gravityImpulse = gravityForce * dt
                let effectiveNormalImpulse = normalImpulseSum + gravityImpulse

                if effectiveNormalImpulse > 0.0 then
                    let staticBody = Body.T()
                    let frictionNormal = getStableFrictionNormal normal &b1 &staticBody

                    let relativeVelocity = b1.Velocity
                    let tangentVelocity = relativeVelocity - (Vector3.Dot(relativeVelocity, frictionNormal) * frictionNormal)
                    
                    if tangentVelocity.MagnitudeSq() > EPSILON_X2 then
                        let tangentDirection = tangentVelocity.Normalize()
                        let j_t_scalar = -Vector3.Dot(relativeVelocity, tangentDirection) / invMass1
                        
                        let maxFrictionImpulse = b1.FrictionCoefficient * effectiveNormalImpulse
                        let frictionImpulseScalar = Math.Clamp(j_t_scalar, -maxFrictionImpulse, maxFrictionImpulse)
                        let frictionImpulse = frictionImpulseScalar * tangentDirection
                        
                        b1.Velocity <- b1.Velocity + frictionImpulse * invMass1
        
        let private handleDynamicBody
            bodyRepo
            (kinematicBody: byref<Body.T>)
            dynamicBodyId
            islandRepo=
                
            if dynamicBodyId <> kinematicBody.Id then
                let otherBody = &Body.getRef dynamicBodyId bodyRepo
                if not <| Unsafe.IsNullRef &otherBody then
                    let result =
                        Collision.checkCollisionSAT
                            kinematicBody.Position
                            kinematicBody.Dimensions
                            kinematicBody.Orientation
                            otherBody.Position
                            otherBody.Dimensions
                            otherBody.Orientation
                    
                    if result.AreColliding then
                        let otherIsland = &Island.getIslandRefForBody dynamicBodyId islandRepo
                        Island.requestWakeIsland &otherIsland islandRepo

                        // Считаем коллизию, где у кинематического тела инвертированная масса равна 0,
                        // а у динамического его реальная инвертированная масса
                        // Это заставит динамическое тело отреагировать на удар, а кинематическое положить болт
                        resolveDynamicCollision
                            &kinematicBody
                            &otherBody
                            result
                            0.0
                            otherBody.InvMass
                        |> ignore

                        otherBody.IsSnappedToGrid <- false
        
        let private processCollidingBody
            bodyRepo
            (kinematicBody: inref<Body.T>)
            islandRepo
            finalPosition
            finalDimensions
            finalOrientation
            otherId =
            
            if otherId <> kinematicBody.Id then
                let otherBody = &Body.getRef otherId bodyRepo
                if not <| Unsafe.IsNullRef &otherBody then
                    let ghostResult =
                        Collision.checkCollisionSAT
                            finalPosition
                            finalDimensions
                            finalOrientation
                            otherBody.Position
                            otherBody.Dimensions
                            otherBody.Orientation
                    
                    if ghostResult.AreColliding then
                        let mutable ghostBody = Body.T()
                        ghostBody.Position <- finalPosition
                        ghostBody.Dimensions <- finalDimensions
                        ghostBody.Orientation <- finalOrientation
                        ghostBody.Velocity <- Vector3.Zero

                        resolveDynamicCollision
                            &ghostBody
                            &otherBody
                            ghostResult
                            0.0
                            otherBody.InvMass
                        |> ignore

                        WorldLimits.wrapPosition &otherBody.Position

                        let island = &Island.getIslandRefForBody otherId islandRepo
                        Island.requestWakeIsland &island islandRepo
        
        let private applyKinematicUpdates sub_dt engine =
            
            let islandRepo = engine._islandRepo
            let bodyRepo = engine._bodyRepo
            let buffers = engine._buffers
            let geometryRepo = engine._geometryRepo
            let activeHash = engine._activeHash
            let sleepingHash = engine._sleepingHash
            
            use bodiesToInitiateFall = new PooledList<int>()
            let activeIslands = islandRepo |> Island.getActiveIslandIds
            for islandId in activeIslands do
                let island = &Island.getIslandRef islandId islandRepo
                for id in island.Bodies do
                    let body = &Body.getRef id bodyRepo
                    if not <| Unsafe.IsNullRef &body && body.IsForceFalling && not body.IsFallingOver then
                        bodiesToInitiateFall.Add id
            
            for id in bodiesToInitiateFall.Span do
                let body = &Body.getRef id bodyRepo
                if not <| Unsafe.IsNullRef &body then
                    let island = &Island.getIslandRefForBody body.Id islandRepo
                    Island.requestWakeIsland &island islandRepo
                    
                    body.IsFallingOver <- true
                    body.FallRotationProgress <- 0.0
                    body.IsSnappedToGrid <- false
                    body.IsForceFalling <- false

            for islandId in activeIslands do
                let island = &Island.getIslandRef islandId islandRepo
                for id in island.Bodies do
                    let body = &Body.getRef id bodyRepo
                    if not <| Unsafe.IsNullRef &body && body.IsFallingOver then                
                        let previousPosition = body.Position
                        body.FallRotationProgress <- min (body.FallRotationProgress + sub_dt / body.FallDuration) 1.0

                        let totalRotationMatrix = Matrix3x3.CreateRotation(body.FallRotationAxis, (PI / 2.0) * body.FallRotationProgress)
     
                        body.Orientation <- totalRotationMatrix * body.FallInitialOrientation

                        let mutable idealNewPosition = body.FallPivotPoint + totalRotationMatrix * body.InitialCenterOffsetFromPivot
                        WorldLimits.wrapPosition &idealNewPosition
                        body.Position <- idealNewPosition

                        let h = body.Dimensions / 2.0
                        let mutable lowestPointZ = Double.MaxValue
                        for i = 0 to 7 do
                            let signX = if (i &&& 1) = 0 then -1.0 else 1.0
                            let signY = if (i &&& 2) = 0 then -1.0 else 1.0
                            let signZ = if (i &&& 4) = 0 then -1.0 else 1.0
                            let localCorner = Vector3(h.X * signX, h.Y * signY, h.Z * signZ)
                            let worldCorner = body.Position + body.Orientation * localCorner
                            lowestPointZ <- min lowestPointZ worldCorner.Z
                        
                        let pivotZ = body.FallPivotPoint.Z
                        if lowestPointZ < pivotZ then
                            let correctionZ = pivotZ - lowestPointZ
                            body.Position.Z <- body.Position.Z + correctionZ
                            
                        let mutable delta = body.Position - previousPosition
                        WorldLimits.relative &delta
                       
                        if sub_dt > EPSILON then
                            body.Velocity <- delta / sub_dt
                        else
                            body.Velocity <- Vector3.Zero
                            
                        let mutable hasStoppedOnStatic = false
                        let mutable penetrationVector = Vector3.Zero

                        Grid.fillOverlappingSubPrismsAABB
                            body.Position
                            body.Dimensions
                            body.Orientation
                            buffers.UniquePrismsFilterBuffer
                            buffers.UniquePrismsBuffer

                        let mutable cellIdx = 0
                        while not hasStoppedOnStatic && cellIdx < buffers.UniquePrismsBuffer.Count do
                            let cellKey = buffers.UniquePrismsBuffer[cellIdx]
                            if geometryRepo |> Geometry.isSolid cellKey then
                                let struct (staticPos, staticDims, staticOrient) = cellKey |> Grid.getPrismSpaceByKey 
                                let result =
                                    Collision.checkCollisionSAT
                                        body.Position
                                        body.Dimensions
                                        body.Orientation
                                        staticPos
                                        staticDims
                                        staticOrient
                                        
                                if result.AreColliding then
                                    let struct(_, penetrationDepth) = result.Normalize()
                                    if penetrationDepth > PENETRATION_SLOP then
                                        hasStoppedOnStatic <- true
                                        penetrationVector <- result.PenetrationVector
                            cellIdx <- cellIdx + 1

                        for cellKey in buffers.UniquePrismsBuffer.Span do
                            for otherId in SpatialHash.query cellKey activeHash do
                                handleDynamicBody bodyRepo &body otherId islandRepo
                            for otherId in SpatialHash.query cellKey sleepingHash do
                                handleDynamicBody bodyRepo &body otherId islandRepo

                        let animationFinished = body.FallRotationProgress >= (1.0 - EPSILON)
                        let shouldFinalize = hasStoppedOnStatic || animationFinished

                        if shouldFinalize then
                            
                            let fallDirection = Vector3.Cross(body.FallRotationAxis, Vector3.Up)
                            let finalFallDirection =
                                if fallDirection.MagnitudeSq() < EPSILON_X2 then
                                    Vector3.UnitX
                                else
                                    fallDirection.Normalize()
                                    
                            let newX_axis = finalFallDirection
                            let newZ_axis = Vector3.Up
                            let newY_axis = Vector3.Cross(newZ_axis, newX_axis).Normalize()
                            let finalNewX_axis = Vector3.Cross(newY_axis, newZ_axis)
                            let finalOrientation = Matrix3x3(finalNewX_axis, newY_axis, newZ_axis)

                            let oldDims = body.Dimensions
                            let finalDimensions =
                                let localRotationAxis = body.FallInitialOrientation.Transpose() * body.FallRotationAxis
                                if abs(localRotationAxis.X) > abs(localRotationAxis.Y) then
                                    Vector3(oldDims.X, oldDims.Z, oldDims.Y)
                                else
                                    Vector3(oldDims.Z, oldDims.Y, oldDims.X)

                            let mutable finalPosition = Vector3.Zero

                            if hasStoppedOnStatic then
                                finalPosition <- body.Position + penetrationVector
                            else
                                let h = finalDimensions / 2.0
                                let surfaceZ = body.FallPivotPoint.Z
                                finalPosition <- Vector3(body.Position.X, body.Position.Y, surfaceZ + h.Z)

                            Grid.fillOverlappingSubPrismsAABB
                                finalPosition
                                finalDimensions
                                finalOrientation
                                buffers.UniquePrismsFilterBuffer
                                buffers.UniquePrismsBuffer

                            for cellKey in buffers.UniquePrismsBuffer.Span do
                                for otherId in SpatialHash.query cellKey activeHash do
                                    processCollidingBody
                                        bodyRepo
                                        &body
                                        islandRepo
                                        finalPosition
                                        finalDimensions
                                        finalOrientation
                                        otherId

                                for otherId in SpatialHash.query cellKey sleepingHash do
                                    processCollidingBody
                                        bodyRepo
                                        &body
                                        islandRepo
                                        finalPosition
                                        finalDimensions
                                        finalOrientation
                                        otherId

                            body.IsFallingOver <- false
                            body.FallRotationProgress <- 1.0
                            body.Orientation <- finalOrientation
                            body.Dimensions <- finalDimensions
                            body.Position <- finalPosition
                            body.Velocity <- Vector3.Zero
                            WorldLimits.wrapPosition &body.Position
                            Body.updateAABB &body

        let inline private checkBodyProximity(minA: Vector3) (maxA: Vector3) (minB: Vector3) (maxB: Vector3) =
            let expansion = PENETRATION_SLOP * 4.0

            let zProx = (maxA.Z + expansion) >= (minB.Z - expansion) && (minA.Z - expansion) <= (maxB.Z + expansion)

            if not <| zProx then
                false
            else
                let inline axisProximity minValA maxValA minValB maxValB worldSize =
                    let centerA = (minValA + maxValA) * 0.5
                    let extentA = (maxValA - minValA) * 0.5
                    let centerB = (minValB + maxValB) * 0.5
                    let extentB = (maxValB - minValB) * 0.5

                    let mutable dist = centerA - centerB
                    let halfWorldSize = worldSize * 0.5
                    if dist > halfWorldSize then dist <- dist - worldSize
                    elif dist < -halfWorldSize then dist <- dist + worldSize

                    abs(dist) <= (extentA + extentB + expansion)

                if not <| axisProximity minA.X maxA.X minB.X maxB.X WorldLimits.X then
                    false
                else
                    axisProximity minA.Y maxA.Y minB.Y maxB.Y WorldLimits.Y
                        
        let private resolveFloorAndCeilingCollisions (b1: byref<Body.T>) invMass1 islandRepo dt =
            if invMass1 > EPSILON then
                let h = b1.Dimensions / 2.0
                
                // Проверка столкновения с полом
                let mutable lowestPointZ = Double.MaxValue
                let worldAABB_minZ = b1.Position.Z - b1.Dimensions.Z * 0.5
                
                if worldAABB_minZ < PENETRATION_SLOP then
                    for i = 0 to 7 do
                        let signX = if (i &&& 1) = 0 then -1.0 else 1.0
                        let signY = if (i &&& 2) = 0 then -1.0 else 1.0
                        let signZ = if (i &&& 4) = 0 then -1.0 else 1.0
                        let localCorner = Vector3(h.X * signX, h.Y * signY, h.Z * signZ)
                        let worldCorner = b1.Position + b1.Orientation * localCorner
                        lowestPointZ <- min lowestPointZ worldCorner.Z
                else
                    lowestPointZ <- worldAABB_minZ
                
                if lowestPointZ < 0.0 then
                    let penetrationDepth = -lowestPointZ
                    let floorCollisionResult = CollisionResult(Vector3.Up * penetrationDepth)
                    let mutable floorBody = Body.T()
                    let totalImpulseScalar =
                        resolveDynamicCollision
                            &b1
                            &floorBody
                            floorCollisionResult
                            invMass1
                            0.0
                            
                    let struct(finalNormal, _) = floorCollisionResult.Normalize() 
                    resolveStaticFriction &b1 finalNormal totalImpulseScalar invMass1 dt
                    
                    let island = &Island.getIslandRefForBody b1.Id islandRepo
                    if not <| Unsafe.IsNullRef &island then
                        island.UpdateMaxPenetration penetrationDepth
                        island.IsGrounded <- true
                        
                // Проверка столкновения с потолком
                let mutable highestPointZ = Double.MinValue
                let worldAABB_maxZ = b1.Position.Z + b1.Dimensions.Z * 0.5
                if worldAABB_maxZ > WORLD_HEIGHT_IN_METERS - PENETRATION_SLOP then
                    for i = 0 to 7 do
                        let signX = if (i &&& 1) = 0 then -1.0 else 1.0
                        let signY = if (i &&& 2) = 0 then -1.0 else 1.0
                        let signZ = if (i &&& 4) = 0 then -1.0 else 1.0
                        let localCorner = Vector3(h.X * signX, h.Y * signY, h.Z * signZ)
                        let worldCorner = b1.Position + b1.Orientation * localCorner
                        highestPointZ <- max highestPointZ worldCorner.Z
                else
                    highestPointZ <- worldAABB_maxZ

                let topPenetration = highestPointZ - WORLD_HEIGHT_IN_METERS
                if topPenetration > 0.0 then
                    let mutable floorBody = Body.T()
                    let ceilingCollisionResult = CollisionResult(Vector3.Down * topPenetration)
                    resolveDynamicCollision &b1 &floorBody ceilingCollisionResult b1.InvMass 0.0 |> ignore
        
        let private resolveDynamicDynamicCollision
            (b1: byref<Body.T>)
            (b2: byref<Body.T>)
            islandRepo
            dt =
            
            let minA = b1.MinAABB
            let maxA = b1.MaxAABB
            let minB = b2.MinAABB
            let maxB = b2.MaxAABB
            
            if Collision.checkCollisionAABB minA maxA minB maxB then
                let island1 = &Island.getIslandRefForBody b1.Id islandRepo
                let contactKey = ContactKey.key b1.Id b2.Id
                let mutable cachedAxis = -1
                island1.TryGetCachedAxis(contactKey, &cachedAxis) |> ignore

                let struct(result, newCachedAxis) =
                    Collision.checkCollisionSATWithCachedAxis
                        b1.Position
                        b1.Dimensions
                        b1.Orientation
                        b2.Position
                        b2.Dimensions
                        b2.Orientation
                        cachedAxis
                    
                if result.AreColliding then
                    let struct(finalNormal, penetrationDepth) = result.Normalize()

                    let stableResult = CollisionResult(finalNormal * penetrationDepth)
                    let totalImpulseScalar =
                        resolveDynamicCollision
                            &b1
                            &b2
                            stableResult
                            b1.InvMass
                            b2.InvMass
                                                
                    resolveDynamicFriction
                        &b1
                        &b2
                        finalNormal
                        totalImpulseScalar
                        b1.InvMass
                        b2.InvMass
                        dt

                    let island2 = &Island.getIslandRefForBody b2.Id islandRepo
                    
                    let contactKey = ContactKey.key b1.Id b2.Id
                    island1.AddOrUpdateContact(contactKey, CONTACT_TTL, newCachedAxis)
                    island1.UpdateMaxPenetration penetrationDepth
                    
                    if island1.Id <> island2.Id then
                        islandRepo |> Island.requestMerge island1.Id island2.Id
                        island2.AddOrUpdateContact(contactKey, CONTACT_TTL, newCachedAxis)                              
                        island2.UpdateMaxPenetration penetrationDepth
                        
                    b1.IsSnappedToGrid <- false
                    b2.IsSnappedToGrid <- false
                elif checkBodyProximity minA maxA minB maxB then
                    let island1 = &Island.getIslandRefForBody b1.Id islandRepo
                    let island2 = &Island.getIslandRefForBody b2.Id islandRepo

                    if island1.Id = island2.Id then
                        let contactKey = ContactKey.key b1.Id b2.Id
                        island1.AddOrUpdateContact(contactKey, CONTACT_TTL, newCachedAxis)
        
        let private resolveDynamicSleepingCollision
            (b1: byref<Body.T>)
            (b2: byref<Body.T>)
            islandRepo
            dt =
            let minA = b1.MinAABB
            let maxA = b1.MaxAABB
            let minB = b2.MinAABB
            let maxB = b2.MaxAABB
            
            if Collision.checkCollisionAABB minA maxA minB maxB then
                let island1 = &Island.getIslandRefForBody b1.Id islandRepo
                let contactKey = ContactKey.key b1.Id b2.Id
                let mutable cachedAxis = -1
                island1.TryGetCachedAxis(contactKey, &cachedAxis) |> ignore
        
                let struct(result, newCachedAxis) =
                    Collision.checkCollisionSATWithCachedAxis
                        b1.Position
                        b1.Dimensions
                        b1.Orientation
                        b2.Position
                        b2.Dimensions
                        b2.Orientation
                        cachedAxis
                        
                if result.AreColliding then
                    let struct(finalNormal, penetrationDepth) = result.Normalize()
                    let stableResult = CollisionResult(finalNormal * penetrationDepth)
                                                    
                    let totalImpulseScalar =
                        resolveDynamicCollision
                            &b1
                            &b2
                            stableResult
                            b1.InvMass
                            b2.InvMass
                                                    
                    resolveDynamicFriction
                        &b1
                        &b2
                        finalNormal
                        totalImpulseScalar
                        b1.InvMass
                        b2.InvMass
                        dt

                    let island2 = &Island.getIslandRefForBody b2.Id islandRepo
                                                    
                    Island.requestWakeIsland &island2 islandRepo
                    islandRepo |> Island.requestMerge island2.Id island1.Id

                    let contactKey = ContactKey.key b1.Id b2.Id
                    island1.AddOrUpdateContact(contactKey, CONTACT_TTL, newCachedAxis)
                    island1.UpdateMaxPenetration penetrationDepth
                    island2.AddOrUpdateContact(contactKey, CONTACT_TTL, newCachedAxis)
                    island2.UpdateMaxPenetration penetrationDepth
                    
                    b1.IsSnappedToGrid <- false
                    b2.IsSnappedToGrid <- false
        
        let private resolveStaticGeometryCollision (b1: byref<Body.T>) cellKey islandRepo dt =
            let struct (pos, dims, orient) = cellKey |> Grid.getPrismSpaceByKey 
            let result =
                Collision.checkCollisionSAT
                    b1.Position
                    b1.Dimensions
                    b1.Orientation
                    pos
                    dims
                    orient

            if result.AreColliding then
                let struct(finalNormal, penetrationDepth) = result.Normalize()
                let stableResult = CollisionResult(finalNormal * penetrationDepth)

                let totalImpulseScalar = resolveStaticCollision &b1 stableResult
                                        
                resolveStaticFriction &b1 finalNormal totalImpulseScalar b1.InvMass dt
                let island = &Island.getIslandRefForBody b1.Id islandRepo
                if not <| Unsafe.IsNullRef &island then
                    island.UpdateMaxPenetration penetrationDepth
                    island.IsGrounded <- true
                    
        let private resolveFloraCollision
            (b1: byref<Body.T>)
            treeId
            floraRepo
            (island: byref<Island.T>)
            buffers
            (rnd: Random)
            dt =
            let treeData = &Flora.tryGetTreeDataRef treeId floraRepo
            if not <| Unsafe.IsNullRef &treeData && not <| treeData.IsDestroyed then
                let result =
                    Collision.checkCollisionSAT
                        b1.Position
                        b1.Dimensions
                        b1.Orientation
                        treeData.Position
                        treeData.Dimensions
                        Matrix3x3.Identity
                                                    
                if result.AreColliding then
                    let impact = b1.Mass * b1.Velocity.Magnitude()
                                                    
                    if impact > treeData.BreakThreshold then
                        treeData.IsDestroyed <- true
                        buffers.CollisionDestroyedFlora.Add treeId |> ignore
                        let mutable newBody = Flora.createBodyFromFlora treeData.Id treeData
                        let hVel = Vector3(b1.Velocity.X, b1.Velocity.Y, 0.0)
                        let fallDirection =
                            if hVel.MagnitudeSq() < EPSILON then
                                Vector3(cos(rnd.NextDouble()*2.0*PI), sin(rnd.NextDouble()*2.0*PI), 0.0)
                            else
                                hVel.Normalize()
                        let cross = Vector3.Cross(Vector3.Up, fallDirection)
                        let fallAxis =
                            if cross.MagnitudeSq() < EPSILON then
                                Vector3.UnitX
                            else
                                cross.Normalize()
                                                                
                        let fallDuration = max 0.5 (treeData.Dimensions.Magnitude() / 5.0)
                        newBody.IsFallingOver <- true
                        newBody.FallInitialOrientation <- newBody.Orientation
                        newBody.FallPivotPoint <- newBody.Position - newBody.Orientation.R2 * (newBody.Dimensions.Z / 2.0)
                        newBody.InitialCenterOffsetFromPivot <- newBody.Position - newBody.FallPivotPoint
                        newBody.FallRotationAxis <- fallAxis
                        newBody.FallDuration <- fallDuration
                        buffers.BodiesToAdd.Add newBody
                        buffers.FloraToRemoveIds.Add treeData.Id
                    else
                        let struct(finalNormal, penetrationDepth) = result.Normalize()
                        island.UpdateMaxPenetration penetrationDepth
                        let stableResult = CollisionResult(finalNormal * penetrationDepth)
                        let totalImpulseScalar = resolveStaticCollision &b1 stableResult
                        resolveStaticFriction &b1 finalNormal totalImpulseScalar b1.InvMass dt

        let private detectCollisionsAndUpdateIslands sub_dt engine =
            
            let buffers = engine._buffers
            let islandRepo = engine._islandRepo
            let bodyRepo = engine._bodyRepo
            let geometryRepo = engine._geometryRepo
            let activeHash = engine._activeHash
            let sleepingHash = engine._sleepingHash
            let floraRepo = engine._floraRepo
            let rnd = engine._random

            let activeIslands = buffers.CollisionActiveIslandIds
            let checkedBodyPairs = buffers.CollisionCheckedBodyPairs   
            let processedStatic = buffers.CollisionProcessedStatic
            let processedFlora = buffers.CollisionProcessedFlora
            let collidingIslandPairs = buffers.CollisionIslandPairs
            let sleepingIslands = buffers.CollisionSleepingIslandIds
            
            activeIslands.Clear()
            sleepingIslands.Clear()
            checkedBodyPairs.Clear()
            collidingIslandPairs.Clear()
            
            activeIslands.AddRange(islandRepo |> Island.getActiveIslandIds)
            sleepingIslands.AddRange(islandRepo |> Island.getSleepingIslandIds)
            
            let activeIslandsSpan = activeIslands.Span
            let sleepingIslandsSpan = sleepingIslands.Span

            for i = 0 to activeIslandsSpan.Length - 1 do
                let activeId1 = activeIslandsSpan[i]
                let activeIsland1 = &Island.getIslandRef activeId1 islandRepo
                for id1 in activeIsland1.Bodies do
                    let b1 = &Body.getRef id1 bodyRepo
                    let invMass1 = if b1.IsFallingOver then 0.0 else b1.InvMass
                    resolveFloorAndCeilingCollisions &b1 invMass1 islandRepo sub_dt

                    let occupiedCells = SpatialHash.getOccupiedCells id1 activeHash

                    processedStatic.Clear()
                    processedFlora.Clear()
        
                    for cellKey in occupiedCells do
                        // Dynamic <-> Dynamic
                        for id2 in SpatialHash.query cellKey activeHash do
                            if id1 < id2 then
                                let activeIsland2 = &Island.getIslandRefForBody id2 islandRepo
                                if activeIsland1.Id = activeIsland2.Id then
                                    let contactKey = ContactKey.key id1 id2
                                    if checkedBodyPairs.Add contactKey then
                                        let b2 = &Body.getRef id2 bodyRepo
                                        resolveDynamicDynamicCollision &b1 &b2 islandRepo sub_dt
                        
                        // Dynamic <-> Static
                        if geometryRepo |> Geometry.isSolid cellKey then
                            if processedStatic.Add cellKey then
                                resolveStaticGeometryCollision &b1 cellKey islandRepo sub_dt
                                
                        // Dynamic <-> Flora
                        match floraRepo |> Flora.tryGetTreesInCell cellKey with
                        | true, treeIds ->
                            for treeId in treeIds.Span do
                                if not <| buffers.CollisionDestroyedFlora.Contains treeId && processedFlora.Add treeId then
                                    resolveFloraCollision &b1 treeId floraRepo &activeIsland1 buffers rnd sub_dt
                        | _ -> ()
                
                // Islands intersection filter
                for j = i + 1 to activeIslandsSpan.Length - 1 do
                    let activeId2 = activeIslandsSpan[j]
                    let activeIsland2 = &Island.getIslandRef activeId2 islandRepo
                    if Collision.checkCollisionAABB activeIsland1.MinAABB activeIsland1.MaxAABB activeIsland2.MinAABB activeIsland2.MaxAABB then
                        collidingIslandPairs.Add (ContactKey.key activeId1 activeId2) |> ignore
                        
                for j = 0 to sleepingIslandsSpan.Length - 1 do
                    let sleepingId = sleepingIslandsSpan[j]
                    let sleepingIsland = &Island.getIslandRef sleepingId islandRepo
                    if Collision.checkCollisionAABB activeIsland1.MinAABB activeIsland1.MaxAABB sleepingIsland.MinAABB sleepingIsland.MaxAABB then
                        collidingIslandPairs.Add (ContactKey.key activeId1 sleepingId) |> ignore
                        
            for pairKey in collidingIslandPairs do
                let struct(id1, id2) = ContactKey.unpack pairKey
                
                let mutable island1 = &Island.getIslandRef id1 islandRepo
                let mutable island2 = &Island.getIslandRef id2 islandRepo

                // We determine which pair: active-active or active-sleeping
                let activeIsland = if island1.IsAwake then &island1 else &island2
                let otherIsland = if island1.IsAwake then &island2 else &island1

                // We go through all the bodies of the active island from the pair
                for bodyId1 in activeIsland.Bodies do
                    let b1 = &Body.getRef bodyId1 bodyRepo
                    let occupiedCells = SpatialHash.getOccupiedCells bodyId1 activeHash
                    for cellKey in occupiedCells do
                        // We are looking for bodies from the second island (may be active or sleeping)
                        let otherHash = if otherIsland.IsAwake then activeHash else sleepingHash
                        for bodyId2 in SpatialHash.query cellKey otherHash do
                            // We make sure that the body belongs to the second island in the pair
                            if otherIsland.Bodies.Contains bodyId2 then
                                let contactKey = ContactKey.key bodyId1 bodyId2
                                if checkedBodyPairs.Add contactKey then
                                    let b2 = &Body.getRef bodyId2 bodyRepo
                                    if otherIsland.IsAwake then
                                        resolveDynamicDynamicCollision &b1 &b2 islandRepo sub_dt
                                    else
                                        resolveDynamicSleepingCollision &b1 &b2 islandRepo sub_dt
                        
        let private postProcessAndUpdateSleepState engine =
            let islandRepo = engine._islandRepo
            let bodyRepo = engine._bodyRepo
            let floraRepo = engine._floraRepo
            let activeHash = engine._activeHash
            let sleepingHash = engine._sleepingHash
            let dt = engine._dt
            let geometryRepo = engine._geometryRepo
            use processedBushes = new PooledSet<int>()
            
            for islandId in islandRepo |> Island.getActiveIslandIds do
                let island = &Island.getIslandRef islandId islandRepo
                if island.IsAwake then
                    if island.Bodies.Count > 1 && island.HadContactsRemovedThisStep && island.ContactCount < island.Bodies.Count - 1 then
                        islandRepo |> Island.requestSplit island.Id
                    let mutable isStillSlow = true
                    let mutable enumerator = island.Bodies.GetEnumerator()
                    while isStillSlow && enumerator.MoveNext() do
                        let bodyId = enumerator.Current
                        let body = &Body.getRef bodyId bodyRepo
                        if not <| Unsafe.IsNullRef &body then
                            processedBushes.Clear()
                            for pCoords in SpatialHash.getOccupiedCells bodyId activeHash do
                                match floraRepo |> Flora.tryGetBushesInCell pCoords with
                                | true, bushIds ->
                                    for bushId in bushIds.Span do
                                        processedBushes.Add bushId |> ignore
                                | _ -> ()

                            for bushId in processedBushes do
                                Flora.applyBushFriction &body bushId dt floraRepo
                                
                            let mSq = body.Velocity.MagnitudeSq()
                            if (mSq >= SLEEP_VELOCITY_THRESHOLD_SQ || body.IsFallingOver) then
                                isStillSlow <- false
                    
                    island.IsSlow <- isStillSlow
                    
                    if island.IsSlow then
                        if island.MaxPenetrationThisStep > MAX_PENETRATION then
                            island.FramesResting <- 0
                        elif island.IsGrounded then
                            island.FramesResting <- island.FramesResting + 1
                            
                            if island.FramesResting >= FRAMES_TO_SLEEP && island.CantSleepFrames <= 0 then
                                let mutable isIslandStable = true
                                let mutable fallCheckEnumerator = island.Bodies.GetEnumerator()
                                while isIslandStable && fallCheckEnumerator.MoveNext() do
                                    let bodyId = fallCheckEnumerator.Current
                                    let body = &Body.getRef bodyId bodyRepo
                                    if not <| Unsafe.IsNullRef &body then
                                        match tryInitiateFall &body geometryRepo bodyRepo activeHash sleepingHash with
                                        | ValueSome (struct (pivot, axis)) ->                                          
                                            isIslandStable <- false
                                            body.IsSnappedToGrid <- false

                                            let dims = body.Dimensions
                                            let flatnessThreshold = 1.0

                                            if dims.X > dims.Z * flatnessThreshold || dims.Y > dims.Z * flatnessThreshold then
                                                island.FramesResting <- FRAMES_TO_SLEEP - 1
                                                let mutable fallDirection = pivot - body.Position
                                                WorldLimits.relative &fallDirection
                                                fallDirection.Z <- 0.0

                                                if fallDirection.MagnitudeSq() > EPSILON then
                                                    let pushStrength = 1.6
                                                    let horizontalPush = fallDirection.Normalize()
                                                    let liftFactor = 0.4
                                                    let verticalLift = Vector3.Up * liftFactor
                                                    let combinedDirection = (horizontalPush + verticalLift).Normalize()
                                                    body.Velocity <- body.Velocity + combinedDirection * pushStrength                            
                                            else
                                                island.FramesResting <- 0
                                                body.IsFallingOver <- true
                                                body.FallRotationProgress <- 0.0
                                                body.FallInitialOrientation <- body.Orientation
                                                body.FallDuration <- max 0.5 (body.Dimensions.Magnitude() / 5.0)
                                                body.FallPivotPoint <- pivot
                                                body.FallRotationAxis <- axis

                                                let mutable offset = body.Position - pivot
                                                WorldLimits.relative &offset
                                                body.InitialCenterOffsetFromPivot <- offset                      
                                        | ValueNone -> ()
                                        
                                if isIslandStable then
                                    for bodyId in island.Bodies do
                                        let body = &Body.getRef bodyId bodyRepo
                                        if not <| Unsafe.IsNullRef &body then
                                            Body.updateAABB &body
                                            body.Velocity <- Vector3.Zero
                                    Island.requestSleep &island islandRepo
                                else 
                                    island.FramesResting <- 0
                        else
                            island.FramesResting <- 0 
                    else
                        island.FramesResting <- 0

                    if island.CantSleepFrames > 0 then
                        island.CantSleepFrames <- island.CantSleepFrames - 1
            
        let private applyDeferredChanges engine =
            let buffers = engine._buffers
            let geometryRepo = engine._geometryRepo
            let floraRepo = engine._floraRepo
            let bodyRepo = engine._bodyRepo
            let islandRepo = engine._islandRepo
            let activeHash = engine._activeHash
            let sleepingHash = engine._sleepingHash
            
            for coords in buffers.PrismsToRemoveCoords.Span do
                geometryRepo |> Geometry.removePrism coords |> ignore

            if buffers.FloraToRemoveIds.Count > 0 then
                for id in buffers.FloraToRemoveIds.Span do
                    floraRepo |> Flora.removeTree id
                    floraRepo |> Flora.removeBush id

                floraRepo |> Flora.updateHashesForRemovedFlora buffers.FloraToRemoveIds

            for id in buffers.BodiesToRemoveIds.Span do
                bodyRepo |> Body.remove id |> ignore
                islandRepo |> Island.removeBody id
                activeHash |> SpatialHash.remove id
                sleepingHash |> SpatialHash.remove id

            for body in buffers.BodiesToAdd.Span do
                Body.tryAdd &body bodyRepo |> ignore
                islandRepo |> Island.addBody body.Id

        let step engine =

            engine |> applyDeferredChanges

            engine._islandRepo |> Island.processIslandChanges engine._geometryRepo engine._buffers
            
            engine._buffers.Clear()

            for islandId in engine._islandRepo.ActiveIslandIds do
                let mutable island = &Island.getIslandRef islandId engine._islandRepo
                island.NextStep() 
                for bodyId in island.Bodies do
                    let body = &Body.getRef bodyId engine._bodyRepo
                    if not <| Unsafe.IsNullRef &body then
                        Body.updateAABB &body
                        SpatialHash.update &body engine._buffers engine._activeHash
                        if not <| body.IsFallingOver && not <| body.IsSnappedToGrid && body.IsGravityEnabled then
                            body.Velocity <- body.Velocity + GRAVITY * engine._dt

                Island.recalculateAABB &island engine.Bodies
            
            for body in engine._buffers.BodiesToAdd.Span do
                 let foundBody = &Body.getRef body.Id engine._bodyRepo
                 if not <| Unsafe.IsNullRef &foundBody then
                    Body.updateAABB &foundBody
                    SpatialHash.add &foundBody engine._buffers engine._activeHash

            let resolutionPasses = 8
            let sub_dt = engine._dt / double resolutionPasses

            for _ = 1 to resolutionPasses do
                engine |> applyKinematicUpdates sub_dt

                engine |> detectCollisionsAndUpdateIslands sub_dt

                for islandId in engine._islandRepo.ActiveIslandIds do
                    let island = &Island.getIslandRef islandId engine._islandRepo
                    for bodyId in island.Bodies do
                        let body = &Body.getRef bodyId engine._bodyRepo
                        if not <| Unsafe.IsNullRef &body && not <| body.IsFallingOver then                          
                            let vSq = body.Velocity.MagnitudeSq()
                            if vSq > MAX_SPEED_SQ then
                                body.Velocity <- body.Velocity.Normalize() * MAX_SPEED
                                
                            body.Position <- body.Position + body.Velocity * sub_dt
                            WorldLimits.wrapPosition &body.Position
                
            engine._floraRepo |> Flora.updatePhysics engine._buffers.UnrootedFlora  engine._buffers.FloraToRemoveIds
            engine._geometryRepo |> Geometry.updatePhysics engine._buffers.FallingPrisms engine._buffers.PrismsToRemoveCoords
            engine._liquidRepo |> Liquid.updatePhysics

            for data in engine._buffers.UnrootedFlora.Span do
                if not <| engine._buffers.BodiesToAdd.Exists(fun b -> b.Id = data.Id) then
                     let mutable newBody = Flora.createBodyFromFlora data.Id data
                     newBody.IsForceFalling <- true
                     engine._buffers.BodiesToAdd.Add newBody
                     
            for fallingPrism in engine._buffers.FallingPrisms.Span do
                if not <| engine._buffers.BodiesToAdd.Exists(fun b -> b.Id = fallingPrism.Id) then
                    engine._buffers.BodiesToAdd.Add fallingPrism

            engine |> postProcessAndUpdateSleepState