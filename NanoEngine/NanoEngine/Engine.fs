namespace NanoEngine
#nowarn "9"
#nowarn "3391"

open System.Diagnostics
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
    let [<Literal>] DISABLE_SLEEP_ISLAND_PENETRATION = PENETRATION_SLOP * 4.0
    let [<Literal>] CORRECTION_PERCENT = 0.65
    let [<Literal>] RESTITUTION_THRESHOLD = 2.0
    let [<Literal>] EPSILON = 1e-6
    let [<Literal>] EPSILON_X2 = EPSILON * EPSILON
    let [<Literal>] EPSILON_F = 1e-5f
    let [<Literal>] EPSILON_F_X2 = EPSILON_F * EPSILON_F
    let [<Literal>] SLEEP_VELOCITY_THRESHOLD_SQ = 0.01
    let [<Literal>] CONTACT_TTL = 3
    let [<Literal>] FRAMES_TO_SLEEP = CONTACT_TTL + 2
    let [<Literal>] PI = Math.PI
    let [<Literal>] SQRT3 = 1.7320508075688772
    let [<Literal>] MIN_DIMENSION_THRESHOLD = 0.001
    let [<Literal>] MAX_DIMENSION = 10.0
    let [<Literal>] GRID_WIDTH_Q = 65536
    let [<Literal>] GRID_WIDTH_Q_DOUBLE = 65536.0
    let [<Literal>] GRID_DEPTH_R = 65536
    let [<Literal>] GRID_DEPTH_R_DOUBLE = 65536.0
    let [<Literal>] GRID_HEIGHT_Z = 248
    let [<Literal>] HEX_HEIGHT = 1.0
    let [<Literal>] WORLD_HEIGHT_IN_METERS = 248.0 * 1.0
    let [<Literal>] HEX_WIDTH = 1.0
    let [<Literal>] HEX_RADIUS = HEX_WIDTH / SQRT3
    let [<Literal>] DYNAMIC_SUPPORT_PROBE_RADIUS = MIN_DIMENSION_THRESHOLD
    let [<Literal>] STATIC_SUPPORT_PROBE_RADIUS = HEX_RADIUS * 0.2
    let [<Literal>] MAX_SPEED = 10.0
    let [<Literal>] MAX_SPEED_SQ = MAX_SPEED * MAX_SPEED
    let [<Literal>] FRAMES_TO_UNROOT = 2
    let [<Literal>] POSITION_THRESHOLD_SQ = (HEX_RADIUS * 0.04) * (HEX_RADIUS * 0.04)
    let [<Literal>] ORIENTATION_THRESHOLD = 0.9998 // cos(1.14 градуса)
    let [<Literal>] STATIC_BODY_MASS = 1e12

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
                this * (1.0 / (sqrt mSq))

    
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
    
    [<RequireQualifiedAccess>]
    module MathF =
        type Vector3f = System.Numerics.Vector3

        [<Struct; IsReadOnly>]
        type Matrix3x3f =
            val R0: Vector3f
            val R1: Vector3f
            val R2: Vector3f

            new(r0, r1, r2) = { R0 = r0; R1 = r1; R2 = r2 }
            
            static member inline Identity =
                Matrix3x3f(Vector3f.UnitX, Vector3f.UnitY, Vector3f.UnitZ)

            member inline this.Transform(v: Vector3f) =
                Vector3f(Vector3f.Dot(this.R0, v), Vector3f.Dot(this.R1, v), Vector3f.Dot(this.R2, v))
                
        let inline toFloatV3 (v: Vector3) = Vector3f(float32 v.X, float32 v.Y, float32 v.Z)

        let inline toDoubleV3 (v: Vector3f) = Vector3(float v.X, float v.Y, float v.Z)

        let inline toFloatM3 (m: Matrix3x3) =
            Matrix3x3f(toFloatV3 m.R0, toFloatV3 m.R1, toFloatV3 m.R2)
    
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

    [<Struct; IsReadOnly; StructLayout(LayoutKind.Sequential)>]
    type CollisionResult =
        new(n: Vector3, d: double, areColliding: bool) = { Normal = n; Depth = d; AreColliding = areColliding }

        static member inline Create(normal, depth) = CollisionResult(normal, depth, true)
        static member inline NoCollision = CollisionResult(Vector3.Zero, 0.0, false)
        
        val AreColliding: bool
        val Normal: Vector3
        val Depth: double

    [<RequireQualifiedAccess>]
    module WorldLimits =
        let [<Literal>] X = HEX_RADIUS * SQRT3 * GRID_WIDTH_Q_DOUBLE
        let [<Literal>] Y = HEX_RADIUS * 1.5 * GRID_DEPTH_R_DOUBLE
        let [<Literal>] HalfX = X / 2.0
        let [<Literal>] HalfY = Y / 2.0
            
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
            if value > HalfX then value - X
            elif value < -HalfX then value + X
            else value
            
        let inline relativeY value =
            if value > HalfY then value - Y
            elif value < -HalfY then value + Y
            else value
            
        let inline relative (delta: byref<Vector3>) =
            delta.X <- relativeX delta.X
            delta.Y <- relativeY delta.Y
    
    [<RequireQualifiedAccess>]
    module Collision =

        let inline getProjectedRadiusZ (dimensions: Vector3) (orientation: Matrix3x3) =
            (dimensions.X / 2.0) * (abs orientation.R0.Z) +
            (dimensions.Y / 2.0) * (abs orientation.R1.Z) +
            (dimensions.Z / 2.0) * (abs orientation.R2.Z)
    
        let getAABB (position: Vector3) (dimensions: Vector3) (orientation: Matrix3x3) =
            
            let h = dimensions / 2.0

            let m11 = abs orientation.R0.X
            let m12 = abs orientation.R1.X
            let m13 = abs orientation.R2.X
            let m21 = abs orientation.R0.Y
            let m22 = abs orientation.R1.Y
            let m23 = abs orientation.R2.Y
            let m31 = abs orientation.R0.Z
            let m32 = abs orientation.R1.Z
            let m33 = abs orientation.R2.Z
            
            let hX = h.X
            let hY = h.Y
            let hZ = h.Z

            let radiusX = hX * m11 + hY * m12 + hZ * m13
            let radiusY = hX * m21 + hY * m22 + hZ * m23
            let radiusZ = hX * m31 + hY * m32 + hZ * m33

            let minCorner =
                Vector3(
                    position.X - radiusX,
                    position.Y - radiusY,
                    position.Z - radiusZ
                )
                
            let maxCorner =
                Vector3(
                    position.X + radiusX,
                    position.Y + radiusY,
                    position.Z + radiusZ
                )
            
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

        [<Struct; IsByRefLike; IsReadOnly>]
        type private AxisTestResultF =
            val IsSeparating: bool
            val PenetrationSq: float32
            val Axis: MathF.Vector3f
            val IsValidAxis: bool
            new(isSeparating, penetrationSq, axis, isValidAxis) =
                {
                    IsSeparating = isSeparating
                    PenetrationSq = penetrationSq
                    Axis = axis
                    IsValidAxis = isValidAxis
                }

        let inline private testSingleAxis
            (index: int)
            (h1: MathF.Vector3f) (h2: MathF.Vector3f) (delta: MathF.Vector3f)
            (o1: MathF.Matrix3x3f) (o2: MathF.Matrix3x3f)
            (ac00: float32) (ac01: float32) (ac02: float32)
            (ac10: float32) (ac11: float32) (ac12: float32)
            (ac20: float32) (ac21: float32) (ac22: float32)
            (t_x: float32) (t_y: float32) (t_z: float32) =

            let mutable sep, R, axis, Lsq = 0.0f, 0.0f, MathF.Vector3f.Zero, 1.0f

            match index with
            | 0 -> sep <- MathF.Abs t_x; R <- h1.X + h2.X * ac00 + h2.Y * ac01 + h2.Z * ac02; axis <- o1.R0
            | 1 -> sep <- MathF.Abs t_y; R <- h1.Y + h2.X * ac10 + h2.Y * ac11 + h2.Z * ac12; axis <- o1.R1
            | 2 -> sep <- MathF.Abs t_z; R <- h1.Z + h2.X * ac20 + h2.Y * ac21 + h2.Z * ac22; axis <- o1.R2
            | 3 -> sep <- MathF.Abs (MathF.Vector3f.Dot(delta, o2.R0)); R <- h1.X * ac00 + h1.Y * ac10 + h1.Z * ac20 + h2.X; axis <- o2.R0
            | 4 -> sep <- MathF.Abs (MathF.Vector3f.Dot(delta, o2.R1)); R <- h1.X * ac01 + h1.Y * ac11 + h1.Z * ac21 + h2.Y; axis <- o2.R1
            | 5 -> sep <- MathF.Abs (MathF.Vector3f.Dot(delta, o2.R2)); R <- h1.X * ac02 + h1.Y * ac12 + h1.Z * ac22 + h2.Z; axis <- o2.R2
            | 6 -> axis <- MathF.Vector3f.Cross(o1.R0, o2.R0); Lsq <- axis.LengthSquared()
            | 7 -> axis <- MathF.Vector3f.Cross(o1.R0, o2.R1); Lsq <- axis.LengthSquared()
            | 8 -> axis <- MathF.Vector3f.Cross(o1.R0, o2.R2); Lsq <- axis.LengthSquared()
            | 9 -> axis <- MathF.Vector3f.Cross(o1.R1, o2.R0); Lsq <- axis.LengthSquared()
            | 10 -> axis <- MathF.Vector3f.Cross(o1.R1, o2.R1); Lsq <- axis.LengthSquared()
            | 11 -> axis <- MathF.Vector3f.Cross(o1.R1, o2.R2); Lsq <- axis.LengthSquared()
            | 12 -> axis <- MathF.Vector3f.Cross(o1.R2, o2.R0); Lsq <- axis.LengthSquared()
            | 13 -> axis <- MathF.Vector3f.Cross(o1.R2, o2.R1); Lsq <- axis.LengthSquared()
            | 14 -> axis <- MathF.Vector3f.Cross(o1.R2, o2.R2); Lsq <- axis.LengthSquared()
            | _ -> ()

            if Lsq > EPSILON_F_X2 then
                if index > 5 then
                    sep <- MathF.Abs (MathF.Vector3f.Dot(delta, axis))
                    match index with
                    | 6 -> R <- h1.Y * ac20 + h1.Z * ac10 + h2.Y * ac02 + h2.Z * ac01
                    | 7 -> R <- h1.Y * ac21 + h1.Z * ac11 + h2.X * ac02 + h2.Z * ac00
                    | 8 -> R <- h1.Y * ac22 + h1.Z * ac12 + h2.X * ac01 + h2.Y * ac00
                    | 9 -> R <- h1.X * ac20 + h1.Z * ac00 + h2.Y * ac12 + h2.Z * ac11
                    | 10 -> R <- h1.X * ac21 + h1.Z * ac01 + h2.X * ac12 + h2.Z * ac10
                    | 11 -> R <- h1.X * ac22 + h1.Z * ac02 + h2.X * ac11 + h2.Y * ac10
                    | 12 -> R <- h1.X * ac10 + h1.Y * ac00 + h2.Y * ac22 + h2.Z * ac21
                    | 13 -> R <- h1.X * ac11 + h1.Y * ac01 + h2.X * ac22 + h2.Z * ac20
                    | 14 -> R <- h1.X * ac12 + h1.Y * ac02 + h2.X * ac21 + h2.Y * ac20
                    | _ -> ()

                if sep > R then
                    AxisTestResultF(true, 0.0f, MathF.Vector3f.Zero, true)
                else
                    let pen = R - sep
                    let penetrationSq = (pen * pen) / Lsq
                    AxisTestResultF(false, penetrationSq, axis, true)
            else
                AxisTestResultF(false, Single.MaxValue, MathF.Vector3f.Zero, false)

        let checkCollisionSATWithCachedAxis
            (p1: Vector3) (d1: Vector3) (o1: Matrix3x3)
            (p2: Vector3) (d2: Vector3) (o2: Matrix3x3)
            (cachedAxisIndex: int) =

            let mutable delta_d = p1 - p2
            WorldLimits.relative &delta_d

            let h1_f = MathF.toFloatV3 (d1 / 2.0)
            let h2_f = MathF.toFloatV3 (d2 / 2.0)
            let delta_f = MathF.toFloatV3 delta_d
            let o1_f = MathF.toFloatM3 o1
            let o2_f = MathF.toFloatM3 o2

            let c00 = MathF.Vector3f.Dot(o1_f.R0, o2_f.R0)
            let c01 = MathF.Vector3f.Dot(o1_f.R0, o2_f.R1)
            let c02 = MathF.Vector3f.Dot(o1_f.R0, o2_f.R2)
            let c10 = MathF.Vector3f.Dot(o1_f.R1, o2_f.R0)
            let c11 = MathF.Vector3f.Dot(o1_f.R1, o2_f.R1)
            let c12 = MathF.Vector3f.Dot(o1_f.R1, o2_f.R2)
            let c20 = MathF.Vector3f.Dot(o1_f.R2, o2_f.R0)
            let c21 = MathF.Vector3f.Dot(o1_f.R2, o2_f.R1)
            let c22 = MathF.Vector3f.Dot(o1_f.R2, o2_f.R2)
            
            let ac00 = MathF.Abs c00 + EPSILON_F
            let ac01 = MathF.Abs c01 + EPSILON_F
            let ac02 = MathF.Abs c02 + EPSILON_F
            let ac10 = MathF.Abs c10 + EPSILON_F
            let ac11 = MathF.Abs c11 + EPSILON_F
            let ac12 = MathF.Abs c12 + EPSILON_F
            let ac20 = MathF.Abs c20 + EPSILON_F
            let ac21 = MathF.Abs c21 + EPSILON_F
            let ac22 = MathF.Abs c22 + EPSILON_F

            let t_x = MathF.Vector3f.Dot(delta_f, o1_f.R0)
            let t_y = MathF.Vector3f.Dot(delta_f, o1_f.R1)
            let t_z = MathF.Vector3f.Dot(delta_f, o1_f.R2)

            let inline runSATLoop startIndex skipIndex minPenSq startAxis startAxisIndex =
                let mutable minPenetrationSq = minPenSq
                let mutable winningAxis = startAxis
                let mutable winningAxisIndex = startAxisIndex
                let mutable areColliding = true
                let mutable index = startIndex

                while areColliding && index < 15 do
                    if index <> skipIndex then
                        let result =
                            testSingleAxis
                                index
                                h1_f h2_f
                                delta_f
                                o1_f o2_f
                                ac00 ac01 ac02 ac10 ac11 ac12 ac20 ac21 ac22
                                t_x t_y t_z

                        if result.IsValidAxis then
                            if result.IsSeparating then
                                areColliding <- false
                                winningAxisIndex <- index
                            elif result.PenetrationSq < minPenetrationSq then
                                minPenetrationSq <- result.PenetrationSq
                                winningAxis <- result.Axis
                                winningAxisIndex <- index
                    index <- index + 1

                if not <| areColliding then
                    struct(CollisionResult.NoCollision, winningAxisIndex)
                else
                    let finalDepth_d = float (MathF.Sqrt minPenetrationSq)
                    let mag_f = winningAxis.Length()
                    let mutable normal_f = winningAxis / mag_f
                    if MathF.Vector3f.Dot(normal_f, delta_f) < 0.0f then
                        normal_f <- -normal_f
                    let normal_d = MathF.toDoubleV3 normal_f
                    let result_d = CollisionResult.Create(normal_d, finalDepth_d)
                    struct(result_d, winningAxisIndex)

            if cachedAxisIndex <> -1 then
                let fastPathResult =
                    testSingleAxis
                        cachedAxisIndex
                        h1_f h2_f
                        delta_f
                        o1_f o2_f
                        ac00 ac01 ac02 ac10 ac11 ac12 ac20 ac21 ac22
                        t_x t_y t_z

                if fastPathResult.IsSeparating || not <| fastPathResult.IsValidAxis then
                    struct(CollisionResult.NoCollision, cachedAxisIndex)
                else
                    runSATLoop 0 cachedAxisIndex fastPathResult.PenetrationSq fastPathResult.Axis cachedAxisIndex
            else
                runSATLoop 0 -1 Single.MaxValue MathF.Vector3f.Zero -1

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
                let struct(minCorner, maxCorner) =
                    Collision.getAABB
                        position
                        dimensions
                        orientation
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
                    MinAABB = minCorner
                    MaxAABB = maxCorner
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
    
        let private _precomputedPrismBaseOffsets =
            [|
                for sector in 0..5 ->
                    let v2_rel = _vertexes[sector]
                    let v3_rel = _vertexes[(sector + 1) % 6]
                    (v2_rel + v3_rel) / 3.0
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
 
            let baseCenter = hexCenter + _precomputedPrismBaseOffsets[sector]
            
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
            (filter: HashSet<uint64>) =
            let mutable rawMinQ, rawMaxQ = Int32.MaxValue, Int32.MinValue
            let mutable rawMinR, rawMaxR = Int32.MaxValue, Int32.MinValue
            let mutable rawMinZ, rawMaxZ = Int32.MaxValue, Int32.MinValue
            
            for i = 0 to corners.Length - 1 do
                let struct(rawQ, rawR, rawZ) = corners[i] |> convertWorldToRawGridCoords
                if rawQ < rawMinQ then rawMinQ <- rawQ
                if rawQ > rawMaxQ then rawMaxQ <- rawQ
                if rawR < rawMinR then rawMinR <- rawR
                if rawR > rawMaxR then rawMaxR <- rawR
                if rawZ < rawMinZ then rawMinZ <- rawZ
                if rawZ > rawMaxZ then rawMaxZ <- rawZ

            let inline checkAndAddSubPrisms q r z startIndex endIndex =
                for sub_idx = startIndex to endIndex do
                    let subPrismCoords = SubPrismCoords.Normalize(q, r, z, sub_idx)
                    let struct(minPrism, maxPrism) = getTriangularPrismAABB subPrismCoords
                    if Collision.checkCollisionAABB minAABB maxAABB minPrism maxPrism then
                        let key = SubPrismKey.pack subPrismCoords
                        filter.Add key |> ignore

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
                                let halfZ = hexBottomZ + 0.5 * HEX_HEIGHT
                                let overlapsBottomHalf = maxAABB.Z >= hexBottomZ && minAABB.Z < halfZ
                                let overlapsTopHalf = maxAABB.Z >= halfZ && minAABB.Z < hexTopZ
                                
                                if overlapsBottomHalf then
                                    checkAndAddSubPrisms q r z 0 5
                                if overlapsTopHalf then
                                    checkAndAddSubPrisms q r z 6 11

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

            fillGridFromCorners shiftBuffer shiftedMin shiftedMax filter
            
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
            
            fillGridFromCorners worldCorners minAABB maxAABB filter

            let margin = (max d.X d.Y) * 1.5
            let nearMinX = minAABB.X < margin
            let nearMaxX = maxAABB.X > (WorldLimits.X - margin)
            let nearMinY = minAABB.Y < margin
            let nearMaxY = maxAABB.Y > (WorldLimits.Y - margin)

            if not <| (nearMinX || nearMaxX || nearMinY || nearMaxY) then
                buffer.AddRange filter
            else
                let shiftBuffer : Span<Vector3> = Stack.alloc 8
                if nearMinX then
                    fillForShift (Vector3(WorldLimits.X, 0.0, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMaxX then
                    fillForShift (Vector3(-WorldLimits.X, 0.0, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMinY then
                    fillForShift (Vector3(0.0, WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMaxY then
                    fillForShift (Vector3(0.0, -WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter

                if nearMinX && nearMinY then
                    fillForShift (Vector3(WorldLimits.X, WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMaxX && nearMinY then
                    fillForShift (Vector3(-WorldLimits.X, WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMinX && nearMaxY then
                    fillForShift (Vector3(WorldLimits.X, -WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter
                if nearMaxX && nearMaxY then
                    fillForShift (Vector3(-WorldLimits.X, -WorldLimits.Y, 0.0)) p d o worldCorners shiftBuffer filter

                buffer.AddRange filter
 
    [<RequireQualifiedAccess>]
    module Geometry =
        type Repo =
            private
                {
                    _logger : ILogger
                    _prisms : Dictionary<uint64, Material>
                    _gravityCheckCandidates: PooledList<uint64>
                    mutable _idGenerator: int
                }
            
            member this.Prisms : IReadOnlyDictionary<_, _> = this._prisms
            member this.Dispose() = this._gravityCheckCandidates |> Dispose.action
            interface IDisposable with
                member this.Dispose() = this.Dispose()
        let createRepo() =
            {
                _logger = Log.ForContext<Repo>()
                _prisms = Dictionary()
                _gravityCheckCandidates = new PooledList<_>()
                _idGenerator = -1
            }
            
        let removePrism key r =
            if r._prisms.Remove key then
                let coords = key |> SubPrismKey.unpack
                let aboveCoords = SubPrismCoords.Normalize(coords.Q, coords.R, coords.Z + 1, coords.SubIndex)
                let aboveKey = aboveCoords |> SubPrismKey.pack

                if r._prisms.ContainsKey aboveKey then
                    if not <| r._gravityCheckCandidates.Contains aboveKey then
                        r._gravityCheckCandidates.Add aboveKey
                true
            else
                false
        
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
            for i = r._gravityCheckCandidates.Count - 1 downto 0 do
                let coordsKey = r._gravityCheckCandidates[i]

                if not <| r._prisms.ContainsKey coordsKey then
                    r._gravityCheckCandidates.RemoveAt i
                elif r._prisms[coordsKey] = Material.GravityEnabled then
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

                r._gravityCheckCandidates.RemoveAt i

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
                        dynamicBody.Position
                        dynamicBody.Dimensions
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
            this.CollisionDestroyedFlora.Clear()
            this.CollisionProcessedStatic.Clear()
 
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
        
        let inline private areTransformsCloseEnough (b1: inref<Body.T>) (b2: inref<BodyCacheEntry>) =
            let posDeltaSq = (b1.Position - b2.Position).MagnitudeSq()
            if posDeltaSq > POSITION_THRESHOLD_SQ then
                false
            else
                let orientDelta = Vector3.Dot(b1.Orientation.R0, b2.Orientation.R0)
                orientDelta > ORIENTATION_THRESHOLD
        
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

        let add(body: byref<Body.T>) buffers r =
            if r._bodyStateCache.ContainsKey body.Id then
                r._logger.Warning("Body {BodyId} was already added into cache", body.Id)
            else
                Body.updateAABB &body
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

        let update (body: byref<Body.T>) buffers r =
            let cachedEntry = &CollectionsMarshal.GetValueRefOrNullRef(r._bodyStateCache, body.Id)
            match Unsafe.IsNullRef &cachedEntry with
            | true -> add &body buffers r
            | false ->
                if not <| areTransformsCloseEnough &body &cachedEntry then
                    Body.updateAABB &body
                    let oldOccupiedCells = cachedEntry.OccupiedCells

                    let newOccupiedCells = new PooledList<uint64>()
                    Grid.fillOverlappingSubPrismsAABB
                        body.Position
                        body.Dimensions
                        body.Orientation
                        buffers.UniquePrismsFilterBuffer
                        newOccupiedCells

                    use newCellsSet = new PooledSet<uint64>(newOccupiedCells.Span)

                    for oldCell in oldOccupiedCells.Span do
                        if not <| newCellsSet.Remove oldCell then
                            r |> removeBodyFromCell body.Id oldCell

                    for newCell in newCellsSet do
                        r |> addBodyToCell body.Id newCell

                    oldOccupiedCells |> Dispose.action
                    cachedEntry <- BodyCacheEntry(&body, newOccupiedCells)
    
    [<RequireQualifiedAccess>]
    module DSU =
        [<Struct; IsByRefLike>]
        type T =
            {
                _mapIdToIndex: PooledDictionary<int, int>
                _mapIndexToId: Span<int>
                _parent: Span<int>
                _size: Span<int>
                mutable _componentCount: int
            }

            member this.Dispose() =
                this._mapIdToIndex.Dispose()

            interface IDisposable with
                member this.Dispose() = this.Dispose()

            member private this.FindRootIndex itemIndex =
                let mutable root = itemIndex
                while root <> this._parent[root] do
                    root <- this._parent[root]

                let mutable current = itemIndex
                while current <> root do
                    let next = this._parent[current]
                    this._parent[current] <- root
                    current <- next
                root

            member this.Find itemId =
                match this._mapIdToIndex.TryGetValue itemId with
                | true, index -> this._mapIndexToId[this.FindRootIndex index]
                | false, _ ->
                    // Should not happen if DSU is used correctly
                    itemId

            member this.Union(id1, id2) =
                match this._mapIdToIndex.TryGetValue id1 with
                | false, _ -> ()
                | true, index1 ->
                    match this._mapIdToIndex.TryGetValue id2 with
                    | false, _ -> ()
                    | true, index2 ->
                        let root1 = this.FindRootIndex index1
                        let root2 = this.FindRootIndex index2

                        if root1 <> root2 then
                            if this._size[root1] < this._size[root2] then
                                this._parent[root1] <- root2
                                this._size[root2] <- this._size[root2] + this._size[root1]
                            else
                                this._parent[root2] <- root1
                                this._size[root1] <- this._size[root1] + this._size[root2]

                            this._componentCount <- this._componentCount - 1

            member this.ComponentCount = this._componentCount

            member this.GetAllComponents() =
                let mapIndexToId = this._mapIndexToId
                let result = new PooledList<PooledList<int>>()
                use rootMap = new PooledDictionary<int, PooledList<int>>()

                for i = 0 to mapIndexToId.Length - 1 do
                    let id = mapIndexToId[i]
                    let rootIndex = this.FindRootIndex i
                    let rootId = mapIndexToId[rootIndex]

                    match rootMap.TryGetValue rootId with
                    | true, list -> list.Add id
                    | false, _ ->
                        let newList = new PooledList<int>()
                        newList.Add id
                        rootMap.Add(rootId, newList)
                        result.Add newList
                result
                
        // Don't remove inline, otherwise the stack working will break
        let inline create(ids: ReadOnlySpan<int>) =
            let mapIdToIndex = new PooledDictionary<int, int>(ids.Length)
            let mapIndexToId = Stack.alloc ids.Length
            let parent = Stack.alloc ids.Length
            let size = Stack.alloc ids.Length

            for i = 0 to ids.Length - 1 do
                let id = ids[i]
                mapIdToIndex.Add(id, i)
                mapIndexToId[i] <- id
                parent[i] <- i
                size[i] <- 1

            {
                _mapIdToIndex = mapIdToIndex
                _mapIndexToId = mapIndexToId
                _parent = parent
                _size = size
                _componentCount = ids.Length
            }
                
    [<RequireQualifiedAccess>]
    module Island =
        
        let [<Literal>] private GROUND_NODE_ID = Int32.MinValue
        
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
                    Bodies = new PooledList<_>()
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
                    IsGroundedCacheValid = false
                    OccupiedGridCells = new PooledList<uint64>()
                }
            val Id: int
            val Bodies: PooledList<int>
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
            val mutable IsGroundedCacheValid : bool
            val OccupiedGridCells: PooledList<uint64>
            
            member inline this.BodiesSpan : ReadOnlySpan<int> = this.Bodies.Span
            member inline this.AddBody bodyId =
                if not <| this.BodiesSpan.Contains bodyId then
                    this.Bodies.Add bodyId
            
            member inline this.RemoveBody bodyId =
                Utils.removeBySwapBack bodyId this.Bodies |> ignore

            member inline this.UpdateMaxPenetration depth =
                if depth > this.MaxPenetrationThisStep then
                    this.MaxPenetrationThisStep <- depth
                    
            member inline this.EnsureContactsExist() =
                if isNull this._contactsHolder then
                    this._contactsHolder <- { Contacts = ExpiringContacts.Create() }
                    
            member inline this.AddOrUpdateContact(key, ttl, newCachedAxis) =
                this.EnsureContactsExist()
                this._contactsHolder.Contacts.AddOrUpdate(key, ttl, newCachedAxis)
                this.IsGroundedCacheValid <- false
                
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
                    this.IsGroundedCacheValid <- false
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
                    this.OccupiedGridCells |> Dispose.action
                    contactsHolder.Contacts |> Dispose.action
                    
            interface IDisposable with
                member this.Dispose() = this.Dispose()

        [<RequireQualifiedAccess>]
        module private IslandGridPhase =
            let inline packKey (gridX: int) (gridY: int) =
                (uint64 gridX <<< 32) ||| (uint64 gridY &&& 0xFFFFFFFFUL)

            let inline worldToGridCoord (pos: double) (cellSize: double) =
                Math.Floor(pos / cellSize) |> int
               
        type Repo =
            private
                {
                    _spatialHash : SpatialHash.Repo
                    _bodyRepo: Body.Repo
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
                    _islandGrid: Dictionary<uint64, PooledList<int>>
                    _gridCellSize: double
                    _newOccupiedCellsBuffer: PooledList<uint64>
                    _newCellsFilter : PooledSet<uint64>

                    mutable _currentId: int
                }
            member this.ActiveIslandIds = this._activeIslandIds
            member this.SleepingIslandIds = this._sleepingIslandIds

            member this.Dispose() =
                this._allIslands.Values |> Seq.iter Dispose.action
                this._allIslands.Clear()
                this._islandGrid.Values |> Seq.iter Dispose.action
                this._islandGrid.Clear()
                this._newOccupiedCellsBuffer |> Dispose.action
                this._newCellsFilter |> Dispose.action
                
            interface IDisposable with
                member this.Dispose() = this.Dispose()
        let getActiveIslandIds r = r._activeIslandIds

        let createRepo spatialHash bodyRepo =
            {
                _spatialHash = spatialHash
                _bodyRepo = bodyRepo
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
                _islandGrid = Dictionary<_,_>()
                _newOccupiedCellsBuffer = new PooledList<_>()
                _newCellsFilter = new PooledSet<_>()
                _gridCellSize = MAX_DIMENSION * 2.0 
                _currentId = -1
            }

        let recalculateAABB (island: byref<T>) r =
            let bodyCount = island.BodiesSpan.Length
            if bodyCount = 0 then
                island.MinAABB <- Vector3(Double.MaxValue, Double.MaxValue, Double.MaxValue)
                island.MaxAABB <- Vector3(Double.MinValue, Double.MinValue, Double.MinValue)
            else
                let firstBody = &Body.getRef island.BodiesSpan[0] r._bodyRepo
                let mutable referencePos = firstBody.Position
                let mutable averageRelativePos = Vector3.Zero

                for i = 1 to bodyCount - 1 do
                    let body = &Body.getRef island.BodiesSpan[i] r._bodyRepo
                    let mutable delta = body.Position - referencePos
                    WorldLimits.relative &delta
                    averageRelativePos <- averageRelativePos + delta

                let mutable centroid = referencePos + averageRelativePos / (double bodyCount)
                WorldLimits.wrapPosition &centroid

                let mutable minExtents = Vector3(Double.MaxValue, Double.MaxValue, Double.MaxValue)
                let mutable maxExtents = Vector3(Double.MinValue, Double.MinValue, Double.MinValue)
                
                for bodyId in island.BodiesSpan do
                    let body = &Body.getRef bodyId r._bodyRepo
                    let mutable deltaMin = body.MinAABB - centroid
                    WorldLimits.relative &deltaMin

                    let mutable deltaMax = body.MaxAABB - centroid
                    WorldLimits.relative &deltaMax

                    minExtents.X <- min minExtents.X deltaMin.X
                    minExtents.Y <- min minExtents.Y deltaMin.Y
                    minExtents.Z <- min minExtents.Z deltaMin.Z

                    maxExtents.X <- max maxExtents.X deltaMax.X
                    maxExtents.Y <- max maxExtents.Y deltaMax.Y
                    maxExtents.Z <- max maxExtents.Z deltaMax.Z

                Debug.Assert(((maxExtents.X - minExtents.X) < (WorldLimits.X * 0.9)), "Island AABB exceeds world size limit on X-axis")
                Debug.Assert(((maxExtents.Y - minExtents.Y) < (WorldLimits.Y * 0.9)), "Island AABB exceeds world size limit on Y-axis")
                
                island.MinAABB <- centroid + minExtents
                island.MaxAABB <- centroid + maxExtents
                
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
                newIsland.AddBody bodyId
                r._bodyToIslandMap.Add(bodyId, newIsland.Id)
                r._allIslands.Add(newIsland.Id, newIsland)
                r._activeIslandIds.Add newIsland.Id |> ignore

        let private buildAdjacencyMap
            (bodies: ReadOnlySpan<int>)
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
            (bodiesToAnalyze: ReadOnlySpan<int>)
            (island: inref<T>) =
            use dsu = DSU.create bodiesToAnalyze

            for contactKey in island.GetContactKeys() do
                let struct(id1, id2) = ContactKey.unpack contactKey
                dsu.Union(id1, id2)

            if dsu.ComponentCount <= 1 then
                ValueNone
            else
                dsu.GetAllComponents() |> ValueSome
        
        let private hasStaticSupport
            (body: inref<Body.T>)
            spatialHash
            geometryRepo
            r =
            
            let cache = r._staticSupportCache
            match cache.TryGetValue body.Id with
            | true, result -> result
            | false, _ ->
                
                let mutable hasFoundSupport = false
                let radiusZ = Collision.getProjectedRadiusZ body.Dimensions body.Orientation
                let lowestPointZ = body.Position.Z - radiusZ
                
                if lowestPointZ < PENETRATION_SLOP then
                    hasFoundSupport <- true
      
                if not <| hasFoundSupport then
                    let occupiedCells = SpatialHash.getOccupiedCells body.Id spatialHash
                    
                    let mutable i = 0
                    while not <| hasFoundSupport && i < occupiedCells.Length do
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
            geometryRepo
            spatialHash
            (island: byref<T>)
            r =
            if island.IsGroundedCacheValid then
                island.IsGrounded
            else
                let bodiesSpan = island.BodiesSpan
                if bodiesSpan.Length = 0 then
                    island.IsGrounded <- true
                    island.IsGroundedCacheValid <- true
                elif bodiesSpan.Length = 1 then
                    let bodyId = bodiesSpan[0]
                    let body = &Body.getRef bodyId bodyRepo

                    island.IsGrounded <- hasStaticSupport &body spatialHash geometryRepo r
                    island.IsGroundedCacheValid <- true 
                else
                    use bodyIdsWithGround = new PooledList<int>(bodiesSpan)
                    bodyIdsWithGround.Add GROUND_NODE_ID
                    use dsu = DSU.create bodyIdsWithGround.Span

                    for bodyId in bodiesSpan do
                        let body = &Body.getRef bodyId bodyRepo
                        if hasStaticSupport &body spatialHash geometryRepo r then
                            dsu.Union(bodyId, GROUND_NODE_ID)

                    for contactKey in island.GetContactKeys() do
                        let struct(id1, id2) = ContactKey.unpack contactKey
                        dsu.Union(id1, id2)

                    let groundRoot = dsu.Find GROUND_NODE_ID
                    let mutable allGrounded = true
                    let mutable i = 0
                    while allGrounded && i < bodiesSpan.Length do
                        let bodyId = bodiesSpan[i]
                        if dsu.Find bodyId <> groundRoot then
                            allGrounded <- false
                        i <- i + 1

                    island.IsGrounded <- allGrounded
                    island.IsGroundedCacheValid <- true
                        
                island.IsGrounded
    
        let getIslandRefForBody bodyId r = &CollectionsMarshal.GetValueRefOrNullRef(r._allIslands, r._bodyToIslandMap[bodyId])
        let getIslandIdForBody bodyId r = r._bodyToIslandMap[bodyId]
        let getIslandRef islandId r = &CollectionsMarshal.GetValueRefOrNullRef(r._allIslands, islandId)
        
        let private calculateOccupiedCells (minCorner: Vector3) (maxCorner: Vector3) (cellSize: double) (buffer: PooledList<uint64>) =
            buffer.Clear()
            let gridCellsX = IslandGridPhase.worldToGridCoord WorldLimits.X cellSize
            let gridCellsY = IslandGridPhase.worldToGridCoord WorldLimits.Y cellSize

            let minGridX = IslandGridPhase.worldToGridCoord minCorner.X cellSize
            let maxGridX = IslandGridPhase.worldToGridCoord maxCorner.X cellSize
            let minGridY = IslandGridPhase.worldToGridCoord minCorner.Y cellSize
            let maxGridY = IslandGridPhase.worldToGridCoord maxCorner.Y cellSize

            let inline addKey (gx: int) (gy: int) =
                let wrappedGx = (gx % gridCellsX + gridCellsX) % gridCellsX
                let wrappedGy = (gy % gridCellsY + gridCellsY) % gridCellsY
                let key = IslandGridPhase.packKey wrappedGx wrappedGy
                buffer.Add key

            let inline processYRange (gx: int) =
                if minGridY <= maxGridY then
                    for gy = minGridY to maxGridY do
                        addKey gx gy
                else
                    for gy = minGridY to gridCellsY - 1 do
                        addKey gx gy
                    for gy = 0 to maxGridY do
                        addKey gx gy

            if minGridX <= maxGridX then
                for gx = minGridX to maxGridX do
                    processYRange gx
            else
                for gx = minGridX to gridCellsX - 1 do
                    processYRange gx
                for gx = 0 to maxGridX do
                    processYRange gx
        
        let queryIslandsByAABB 
            (minAABB: Vector3) 
            (maxAABB: Vector3) 
            (r: Repo)
            (resultSet: PooledSet<int>)
            (tempBuffer: PooledList<uint64>) =

            resultSet.Clear()
            tempBuffer.Clear()
    
            calculateOccupiedCells minAABB maxAABB r._gridCellSize tempBuffer

            for cellKey in tempBuffer.Span do
                match r._islandGrid.TryGetValue cellKey with
                | true, idList ->
                    for islandId in idList.Span do
                        resultSet.Add islandId |> ignore
                | false, _ -> ()
                            
        let private removeIslandFromGrid (island: inref<T>) r =
            let islandId = island.Id
            for cellKey in island.OccupiedGridCells.Span do
                match r._islandGrid.TryGetValue cellKey with
                | true, idList ->
                    if idList |> Utils.removeBySwapBack islandId && idList.Count = 0 then
                        r._islandGrid.Remove cellKey |> ignore
                        idList |> Dispose.action
                | false, _ -> ()
            island.OccupiedGridCells.Clear()

        let private addIslandToGrid (island: byref<T>) r =
            let islandId = island.Id
            let tempBuffer = island.OccupiedGridCells
            
            calculateOccupiedCells island.MinAABB island.MaxAABB r._gridCellSize tempBuffer
            
            for cellKey in tempBuffer.Span do
                let mutable isListExists = false
                let list = &CollectionsMarshal.GetValueRefOrAddDefault(r._islandGrid, cellKey, &isListExists)
                if not <| isListExists then
                    list <- new PooledList<int>()
                    
                list.Add islandId

        let private updateIslandInGrid (island: byref<T>) r =
            r._newOccupiedCellsBuffer.Clear()
            
            calculateOccupiedCells island.MinAABB island.MaxAABB r._gridCellSize r._newOccupiedCellsBuffer

            r._newCellsFilter.Clear()
            for newCell in r._newOccupiedCellsBuffer.Span do
                r._newCellsFilter.Add newCell |> ignore
            
            let islandId = island.Id
            let oldOccupiedCells = island.OccupiedGridCells

            for oldCellKey in oldOccupiedCells.Span do
                if not <| r._newCellsFilter.Remove oldCellKey then
                    match r._islandGrid.TryGetValue oldCellKey with
                    | true, idList ->
                        if idList |> Utils.removeBySwapBack islandId && idList.Count = 0 then
                            r._islandGrid.Remove oldCellKey |> ignore
                            idList |> Dispose.action
                    | false, _ -> ()

            for newCellKey in r._newCellsFilter do
                let mutable isListExists = false
                let list = &CollectionsMarshal.GetValueRefOrAddDefault(r._islandGrid, newCellKey, &isListExists)
                if not <| isListExists then
                    list <- new PooledList<int>()
                
                list.Add islandId

            oldOccupiedCells.Clear()
            oldOccupiedCells.AddRange(r._newOccupiedCellsBuffer.Span)

        let recalculateAABBAndUpdateGrid (island: byref<T>) r =
            recalculateAABB &island r
            updateIslandInGrid &island r
        
        let detectBroadPhaseIslandPairs buffers r =
            let grid = r._islandGrid
            let collidingIslandPairs = buffers.CollisionIslandPairs
            collidingIslandPairs.Clear()

            for idsInCell in grid.Values do
                let idsSpan = idsInCell.Span
                if idsSpan.Length > 1 then
                    for i = 0 to idsSpan.Length - 1 do
                        for j = i + 1 to idsSpan.Length - 1 do
                            let id1 = idsSpan[i]
                            let id2 = idsSpan[j]

                            let mutable island1 = &getIslandRef id1 r
                            let mutable island2 = &getIslandRef id2 r

                            if island1.IsAwake || island2.IsAwake then
                                let pairKey = ContactKey.key id1 id2

                                if collidingIslandPairs.Add pairKey then
                                    if not <| Collision.checkCollisionAABB island1.MinAABB island1.MaxAABB island2.MinAABB island2.MaxAABB then
                                        collidingIslandPairs.Remove pairKey |> ignore
            
            collidingIslandPairs
            
        let addBody bodyId r=
            let mutable isFound = false
            let islandId = &CollectionsMarshal.GetValueRefOrAddDefault(r._bodyToIslandMap, bodyId, &isFound)
            if not <| isFound then 
                let mutable newIsland = r |> newIsland
                r._allIslands.Add(newIsland.Id, newIsland)
                islandId <- newIsland.Id
                newIsland.AddBody bodyId
                r._activeIslandIds.Add newIsland.Id |> ignore
                recalculateAABB &newIsland r
                addIslandToGrid &newIsland r
                
        let removeBody bodyId r =
            let mutable islandId = 0
            if r._bodyToIslandMap.TryGetValue(bodyId, &islandId) then
                r._bodyToIslandMap.Remove bodyId |> ignore
                
                let mutable island = &getIslandRef islandId r
                island.RemoveBody bodyId
                if island.BodiesSpan.Length = 0 then
                    if not <| r._removeIslandsBuffer.Contains islandId then
                       r._removeIslandsBuffer.Add islandId
                else
                    recalculateAABBAndUpdateGrid &island r
                
                island.IsGroundedCacheValid <- false
                
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
                    removeIslandFromGrid &sourceIsland r
                    removeIslandFromGrid &targetIsland r
                    
                    targetIsland.CantSleepFrames <- max sourceIsland.CantSleepFrames targetIsland.CantSleepFrames
                    targetIsland.FramesResting <- 0
                    targetIsland.IsGroundedCacheValid <- false
                    targetIsland.UpdateMaxPenetration sourceIsland.MaxPenetrationThisStep

                    for bodyId in sourceIsland.BodiesSpan do
                        targetIsland.AddBody bodyId
                        r._bodyToIslandMap[bodyId] <- targetId
                    
                    targetIsland.MergeContactsFrom &sourceIsland
   
                    let mutable finalMin = targetIsland.MinAABB
                    let mutable finalMax = targetIsland.MaxAABB
                    let sourceMin = sourceIsland.MinAABB
                    let sourceMax = sourceIsland.MaxAABB

                    let mutable delta = sourceMin - finalMin
                    WorldLimits.relative &delta
                    let adjustedSourceMin = finalMin + delta

                    delta <- sourceMax - finalMin
                    WorldLimits.relative &delta
                    let adjustedSourceMax = finalMin + delta
                    
                    finalMin.X <- min finalMin.X adjustedSourceMin.X
                    finalMin.Y <- min finalMin.Y adjustedSourceMin.Y
                    finalMin.Z <- min finalMin.Z adjustedSourceMin.Z
                    
                    finalMax.X <- max finalMax.X adjustedSourceMax.X
                    finalMax.Y <- max finalMax.Y adjustedSourceMax.Y
                    finalMax.Z <- max finalMax.Z adjustedSourceMax.Z

                    targetIsland.MinAABB <- finalMin
                    targetIsland.MaxAABB <- finalMax

                    addIslandToGrid &targetIsland r
                    
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
                let island = &getIslandRef removedId r
                if not <| Unsafe.IsNullRef &island then
                    removeIslandFromGrid &island r
                r._allIslands.Remove removedId |> ignore
                r._activeIslandIds.Remove removedId |> ignore
                r._sleepingIslandIds.Remove removedId |> ignore
            r._removeIslandsBuffer.Clear()
        
        let private wakeUpIslands spatialHash (buffers: Buffers) r =
            for islandId in r._islandsToWake do
                let mutable island = &getIslandRef islandId r
                if Unsafe.IsNullRef &island then
                    r._removeIslandsBuffer.Add islandId
                elif island.Bodies.Count = 0 then
                        r._removeIslandsBuffer.Add island.Id
                        island |> Dispose.action
                elif not <| island.IsAwake then
                    island.IsAwake <- true
                    island.FramesResting <- 0
                    island.IsGroundedCacheValid <- false
                    
                    // It is strictly forbidden to erase ContactTTL,
                    // because any awakened island must remember all its contacts, so as not to
                    // start the tedious and expensive procedure of assembling back into the island again
                    // island.ContactTTL.Clear()
                    
                    for bodyId in island.BodiesSpan do
                        let body = &Body.getRef bodyId r._bodyRepo
                        SpatialHash.add &body buffers spatialHash
                        r._staticSupportCache.Remove bodyId |> ignore
                        
                    r._sleepingIslandIds.Remove island.Id |> ignore
                    r._activeIslandIds.Add island.Id |> ignore
            r._islandsToWake.Clear()

        let private mergeIslands r =        
            while r._islandsToMerge.Count > 0 do
                let struct(sourceId, targetId) = r._islandsToMerge.Dequeue()
                r |> islandMerge sourceId targetId
            r._islandsToMergePairs.Clear()

        let private splitIslands r =        
            for islandIdToSplit in r._islandsMarkedForSplit do
                let mutable originalIsland = &getIslandRef islandIdToSplit r
                if not <| Unsafe.IsNullRef &originalIsland then
                    match findConnectedComponents originalIsland.BodiesSpan &originalIsland with
                    | ValueSome components ->
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
                                    newFragmentIsland.AddBody bodyId
                                    originalIsland.RemoveBody bodyId
                                    r._bodyToIslandMap[bodyId] <- newFragmentIsland.Id

                                let fragmentBodiesSpan = newFragmentIsland.BodiesSpan
                                for contactKey in originalContactKeys do
                                    let struct(id1, id2) = ContactKey.unpack contactKey
                                    if fragmentBodiesSpan.Contains id1 && fragmentBodiesSpan.Contains id2 then
                                       let mutable cachedAxis = -1
                                       originalIsland.TryGetCachedAxis(contactKey, &cachedAxis) |> ignore
                                       newFragmentIsland.AddOrUpdateContact(contactKey, CONTACT_TTL, cachedAxis)
                                
                                newFragmentIsland.IsGroundedCacheValid <- false
       
                                recalculateAABBAndUpdateGrid &newFragmentIsland r
                                
                        originalIsland.IsGroundedCacheValid <- false
 
                        recalculateAABBAndUpdateGrid &originalIsland r

                        use contactsToRemove = new PooledList<int64>()
                        let originalIslandBodiesSpan = originalIsland.BodiesSpan
                        for contactKey in originalContactKeys do
                            let struct(id1, id2) = ContactKey.unpack contactKey
                            if not <| originalIslandBodiesSpan.Contains id1 || not <| originalIslandBodiesSpan.Contains id2 then
                                contactsToRemove.Add contactKey
                        
                        for key in contactsToRemove.Span do
                            originalIsland.RemoveContact key |> ignore

                        components |> Seq.iter Dispose.action
                    | ValueNone -> ()
            r._islandsMarkedForSplit.Clear()

        let private putIslandsToSleep spatialHash r =         
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
                        let mutable i = 0
                        while isStillEligibleForSleep && i < island.BodiesSpan.Length do
                            let bodyId = island.BodiesSpan[i]
                            i <- i + 1
                            let body = &Body.getRef bodyId r._bodyRepo
                            if body.Velocity.MagnitudeSq() >= SLEEP_VELOCITY_THRESHOLD_SQ then
                                isStillEligibleForSleep <- false
                        
                        if isStillEligibleForSleep then
                            island.IsAwake <- false
                            island.MaxPenetrationThisStep <- 0.0
                            for bodyId in island.BodiesSpan do
                                let body = &Body.getRef bodyId r._bodyRepo
                                spatialHash |> SpatialHash.remove bodyId
                                body.Velocity <- Vector3.Zero
                            
                            recalculateAABBAndUpdateGrid &island r
                            
                            r._activeIslandIds.Remove island.Id |> ignore
                            r._sleepingIslandIds.Add island.Id |> ignore
                        else
                            island.FramesResting <- 0
          
            r._islandsToSleep.Clear()

        let filterEmptyIslands r =
            for island in r._allIslands.Values do
                if island.Bodies.Count = 0 then
                    r._removeIslandsBuffer.Add island.Id
                    
        let processIslandChanges spatialHash buffers r =

            r._mergeRedirects.Clear()
            r._islandsInvolvedInMerge.Clear()
            
            r |> wakeUpIslands spatialHash buffers
            
            r |> mergeIslands

            r |> removeIslands
    
            r |> splitIslands
            
            r |> putIslandsToSleep spatialHash
            
            r |> filterEmptyIslands
                    
            r |> removeIslands
    
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

        let private isTargetPositionFree
            bodyToSnapId
            candidatePosition
            bodyDimensions
            bodyOrientation
            bodyRepo
            islandRepo
            geometryRepo
            (buffers: Buffers) =
                
            let mutable isFree = true

            let struct(candidateMinAABB, candidateMaxAABB) =
                Collision.getAABB candidatePosition bodyDimensions bodyOrientation

            use islandCandidates = new PooledSet<int>()

            Island.queryIslandsByAABB 
                candidateMinAABB 
                candidateMaxAABB 
                islandRepo 
                islandCandidates
                buffers.UniquePrismsBuffer

            let mutable enumerator = islandCandidates.GetEnumerator()
            while isFree && enumerator.MoveNext() do
                let islandId = enumerator.Current
                let otherIsland = &Island.getIslandRef islandId islandRepo
                if Collision.checkCollisionAABB candidateMinAABB candidateMaxAABB otherIsland.MinAABB otherIsland.MaxAABB then
                    let mutable i = 0
                    while isFree && i < otherIsland.BodiesSpan.Length do
                        let otherId = otherIsland.BodiesSpan[i]
                        i <- i + 1
                        if otherId <> bodyToSnapId then
                            let otherBody = &Body.getRef otherId bodyRepo
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

            if isFree then
                Grid.fillOverlappingSubPrismsAABB
                    candidatePosition
                    bodyDimensions
                    bodyOrientation
                    buffers.UniquePrismsFilterBuffer
                    buffers.UniquePrismsBuffer

                let occupiedCells = buffers.UniquePrismsBuffer.Span
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

                        if result.AreColliding && result.Depth > PENETRATION_SLOP then
                            isFree <- false
                    i <- i + 1
                                    
            isFree

        [<Struct; IsReadOnly>]
        type private VectorDistanceComparer(referencePoint: Vector3) =
            interface IComparer<Vector3> with
                member _.Compare(p1, p2) =
                    let distSq1 = (p1 - referencePoint).MagnitudeSq()
                    let distSq2 = (p2 - referencePoint).MagnitudeSq()
                    distSq1.CompareTo distSq2
            
        let trySnapToGridCollisionAware
            (body: byref<Body.T>)
            bodyRepo
            islandRepo
            geometryRepo
            buffers =
            
            let offsetFromCenterToBottom = body.Orientation * Vector3(0.0, 0.0, -body.Dimensions.Z / 2.0)
            let bodyPos = body.Position
            let bodyHeight = body.Dimensions.Z

            let halfBodyHeightInCells = int (Math.Ceiling((bodyHeight / 2.0) / HEX_HEIGHT))
            let minZOffset = -halfBodyHeightInCells - 1
            
            let struct(rawQ, rawR, rawZ) = bodyPos |> Grid.convertWorldToRawGridCoords

            use supportedCandidates = new PooledList<Vector3>()
    
            for zOffset = minZOffset to 0 do
                let searchCoords = SubPrismCoords.Normalize(rawQ, rawR, rawZ + zOffset, 0)
                getSnapPointsForHex searchCoords buffers.SnapPointsBuffer

                for point in buffers.SnapPointsBuffer.Span do
                    if geometryRepo |> Geometry.hasSupportBeneath point then
                        supportedCandidates.Add point

            if supportedCandidates.Count = 0 then
                false
            else
                supportedCandidates.Span.Sort(VectorDistanceComparer(bodyPos))

                let mutable isFoundSnapPoint = false
                let mutable i = 0
                let candidatesSpan = supportedCandidates.Span
                while not <| isFoundSnapPoint && i < candidatesSpan.Length do
                    let point = candidatesSpan[i]
                    i <- i + 1
                    let candidateBodyCenter = point - offsetFromCenterToBottom

                    if isTargetPositionFree
                        body.Id
                        candidateBodyCenter
                        body.Dimensions
                        body.Orientation
                        bodyRepo
                        islandRepo
                        geometryRepo
                        buffers
                    then
                        body.Position <- candidateBodyCenter
                        body.Velocity <- Vector3.Zero
                        body.IsSnappedToGrid <- true
                        isFoundSnapPoint <- true

                isFoundSnapPoint

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
                _islandRepo : Island.Repo
                _staticCache: Dictionary<uint64, struct(Vector3 * Vector3 * Matrix3x3)>
                _liquidRepo : Liquid.Repo
                _floraRepo : Flora.Repo
                _buffers : Buffers
                _spatialHash : SpatialHash.Repo
                _random : Random
                _dt : double
            }
        
        member this.Geometry = this._geometryRepo
        member this.Bodies = this._bodyRepo
        member this.Islands = this._islandRepo
        member this.Liquid = this._liquidRepo
        member this.Flora = this._floraRepo
        member this.Buffers = this._buffers
        
        member this.Dispose() =
            this._islandRepo |> Dispose.action
            this._liquidRepo |> Dispose.action
            this._floraRepo |> Dispose.action
            this._buffers |> Dispose.action
            this._spatialHash |> Dispose.action
            this._geometryRepo |> Dispose.action
            
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
        let spatialHash = SpatialHash.createRepo()
        let buffers = Buffers.Create()
        {
            _bodyRepo = bodyRepo
            _islandRepo = Island.createRepo spatialHash bodyRepo
            _floraRepo = Flora.createRepo geometryRepo
            _geometryRepo = geometryRepo
            _liquidRepo = Liquid.createRepo geometryRepo
            _buffers = buffers
            _spatialHash = spatialHash
            _staticCache = Dictionary<_, _>()
            _random = Random(Guid.NewGuid().GetHashCode())
            _dt = dt
        }
        
    let nextBodyId engine = engine._bodyRepo |> Body.nextId
    
    [<RequireQualifiedAccess>]
    module Simulation = 
        let isPointSupported
            geometryRepo
            bodyRepo
            (body: inref<Body.T>)
            (island: inref<Island.T>)
            (checkPoint: Vector3)
            (buffers: Buffers) =
            if checkPoint.Z <= PENETRATION_SLOP then
                true
            else
                let checkDepth = PENETRATION_SLOP * 2.0
                let staticProbeDims = Vector3(STATIC_SUPPORT_PROBE_RADIUS * 2.0, STATIC_SUPPORT_PROBE_RADIUS * 2.0, checkDepth)
                let probeCenter = Vector3(checkPoint.X, checkPoint.Y, checkPoint.Z - checkDepth / 2.0)
                let probeOrientation = Matrix3x3.Identity

                Grid.fillOverlappingSubPrismsAABB
                    probeCenter
                    staticProbeDims
                    probeOrientation
                    buffers.UniquePrismsFilterBuffer
                    buffers.UniquePrismsBuffer

                let occupiedCells = buffers.UniquePrismsBuffer.Span
                let mutable isSupported = false
                let mutable i = 0
                while not <| isSupported && i < occupiedCells.Length do
                    let cellKey = occupiedCells[i]
                    i <- i + 1
                    if geometryRepo |> Geometry.isSolid cellKey then
                        isSupported <- true

                if not <| isSupported then
                    let dynamicProbeRadius = DYNAMIC_SUPPORT_PROBE_RADIUS
                    let dynamicProbeDims = Vector3(dynamicProbeRadius * 2.0, dynamicProbeRadius * 2.0, checkDepth)
                    
                    let probeMinCorner = probeCenter - dynamicProbeDims / 2.0
                    let probeMaxCorner = probeCenter + dynamicProbeDims / 2.0

                    let mutable i = 0
                    while not <| isSupported && i < island.BodiesSpan.Length do
                        let otherId = island.BodiesSpan[i]
                        i <- i + 1
                        if otherId <> body.Id then
                            let otherBody = &Body.getRef otherId bodyRepo
                            if Collision.checkCollisionAABB probeMinCorner probeMaxCorner otherBody.MinAABB otherBody.MaxAABB then
                                let result =
                                    Collision.checkCollisionSAT
                                        probeCenter
                                        dynamicProbeDims
                                        probeOrientation
                                        otherBody.Position
                                        otherBody.Dimensions
                                        otherBody.Orientation

                                if result.AreColliding then
                                    isSupported <- true
                                
                isSupported
        
        let private tryInitiateFall
            (body: inref<Body.T>)
             geometryRepo
             bodyRepo
             islandRepo
             buffers =
            if body.IsFallingOver || body.InvMass < EPSILON then
                ValueNone
            else
                let h = body.Dimensions / 2.0
                let bottomCenterOffset = body.Orientation * Vector3(0.0, 0.0, -h.Z)
                let mutable projectedCenterOfMass = body.Position + bottomCenterOffset
                WorldLimits.wrapPosition &projectedCenterOfMass
                let islandId = islandRepo |> Island.getIslandIdForBody body.Id
                let island = &Island.getIslandRef islandId islandRepo
                let isCenterOfMassSupported = isPointSupported geometryRepo bodyRepo &body &island projectedCenterOfMass buffers
                if isCenterOfMassSupported then
                    ValueNone
                else
                    let localCornersSpan = Stack.alloc 4
                    localCornersSpan[0] <- Vector3(-h.X, -h.Y, -h.Z)
                    localCornersSpan[1] <- Vector3(h.X, -h.Y, -h.Z)
                    localCornersSpan[2] <- Vector3(h.X, h.Y, -h.Z)
                    localCornersSpan[3] <- Vector3(-h.X, h.Y, -h.Z) 

                    let mutable supportedCorners = 0
                    let supportedCornersSpan = Stack.alloc 4
                    let supportedLocalCornersSpan = Stack.alloc 4
                                        
                    let mutable unsupportedCorners = 0
                    let unsupportedCornersSpan = Stack.alloc 4

                    for localCorner in localCornersSpan do
                        let worldCorner = body.Position + body.Orientation * localCorner
                        if isPointSupported geometryRepo bodyRepo &body &island worldCorner buffers then
                            supportedCornersSpan[supportedCorners] <- worldCorner
                            supportedLocalCornersSpan[supportedCorners] <- localCorner
                            supportedCorners <- supportedCorners + 1
                        else
                            unsupportedCornersSpan[unsupportedCorners] <- worldCorner
                            unsupportedCorners <- unsupportedCorners + 1
                            
                    match supportedCorners with
                    | 2 -> 
                        let local1 = supportedLocalCornersSpan[0]
                        let local2 = supportedLocalCornersSpan[1]

                        if (local1.X = -local2.X) && (local1.Y = -local2.Y) then
                           // We found support on diagonal corners-this is stable for bodies with a square base,
                           // for rectangular ones this is not true, but for simplicity we will assume that it is also stable
                           ValueNone
                        else
                            // Game design decision for robust falling behavior.
                            // Instead of pivoting around the supported edge (physically accurate),
                            // we pivot around the unsupported edge. This ensures the body
                            // moves away from its unstable position and guarantees it will fall off
                            let pivotEdgeCorner1 = unsupportedCornersSpan[0]
                            let pivotEdgeCorner2 = unsupportedCornersSpan[1]
                            
                            let pivotPoint = (pivotEdgeCorner1 + pivotEdgeCorner2) / 2.0
                            let mutable fallDirection = projectedCenterOfMass - pivotPoint
                            WorldLimits.relative &fallDirection

                            let mutable initialRotationAxis = pivotEdgeCorner2 - pivotEdgeCorner1
                            WorldLimits.relative &initialRotationAxis

                            if initialRotationAxis.MagnitudeSq() < EPSILON_X2 then
                                ValueNone
                            else
                                let crossProductZ = initialRotationAxis.X * fallDirection.Y - initialRotationAxis.Y * fallDirection.X
                                let finalRotationAxis = if crossProductZ < 0.0 then -initialRotationAxis else initialRotationAxis
                                ValueSome(struct(pivotPoint, finalRotationAxis.Normalize()))
                    
                    | 1 ->
                        let pivotPoint = supportedCornersSpan[0]
                        let mutable fallDirection = projectedCenterOfMass - pivotPoint
                        WorldLimits.relative &fallDirection
                        fallDirection.Z <- 0.0

                        if fallDirection.MagnitudeSq() < EPSILON_X2 then
                            ValueNone
                        else
                            let rotationAxis = Vector3.Cross(fallDirection, Vector3.Up)
                            let finalAxis = if rotationAxis.MagnitudeSq() < EPSILON_X2 then Vector3.UnitX else rotationAxis.Normalize()
                            ValueSome(struct(pivotPoint, finalAxis))
                    
                    | _ -> ValueNone
                    
        let inline private getStableFrictionNormal (normal: Vector3) (o1: Matrix3x3) (o2: Matrix3x3) =
            if abs(Vector3.Dot(o1.R2, o2.R2)) > 0.98 && abs(normal.Z) > 0.9 then
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
                let finalNormal = result.Normal
                let penetrationDepth = result.Depth

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
            dt =
            let finalInvMass1 = if b1.IsFallingOver then 0.0 else b1.InvMass
            let finalInvMass2 = if b2.IsFallingOver then 0.0 else b2.InvMass
            let totalInvMass = finalInvMass1 + finalInvMass2
        
            if totalInvMass > EPSILON then
                let mutable effectiveNormalImpulse = normalImpulseSum

                let gravityComponent = Vector3.Dot(GRAVITY, normal)
                if gravityComponent < 0.0 then
                    let mass1 = if finalInvMass1 > EPSILON then 1.0/finalInvMass1 else 0.0
                    let mass2 = if finalInvMass2 > EPSILON then 1.0/finalInvMass2 else 0.0
                    let gravityForceAlongNormal = (mass1 + mass2) * -gravityComponent
                    let gravityImpulse = gravityForceAlongNormal * dt
                    effectiveNormalImpulse <- effectiveNormalImpulse + gravityImpulse

                if effectiveNormalImpulse > 0.0 then
                    let frictionNormal = getStableFrictionNormal normal b1.Orientation b2.Orientation

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
                let inline safeMass invMass =
                    if invMass > EPSILON then 1.0 / invMass else STATIC_BODY_MASS
    
                let finalNormal = result.Normal
                let penetrationDepth = result.Depth
                let velAlongNormal = Vector3.Dot(b1.Velocity, finalNormal)
                let mutable totalImpulseScalar = 0.0

                if velAlongNormal < 0.0 then
                    let e = if abs velAlongNormal < RESTITUTION_THRESHOLD then 0.0 else 0.1
                    
                    let velocityToCancel = finalNormal * velAlongNormal
                    b1.Velocity <- b1.Velocity - velocityToCancel * (1.0 + e)

                    let m1 = b1.InvMass |> safeMass  
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
                let mutable effectiveNormalImpulse = normalImpulseSum
                let gravityComponent = Vector3.Dot(GRAVITY, normal)
                if gravityComponent < 0.0 then
                    let gravityForceAlongNormal = (1.0 / invMass1) * (-gravityComponent)
                    let gravityImpulse = gravityForceAlongNormal * dt
                    effectiveNormalImpulse <- effectiveNormalImpulse + gravityImpulse
            
                if effectiveNormalImpulse > 0.0 then
                    let frictionNormal = getStableFrictionNormal normal b1.Orientation Matrix3x3.Identity

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

            use bodiesToInitiateFall = new PooledList<int>()
            let activeIslands = islandRepo |> Island.getActiveIslandIds
            for islandId in activeIslands do
                let island = &Island.getIslandRef islandId islandRepo
                for id in island.BodiesSpan do
                    let body = &Body.getRef id bodyRepo
                    if body.IsForceFalling && not <| body.IsFallingOver then
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
                for id in island.BodiesSpan do
                    let body = &Body.getRef id bodyRepo
                    if body.IsFallingOver then                
                        let previousPosition = body.Position
                        body.FallRotationProgress <- min (body.FallRotationProgress + sub_dt / body.FallDuration) 1.0

                        let totalRotationMatrix = Matrix3x3.CreateRotation(body.FallRotationAxis, (PI / 2.0) * body.FallRotationProgress)
     
                        body.Orientation <- totalRotationMatrix * body.FallInitialOrientation

                        let mutable idealNewPosition = body.FallPivotPoint + totalRotationMatrix * body.InitialCenterOffsetFromPivot
                        WorldLimits.wrapPosition &idealNewPosition
                        body.Position <- idealNewPosition

                        let radiusZ = Collision.getProjectedRadiusZ body.Dimensions body.Orientation
                        let lowestPointZ = body.Position.Z - radiusZ

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
                        while not <| hasStoppedOnStatic && cellIdx < buffers.UniquePrismsBuffer.Count do
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
                                        
                                if result.AreColliding && result.Depth > PENETRATION_SLOP then
                                    hasStoppedOnStatic <- true
                                    penetrationVector <- result.Normal * result.Depth
                                    
                            cellIdx <- cellIdx + 1

                        for otherId in island.BodiesSpan do
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

                            for otherId in island.BodiesSpan do
                                processCollidingBody bodyRepo &body islandRepo finalPosition finalDimensions finalOrientation otherId

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
        
        let private getRadiusZ (b: inref<Body.T>) (radiusZ: byref<double>) =
            if radiusZ < 0.0 then
                radiusZ <- Collision.getProjectedRadiusZ b.Dimensions b.Orientation
            
            radiusZ
                        
        let mutable private _verticalLimiter = Body.T()
        let private resolveFloorAndCeilingCollisions (b1: byref<Body.T>) invMass1 islandRepo dt =
            if invMass1 > EPSILON then
                let mutable radiusZ = -1.0

                if b1.MinAABB.Z < PENETRATION_SLOP then
                    let lowestPointZ = b1.Position.Z - getRadiusZ &b1 &radiusZ
                    
                    if lowestPointZ < 0.0 then
                        let penetrationDepth = -lowestPointZ
                        let floorCollisionResult = CollisionResult.Create(Vector3.Up, penetrationDepth)
                        let totalImpulseScalar =
                            resolveDynamicCollision
                                &b1
                                &_verticalLimiter
                                floorCollisionResult
                                invMass1
                                0.0
                                
                        let finalNormal = floorCollisionResult.Normal
                        resolveStaticFriction
                            &b1
                            finalNormal
                            totalImpulseScalar
                            invMass1
                            dt
                        
                        let island = &Island.getIslandRefForBody b1.Id islandRepo
                        island.UpdateMaxPenetration penetrationDepth
                        island.IsGrounded <- true

                if b1.MaxAABB.Z > WORLD_HEIGHT_IN_METERS - PENETRATION_SLOP then
                    let highestPointZ = b1.Position.Z + getRadiusZ &b1 &radiusZ

                    let topPenetration = highestPointZ - WORLD_HEIGHT_IN_METERS
                    if topPenetration > 0.0 then
                        let ceilingCollisionResult = CollisionResult.Create(Vector3.Down, topPenetration)
                        resolveDynamicCollision
                            &b1
                            &_verticalLimiter
                            ceilingCollisionResult
                            b1.InvMass
                            0.0
                        |> ignore
        
        let private resolveDynamicDynamicCollision
            (b1: byref<Body.T>)
            (b2: byref<Body.T>)
            (island1: byref<Island.T>)
            (island2: byref<Island.T>)
            islandRepo
            dt =
            
            let minA = b1.MinAABB
            let maxA = b1.MaxAABB
            let minB = b2.MinAABB
            let maxB = b2.MaxAABB
            
            if Collision.checkCollisionAABB minA maxA minB maxB then
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
                    let totalImpulseScalar =
                        resolveDynamicCollision
                            &b1
                            &b2
                            result
                            b1.InvMass
                            b2.InvMass
                                                
                    resolveDynamicFriction
                        &b1
                        &b2
                        result.Normal
                        totalImpulseScalar
                        dt

                    let contactKey = ContactKey.key b1.Id b2.Id
                    island1.AddOrUpdateContact(contactKey, CONTACT_TTL, newCachedAxis)
                    island1.UpdateMaxPenetration result.Depth
                    
                    if island1.Id <> island2.Id then
                        islandRepo |> Island.requestMerge island1.Id island2.Id
                        island2.AddOrUpdateContact(contactKey, CONTACT_TTL, newCachedAxis)                              
                        island2.UpdateMaxPenetration result.Depth
                        
                    b1.IsSnappedToGrid <- false
                    b2.IsSnappedToGrid <- false
                elif checkBodyProximity minA maxA minB maxB then
                    if island1.Id = island2.Id then
                        let contactKey = ContactKey.key b1.Id b2.Id
                        island1.AddOrUpdateContact(contactKey, CONTACT_TTL, newCachedAxis)
        
        let private resolveDynamicSleepingCollision
            (b1: byref<Body.T>)
            (b2: byref<Body.T>)
            (island1: byref<Island.T>)
            (island2: byref<Island.T>)
            islandRepo
            dt =
            let minA = b1.MinAABB
            let maxA = b1.MaxAABB
            let minB = b2.MinAABB
            let maxB = b2.MaxAABB
            
            if Collision.checkCollisionAABB minA maxA minB maxB then
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
                    let finalNormal = result.Normal
                    let penetrationDepth = result.Depth
                           
                    let totalImpulseScalar =
                        resolveDynamicCollision
                            &b1
                            &b2
                            result
                            b1.InvMass
                            b2.InvMass
                                                    
                    resolveDynamicFriction
                        &b1
                        &b2
                        finalNormal
                        totalImpulseScalar
                        dt
                          
                    Island.requestWakeIsland &island2 islandRepo
                    islandRepo |> Island.requestMerge island2.Id island1.Id

                    let contactKey = ContactKey.key b1.Id b2.Id
                    island1.AddOrUpdateContact(contactKey, CONTACT_TTL, newCachedAxis)
                    island1.UpdateMaxPenetration penetrationDepth
                    island2.AddOrUpdateContact(contactKey, CONTACT_TTL, newCachedAxis)
                    island2.UpdateMaxPenetration penetrationDepth
                    
                    b1.IsSnappedToGrid <- false
                    b2.IsSnappedToGrid <- false
        
        let private resolveStaticGeometryCollision
            (b1: byref<Body.T>)
            (island: byref<Island.T>)
            (prismData: inref<struct(Vector3 * Vector3 * Matrix3x3)>)
            dt =
            let struct(staticPos, staticDims, staticOrient) = prismData
            let result =
                Collision.checkCollisionSAT
                    b1.Position
                    b1.Dimensions
                    b1.Orientation
                    staticPos
                    staticDims
                    staticOrient

            if result.AreColliding then
                let finalNormal = result.Normal
                let penetrationDepth = result.Depth
                let totalImpulseScalar = resolveStaticCollision &b1 result
                                        
                resolveStaticFriction &b1 finalNormal totalImpulseScalar b1.InvMass dt
                island.UpdateMaxPenetration penetrationDepth
                island.IsGroundedCacheValid <- true
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
                        let finalNormal = result.Normal
                        let penetrationDepth = result.Depth
                        island.UpdateMaxPenetration penetrationDepth
                        let totalImpulseScalar = resolveStaticCollision &b1 result
                        resolveStaticFriction &b1 finalNormal totalImpulseScalar b1.InvMass dt
        
        let private resolveNarrowPhaseCollisions
            collidingIslandPairs
            sub_dt
            engine =
            
            let buffers = engine._buffers
            let islandRepo = engine._islandRepo
            let bodyRepo = engine._bodyRepo
            let geometryRepo = engine._geometryRepo
            let floraRepo = engine._floraRepo
            let spatialHash = engine._spatialHash
            let rnd = engine._random

            let checkedBodyPairs = buffers.CollisionCheckedBodyPairs
            let destroyedFlora = buffers.CollisionDestroyedFlora
            let staticCache = engine._staticCache
            let processedStatic = buffers.CollisionProcessedStatic
            let processedFlora = buffers.CollisionProcessedFlora
                    
            checkedBodyPairs.Clear()
            destroyedFlora.Clear()

            for islandId in islandRepo |> Island.getActiveIslandIds do
                let island = &Island.getIslandRef islandId islandRepo
                let bodiesInIsland = island.BodiesSpan
                for i = 0 to bodiesInIsland.Length - 1 do
                    let id1 = bodiesInIsland[i]
                    let b1 = &Body.getRef id1 bodyRepo

                    let invMass = if b1.IsFallingOver then 0.0 else b1.InvMass
                    resolveFloorAndCeilingCollisions &b1 invMass islandRepo sub_dt

                    let occupiedCells = SpatialHash.getOccupiedCells id1 spatialHash
                    
                    processedStatic.Clear()
                    processedFlora.Clear()

                    for cellKey in occupiedCells do
                        if geometryRepo |> Geometry.isSolid cellKey then
                            let mutable isPrismDataExists = false
                            let prismData = &CollectionsMarshal.GetValueRefOrAddDefault(staticCache, cellKey, &isPrismDataExists)

                            if not <| isPrismDataExists then
                                prismData <- cellKey |> Grid.getPrismSpaceByKey

                            if processedStatic.Add cellKey then
                                resolveStaticGeometryCollision
                                    &b1
                                    &island
                                    &prismData
                                    sub_dt

                        match floraRepo |> Flora.tryGetTreesInCell cellKey with
                        | true, treeIds ->
                            for treeId in treeIds.Span do
                                if not <| destroyedFlora.Contains treeId && processedFlora.Add treeId then
                                    resolveFloraCollision &b1 treeId floraRepo &island buffers rnd sub_dt
                        | _ -> ()
                    
                    for j = i + 1 to bodiesInIsland.Length - 1 do
                        let id2 = bodiesInIsland[j]
                        let b2 = &Body.getRef id2 bodyRepo
                        resolveDynamicDynamicCollision
                            &b1
                            &b2
                            &island
                            &island
                            islandRepo
                            sub_dt
                        
            for pairKey in collidingIslandPairs do
                let struct(islandId1, islandId2) = ContactKey.unpack pairKey            
                let mutable island1 = &Island.getIslandRef islandId1 islandRepo
                let mutable island2 = &Island.getIslandRef islandId2 islandRepo

                for bodyId1 in island1.BodiesSpan do
                    for bodyId2 in island2.BodiesSpan do                       
                        let contactKey = ContactKey.key bodyId1 bodyId2
                        if checkedBodyPairs.Add contactKey then
                            let b1 = &Body.getRef bodyId1 bodyRepo
                            let b2 = &Body.getRef bodyId2 bodyRepo
                            
                            if island2.IsAwake then
                                resolveDynamicDynamicCollision
                                    &b1
                                    &b2
                                    &island1
                                    &island2
                                    islandRepo
                                    sub_dt
                            else
                                resolveDynamicSleepingCollision
                                    &b1
                                    &b2
                                    &island1
                                    &island2
                                    islandRepo
                                    sub_dt
                        
        let private postProcessAndUpdateSleepState engine =
            let islandRepo = engine._islandRepo
            let bodyRepo = engine._bodyRepo
            let floraRepo = engine._floraRepo
            let geometryRepo = engine._geometryRepo
            let spatialHash = engine._spatialHash
            let dt = engine._dt
            use processedBushes = new PooledSet<int>()
            
            for islandId in islandRepo |> Island.getActiveIslandIds do
                let island = &Island.getIslandRef islandId islandRepo
                if island.IsAwake then
                    if island.Bodies.Count > 1 && island.HadContactsRemovedThisStep && island.ContactCount < island.Bodies.Count - 1 then
                        islandRepo |> Island.requestSplit island.Id
                    let mutable isStillSlow = true
                    let mutable i = 0
                    while isStillSlow && i < island.BodiesSpan.Length do
                        let bodyId = island.BodiesSpan[i]
                        i <- i + 1
                        let body = &Body.getRef bodyId bodyRepo
                        processedBushes.Clear()
                        let occupiedCells = SpatialHash.getOccupiedCells bodyId spatialHash
                        for pCoords in occupiedCells do
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
                        if island.MaxPenetrationThisStep > DISABLE_SLEEP_ISLAND_PENETRATION then
                            island.FramesResting <- 0
                        elif Island.isGrounded bodyRepo geometryRepo spatialHash &island islandRepo then
                            island.FramesResting <- island.FramesResting + 1
                            
                            if island.FramesResting >= FRAMES_TO_SLEEP && island.CantSleepFrames <= 0 then
                                let mutable isIslandStable = true
                                let mutable i = 0
                                while isIslandStable && i < island.BodiesSpan.Length do
                                    let bodyId = island.BodiesSpan[i]
                                    i <- i + 1
                                    let body = &Body.getRef bodyId bodyRepo
                                    match tryInitiateFall &body geometryRepo bodyRepo islandRepo engine._buffers with
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
                                    for bodyId in island.BodiesSpan do
                                        let body = &Body.getRef bodyId bodyRepo
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
            let spatialHash = engine._spatialHash
            
            for coords in buffers.PrismsToRemoveCoords.Span do
                geometryRepo |> Geometry.removePrism coords |> ignore
                engine._staticCache.Remove coords |> ignore
                
            if buffers.FloraToRemoveIds.Count > 0 then
                for id in buffers.FloraToRemoveIds.Span do
                    floraRepo |> Flora.removeTree id
                    floraRepo |> Flora.removeBush id

                floraRepo |> Flora.updateHashesForRemovedFlora buffers.FloraToRemoveIds

            for id in buffers.BodiesToRemoveIds.Span do
                bodyRepo |> Body.remove id |> ignore
                islandRepo |> Island.removeBody id
                spatialHash |> SpatialHash.remove id

            for i = 0 to buffers.BodiesToAdd.Span.Length - 1 do
                let body = &buffers.BodiesToAdd.Span[i]
                Body.tryAdd &body bodyRepo |> ignore
                islandRepo |> Island.addBody body.Id
                
        let step engine =

            engine |> applyDeferredChanges

            engine._islandRepo |> Island.processIslandChanges engine._spatialHash engine._buffers
                        
            let buffers = engine._buffers
            buffers.Clear()
            
            for islandId in engine._islandRepo.ActiveIslandIds do
                let mutable island = &Island.getIslandRef islandId engine._islandRepo
                island.NextStep() 
                for bodyId in island.BodiesSpan do
                    let body = &Body.getRef bodyId engine._bodyRepo
                    SpatialHash.update &body buffers engine._spatialHash
                    
                    if not <| body.IsFallingOver && not <| body.IsSnappedToGrid && body.IsGravityEnabled then
                        body.Velocity <- body.Velocity + GRAVITY * engine._dt
                                            
                Island.recalculateAABBAndUpdateGrid &island engine.Islands

            let collidingIslandPairs = engine._islandRepo |> Island.detectBroadPhaseIslandPairs buffers
            
            let resolutionPasses = 8
            let sub_dt = engine._dt / double resolutionPasses

            for _ = 1 to resolutionPasses do
                engine |> applyKinematicUpdates sub_dt

                engine |> resolveNarrowPhaseCollisions collidingIslandPairs sub_dt

                for islandId in engine._islandRepo.ActiveIslandIds do
                    let island = &Island.getIslandRef islandId engine._islandRepo
                    for bodyId in island.BodiesSpan do
                        let body = &Body.getRef bodyId engine._bodyRepo
                        if not <| body.IsFallingOver then                          
                            let vSq = body.Velocity.MagnitudeSq()
                            if vSq > MAX_SPEED_SQ then
                                body.Velocity <- body.Velocity.Normalize() * MAX_SPEED
                                
                            body.Position <- body.Position + body.Velocity * sub_dt
                            WorldLimits.wrapPosition &body.Position
                
            engine._floraRepo |> Flora.updatePhysics buffers.UnrootedFlora buffers.FloraToRemoveIds
            engine._geometryRepo |> Geometry.updatePhysics buffers.FallingPrisms buffers.PrismsToRemoveCoords
            engine._liquidRepo |> Liquid.updatePhysics

            for data in buffers.UnrootedFlora.Span do
                if not <| buffers.BodiesToAdd.Exists(fun b -> b.Id = data.Id) then
                     let mutable newBody = Flora.createBodyFromFlora data.Id data
                     newBody.IsForceFalling <- true
                     engine._buffers.BodiesToAdd.Add newBody
                     
            for fallingPrism in buffers.FallingPrisms.Span do
                if not <| buffers.BodiesToAdd.Exists(fun b -> b.Id = fallingPrism.Id) then
                    buffers.BodiesToAdd.Add fallingPrism

            engine |> postProcessAndUpdateSleepState