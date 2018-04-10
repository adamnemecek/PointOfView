import simd

extension float3 {
    static let xAxis = float3(x: 1, y: 0, z: 0)
    static let yAxis = float3(x: 0, y: 1, z: 0)
    static let zAxis = float3(x: 0, y: 0, z: 1)
    
    init(xy: float2, z: Float) {
        self.init(x: xy.x, y: xy.y, z: z)
    }
    
    func transformed(by matrix: float4x4) -> float3 {
        let homogenous = matrix * float4(xyz: self, w: 1)
        return homogenous.xyz / homogenous.w
    }
    
    func rotated(about axis: float3, by angle: Float) -> float3 {
        return self.transformed(by: float4x4.rotation(about: axis, by: angle))
    }
}

extension float4 {
    init(xyz: float3, w: Float) {
        self.init(x: xyz.x, y: xyz.y, z: xyz.z, w: w)
    }
    
    var xyz: float3 {
        get {
            return float3(x: x, y: y, z: z)
        }
        set {
            x = newValue.x
            y = newValue.y
            z = newValue.z
        }
    }
    
    init(x: Float, yzw: float3) {
        self.init(x: x, y: yzw.x, z: yzw.y, w: yzw.z)
    }
    
    init(x: Float, y: Float, zw: float2) {
        self.init(x: x, y: y, z: zw.x, w: zw.y)
    }
    
    init(xy: float2, z: Float, w: Float) {
        self.init(x: xy.x, y: xy.y, z: z, w: w)
    }
    
    init(xy: float2, zw: float2) {
        self.init(x: xy.x, y: xy.y, z: zw.x, w: zw.y)
    }
    
    func transformed(by matrix: float4x4) -> float4 {
        return matrix * self
    }
}

extension float4x4 {
    static var identity: float4x4 {
        return .init(diagonal: .init(1))
    }
    
    static func translation(by vector: float3) -> float4x4 {
        return .identity + .init(columns: (.init(0), .init(0), .init(0), .init(xyz: vector, w: 0)))
    }
    
    static func scaling(by vector: float3) -> float4x4 {
        return .init(diagonal: .init(xyz: vector, w: 1))
    }
    
    static func rotation(about axis: float3, by angle: Float) -> float4x4 {
        return float4x4(columns: (
            float4(
                x: cos(angle) + axis.x * axis.x * (1 - cos(angle)),
                y: axis.x * axis.y * (1 - cos(angle)) + axis.z * sin(angle),
                z: axis.x * axis.z * (1 - cos(angle)) - axis.y * sin(angle),
                w: 0
            ),
            float4(
                x: axis.x * axis.y * (1 - cos(angle)) - axis.z * sin(angle),
                y: cos(angle) + axis.y * axis.y * (1 - cos(angle)),
                z: axis.y * axis.z * (1 - cos(angle)) + axis.x * sin(angle),
                w: 0
            ),
            float4(
                x: axis.x * axis.z * (1 - cos(angle)) + axis.y * sin(angle),
                y: axis.y * axis.z * (1 - cos(angle)) - axis.x * sin(angle),
                z: cos(angle) + axis.z * axis.z * (1 - cos(angle)),
                w: 0
            ),
            float4(xyz: .init(0), w: 1)
        ))
    }
    
    static func lookat(forward: float3, up: float3) -> float4x4 {
        let normalizedForward = normalize(forward)
        let right = normalize(cross(up, forward))
        let orthogonalizedUp = normalize(cross(forward, right))
        return float4x4(columns: (
            float4(x: right.x, y: orthogonalizedUp.x, z: normalizedForward.x, w: 0),
            float4(x: right.y, y: orthogonalizedUp.y, z: normalizedForward.y, w: 0),
            float4(x: right.z, y: orthogonalizedUp.z, z: normalizedForward.z, w: 0),
            float4(xyz: .init(0), w: 1)
        ))
    }
    
    static func infinitePerspective(fovy: Float, nearDistance: Float, aspectRatio: Float) -> float4x4 {
        return float4x4(columns: (
            float4(x: 1 / tan(fovy / 2) / aspectRatio, yzw: .init(0)),
            float4(x: 0, y: 1 / tan(fovy / 2), zw: .init(0)),
            float4(xy: .init(0), zw: .init(1)),
            float4(xy: .init(0), z: -2 * nearDistance, w: 0)
        ))
    }
}
