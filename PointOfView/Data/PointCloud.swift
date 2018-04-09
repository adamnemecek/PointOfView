import simd
import Foundation

@inline(__always)
fileprivate func parseInteger(bytes: UnsafeBufferPointer<UInt8>.SubSequence) -> Int {
    var sum = 0
    for byte in bytes {
        sum *= 10
        sum += Int(byte - .init(ascii: "0"))
    }
    return sum
}

@inline(__always)
fileprivate func parseDecimal(bytes: UnsafeBufferPointer<UInt8>.SubSequence) throws -> Double {
    guard let decimalPoint = bytes.index(of: .init(ascii: ".")) else { throw PointCloud.ParseError() }
    let integralPart = parseInteger(bytes: bytes[..<decimalPoint])
    let decimalPart = parseInteger(bytes: bytes[(decimalPoint + 1)...])
    return Double(integralPart) + Double(decimalPart) * pow(10, -Double(bytes[(decimalPoint + 1)...].count))
}

public struct Point {
    public let position: float3
    public let intensity: UInt8
}

public class PointCloud {
    public struct ParseError: Error {}
    
    public let count: Int
    
    internal let xPositions: UnsafeMutablePointer<Float>
    internal let yPositions: UnsafeMutablePointer<Float>
    internal let zPositions: UnsafeMutablePointer<Float>
    internal let intensities: UnsafeMutablePointer<UInt8>
    
    public let latitudeBounds: ClosedRange<Double>
    public let longitudeBounds: ClosedRange<Double>
    public let elevationBounds: ClosedRange<Double>
    
    deinit {
        xPositions.deallocate()
        yPositions.deallocate()
        zPositions.deallocate()
        intensities.deallocate()
    }
    
    public init(contentsOf url: URL) throws {
        var latitudes: [Double] = []
        var longitudes: [Double] = []
        var elevations: [Double] = []
        var intensities: [UInt8] = []
        
        var latitudeBounds: ClosedRange<Double>? = nil
        var longitudeBounds: ClosedRange<Double>? = nil
        var elevationBounds: ClosedRange<Double>? = nil
        
        let data = try Data(contentsOf: url)
        try data.withUnsafeBytes { (bytes: UnsafePointer<UInt8>) in
            let buffer = UnsafeBufferPointer.init(start: bytes, count: data.count)
            var bufferIndex = buffer.startIndex
            while bufferIndex < buffer.endIndex {
                guard let firstSpace = buffer[bufferIndex...].index(of: .init(ascii: " ")) else { throw ParseError() }
                guard let secondSpace = buffer[(firstSpace + 1)...].index(of: .init(ascii: " ")) else { throw ParseError() }
                guard let thirdSpace = buffer[(secondSpace + 1)...].index(of: .init(ascii: " ")) else { throw ParseError() }
                guard let newline = buffer[(thirdSpace + 1)...].index(of: .init(ascii: "\n")) else { throw ParseError() }
                
                let latitude = try parseDecimal(bytes: buffer[bufferIndex ..< firstSpace])
                let longitude = try parseDecimal(bytes: buffer[(firstSpace + 1) ..< secondSpace])
                let elevation = try parseDecimal(bytes: buffer[(secondSpace + 1) ..< thirdSpace])
                let intensity = UInt8(parseInteger(bytes: buffer[(thirdSpace + 1) ..< newline]))
                
                latitudeBounds = latitudeBounds.map { $0.including(latitude) } ?? latitude...latitude
                longitudeBounds = longitudeBounds.map { $0.including(longitude) } ?? longitude...longitude
                elevationBounds = elevationBounds.map { $0.including(elevation) } ?? elevation...elevation
                
                latitudes.append(latitude)
                longitudes.append(longitude)
                elevations.append(elevation)
                intensities.append(intensity)
                
                bufferIndex = newline + 1
            }
        }
        
        guard let ensuredLatitudeBounds = latitudeBounds else { throw ParseError() }
        guard let ensuredLongitudeBounds = longitudeBounds else { throw ParseError() }
        guard let ensuredElevationBounds = elevationBounds else { throw ParseError() }
        
        self.count = latitudes.count
        self.xPositions = UnsafeMutableRawPointer.allocate(byteCount: (MemoryLayout<Float>.stride * count).aligned(to: 4096), alignment: 4096).bindMemory(to: Float.self, capacity: count)
        self.yPositions = UnsafeMutableRawPointer.allocate(byteCount: (MemoryLayout<Float>.stride * count).aligned(to: 4096), alignment: 4096).bindMemory(to: Float.self, capacity: count)
        self.zPositions = UnsafeMutableRawPointer.allocate(byteCount: (MemoryLayout<Float>.stride * count).aligned(to: 4096), alignment: 4096).bindMemory(to: Float.self, capacity: count)
        self.intensities = UnsafeMutableRawPointer.allocate(byteCount: (MemoryLayout<UInt8>.stride * count).aligned(to: 4096), alignment: 4096).bindMemory(to: UInt8.self, capacity: count)
        
        for index in 0 ..< self.count {
            self.xPositions[index] = .init((longitudes[index] - ensuredLongitudeBounds.center) / ensuredLongitudeBounds.halfLength)
            self.zPositions[index] = .init((latitudes[index] - ensuredLatitudeBounds.center) / ensuredLatitudeBounds.halfLength)
            self.yPositions[index] = .init((elevations[index] - ensuredElevationBounds.center) / ensuredElevationBounds.halfLength)
            self.intensities[index] = intensities[index]
        }
        
        self.latitudeBounds = ensuredLatitudeBounds
        self.longitudeBounds = ensuredLongitudeBounds
        self.elevationBounds = ensuredElevationBounds
    }
}

extension PointCloud: RandomAccessCollection {
    public var startIndex: Int {
        return 0
    }

    public var endIndex: Int {
        return count
    }

    public subscript(index: Int) -> Point {
        return .init(position: .init(x: xPositions[index], y: yPositions[index], z: zPositions[index]), intensity: intensities[index])
    }
}
