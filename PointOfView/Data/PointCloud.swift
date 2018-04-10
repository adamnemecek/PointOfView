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
    public struct ParseError: Error {
    }
    
    private enum Octree {
        case leaf([Index])
        case branch([Octree])
    }
    
    private struct OctreeBounds {
        static let base = OctreeBounds(x: -1 ... 1, y: -1 ... 1, z: -1 ... 1)
        
        let x: ClosedRange<Float>
        let y: ClosedRange<Float>
        let z: ClosedRange<Float>
        
        func subtreeBounds(at index: Int) -> OctreeBounds {
            return .init(
                x: index & 1 != 0 ? x.center ... x.upperBound : x.lowerBound ... x.center,
                y: index & 2 != 0 ? y.center ... y.upperBound : y.lowerBound ... y.center,
                z: index & 4 != 0 ? z.center ... z.upperBound : z.lowerBound ... z.center
            )
        }
        
        func subtreeIndex(for position: (x: Float, y: Float, z: Float)) -> Int {
            let xBit = position.x > x.center
            let yBit = position.y > y.center
            let zBit = position.z > z.center
            return (xBit ? 1 : 0) + (yBit ? 2 : 0) + (zBit ? 4 : 0)
        }
    }
    
    public let count: Int
    
    internal let xPositions: UnsafeMutablePointer<Float>
    internal let yPositions: UnsafeMutablePointer<Float>
    internal let zPositions: UnsafeMutablePointer<Float>
    internal let raddi: UnsafeMutablePointer<Float>
    internal let intensities: UnsafeMutablePointer<UInt8>
    
    private let octree: Octree
    
    public let latitudeBounds: ClosedRange<Double>
    public let longitudeBounds: ClosedRange<Double>
    public let elevationBounds: ClosedRange<Double>
    
    deinit {
        xPositions.deallocate()
        yPositions.deallocate()
        zPositions.deallocate()
        raddi.deallocate()
        intensities.deallocate()
    }
    
    public init(contentsOf url: URL, pointsPerLeaf: Int = 16) throws {
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
        
        self.latitudeBounds = ensuredLatitudeBounds
        self.longitudeBounds = ensuredLongitudeBounds
        self.elevationBounds = ensuredElevationBounds
        
        let count = latitudes.count
        let xPositions = UnsafeMutableRawPointer.allocate(byteCount: (MemoryLayout<Float>.stride * count).aligned(to: 4096), alignment: 4096).bindMemory(to: Float.self, capacity: count)
        let yPositions = UnsafeMutableRawPointer.allocate(byteCount: (MemoryLayout<Float>.stride * count).aligned(to: 4096), alignment: 4096).bindMemory(to: Float.self, capacity: count)
        let zPositions = UnsafeMutableRawPointer.allocate(byteCount: (MemoryLayout<Float>.stride * count).aligned(to: 4096), alignment: 4096).bindMemory(to: Float.self, capacity: count)
        let raddi = UnsafeMutableRawPointer.allocate(byteCount: (MemoryLayout<Float>.stride * count).aligned(to: 4096), alignment: 4096).bindMemory(to: Float.self, capacity: count)
        let finalIntensities = UnsafeMutableRawPointer.allocate(byteCount: (MemoryLayout<UInt8>.stride * count).aligned(to: 4096), alignment: 4096).bindMemory(to: UInt8.self, capacity: count)
        
        var octree = Octree.leaf([])
        
        func balanceLeaf(_ octree: Octree, bounds: OctreeBounds) -> Octree {
            guard case let .leaf(points) = octree else {
                preconditionFailure()
            }
            
            guard points.count > pointsPerLeaf else {
                return octree
            }
            
            var subtreePoints = [[Index]](repeating: [], count: 8)
            
            for pointIndex in points {
                let xPosition = xPositions[pointIndex]
                let yPosition = yPositions[pointIndex]
                let zPosition = zPositions[pointIndex]
                subtreePoints[bounds.subtreeIndex(for: (xPosition, yPosition, zPosition))].append(pointIndex)
            }
            
            let subtrees = [
                balanceLeaf(.leaf(subtreePoints[0]), bounds: bounds.subtreeBounds(at: 0)),
                balanceLeaf(.leaf(subtreePoints[1]), bounds: bounds.subtreeBounds(at: 1)),
                balanceLeaf(.leaf(subtreePoints[2]), bounds: bounds.subtreeBounds(at: 2)),
                balanceLeaf(.leaf(subtreePoints[3]), bounds: bounds.subtreeBounds(at: 3)),
                balanceLeaf(.leaf(subtreePoints[4]), bounds: bounds.subtreeBounds(at: 4)),
                balanceLeaf(.leaf(subtreePoints[5]), bounds: bounds.subtreeBounds(at: 5)),
                balanceLeaf(.leaf(subtreePoints[6]), bounds: bounds.subtreeBounds(at: 6)),
                balanceLeaf(.leaf(subtreePoints[7]), bounds: bounds.subtreeBounds(at: 7)),
            ]
            
            return .branch(subtrees)
        }
        
        func addPointToOctree(_ octree: Octree, pointIndex: Int, position: (x: Float, y: Float, z: Float), bounds: OctreeBounds) -> Octree {
            switch octree {
            case var .leaf(points):
                points.append(pointIndex)
                return balanceLeaf(.leaf(points), bounds: bounds)
            case var .branch(subtrees):
                let subtreeIndex = bounds.subtreeIndex(for: position)
                subtrees[subtreeIndex] = addPointToOctree(subtrees[subtreeIndex], pointIndex: pointIndex, position: position, bounds: bounds.subtreeBounds(at: subtreeIndex))
                return .branch(subtrees)
            }
        }
        
        for index in 0 ..< count {
            xPositions[index] = .init((longitudes[index] - ensuredLongitudeBounds.center) / ensuredLongitudeBounds.halfLength)
            zPositions[index] = .init((latitudes[index] - ensuredLatitudeBounds.center) / ensuredLatitudeBounds.halfLength)
            yPositions[index] = .init((elevations[index] - ensuredElevationBounds.center) / ensuredElevationBounds.halfLength)
            raddi[index] = .infinity
            intensities[index] = intensities[index]
            
            let position = (x: xPositions[index], y: yPositions[index], z: zPositions[index])
            octree = addPointToOctree(octree, pointIndex: index, position: position, bounds: .base)
        }
        
        self.count = count
        self.xPositions = xPositions
        self.yPositions = yPositions
        self.zPositions = zPositions
        self.raddi = raddi
        self.intensities = finalIntensities
        self.octree = octree
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
