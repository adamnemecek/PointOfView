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
    
    private class MutationCell<Wrapped> {
        var contents: Wrapped
        
        init(_ contents: Wrapped) {
            self.contents = contents
        }
    }
    
    private enum Octree {
        case `nil`
        case leaf([Index])
        case branch(MutationCell<(Octree, Octree, Octree, Octree, Octree, Octree, Octree, Octree)>)
        
        mutating func splitIfNeeded(xPositions: UnsafePointer<Float>, yPositions: UnsafePointer<Float>, zPositions: UnsafePointer<Float>, bounds: OctreeBounds, maximumPointsPerLeaf: Int) {
            switch self {
            case let .leaf(pointIndices):
                guard pointIndices.count > maximumPointsPerLeaf else { return }
                var subtreePointIndices = [[Index]](repeating: [], count: 8)
                
                for pointIndex in pointIndices {
                    let xPosition = xPositions[pointIndex]
                    let yPosition = yPositions[pointIndex]
                    let zPosition = zPositions[pointIndex]
                    subtreePointIndices[bounds.subtreeIndex(for: .init(x: xPosition, y: yPosition, z: zPosition))].append(pointIndex)
                }
    
                var subtrees = (
                    Octree.leaf(subtreePointIndices[0]),
                    Octree.leaf(subtreePointIndices[1]),
                    Octree.leaf(subtreePointIndices[2]),
                    Octree.leaf(subtreePointIndices[3]),
                    Octree.leaf(subtreePointIndices[4]),
                    Octree.leaf(subtreePointIndices[5]),
                    Octree.leaf(subtreePointIndices[6]),
                    Octree.leaf(subtreePointIndices[7])
                )
                
                subtrees.0.splitIfNeeded(xPositions: xPositions, yPositions: yPositions, zPositions: zPositions, bounds: bounds.subtreeBounds(at: 0), maximumPointsPerLeaf: maximumPointsPerLeaf)
                subtrees.1.splitIfNeeded(xPositions: xPositions, yPositions: yPositions, zPositions: zPositions, bounds: bounds.subtreeBounds(at: 1), maximumPointsPerLeaf: maximumPointsPerLeaf)
                subtrees.2.splitIfNeeded(xPositions: xPositions, yPositions: yPositions, zPositions: zPositions, bounds: bounds.subtreeBounds(at: 2), maximumPointsPerLeaf: maximumPointsPerLeaf)
                subtrees.3.splitIfNeeded(xPositions: xPositions, yPositions: yPositions, zPositions: zPositions, bounds: bounds.subtreeBounds(at: 3), maximumPointsPerLeaf: maximumPointsPerLeaf)
                subtrees.4.splitIfNeeded(xPositions: xPositions, yPositions: yPositions, zPositions: zPositions, bounds: bounds.subtreeBounds(at: 4), maximumPointsPerLeaf: maximumPointsPerLeaf)
                subtrees.5.splitIfNeeded(xPositions: xPositions, yPositions: yPositions, zPositions: zPositions, bounds: bounds.subtreeBounds(at: 5), maximumPointsPerLeaf: maximumPointsPerLeaf)
                subtrees.6.splitIfNeeded(xPositions: xPositions, yPositions: yPositions, zPositions: zPositions, bounds: bounds.subtreeBounds(at: 6), maximumPointsPerLeaf: maximumPointsPerLeaf)
                subtrees.7.splitIfNeeded(xPositions: xPositions, yPositions: yPositions, zPositions: zPositions, bounds: bounds.subtreeBounds(at: 7), maximumPointsPerLeaf: maximumPointsPerLeaf)
                
                self = .branch(MutationCell(subtrees))
            default:
                preconditionFailure()
            }
        }
        
        mutating func insertPoint(xPositions: UnsafePointer<Float>, yPositions: UnsafePointer<Float>, zPositions: UnsafePointer<Float>, pointIndex: Int, pointPosition: float3, bounds: OctreeBounds, maximumPointsPerLeaf: Int) {
            switch self {
            case var .leaf(points):
                self = .nil
                points.append(pointIndex)
                self = .leaf(points)
                self.splitIfNeeded(xPositions: xPositions, yPositions: yPositions, zPositions: zPositions, bounds: bounds, maximumPointsPerLeaf: maximumPointsPerLeaf)
            case let .branch(subtrees):
                let subtreeIndex = bounds.subtreeIndex(for: pointPosition)
                withUnsafeMutableBytes(of: &subtrees.contents) {
                    $0.bindMemory(to: Octree.self)[subtreeIndex].insertPoint(xPositions: xPositions, yPositions: yPositions, zPositions: zPositions, pointIndex: pointIndex, pointPosition: pointPosition, bounds: bounds.subtreeBounds(at: subtreeIndex), maximumPointsPerLeaf: maximumPointsPerLeaf)
                }
            default:
                preconditionFailure()
            }
        }
    }
    
    private struct OctreeBounds {
        static let base = OctreeBounds(x: -1 ... 1, y: -1 ... 1, z: -1 ... 1)
        
        let x: ClosedRange<Float>
        let y: ClosedRange<Float>
        let z: ClosedRange<Float>
        
        var lowerBound: float3 {
            return .init(x: x.lowerBound, y: y.lowerBound, z: z.lowerBound)
        }
        
        var upperBound: float3 {
            return .init(x: x.upperBound, y: y.upperBound, z: z.upperBound)
        }
        
        func subtreeBounds(at index: Int) -> OctreeBounds {
            return .init(
                x: index & 1 != 0 ? x.center ... x.upperBound : x.lowerBound ... x.center,
                y: index & 2 != 0 ? y.center ... y.upperBound : y.lowerBound ... y.center,
                z: index & 4 != 0 ? z.center ... z.upperBound : z.lowerBound ... z.center
            )
        }
        
        func subtreeIndex(for position: float3) -> Int {
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
    
    public init(contentsOf url: URL, maximumPointsPerLeaf: Int = 128) throws {
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
        
        for index in 0 ..< count {
            xPositions[index] = .init((longitudes[index] - ensuredLongitudeBounds.center) / ensuredLongitudeBounds.halfLength)
            zPositions[index] = .init((latitudes[index] - ensuredLatitudeBounds.center) / ensuredLatitudeBounds.halfLength)
            yPositions[index] = .init((elevations[index] - ensuredElevationBounds.center) / ensuredElevationBounds.halfLength)
            raddi[index] = .infinity
            intensities[index] = intensities[index]
            
            let position = float3(x: xPositions[index], y: yPositions[index], z: zPositions[index])
            octree.insertPoint(xPositions: xPositions, yPositions: yPositions, zPositions: zPositions, pointIndex: index, pointPosition: position, bounds: .base, maximumPointsPerLeaf: maximumPointsPerLeaf)
        }
        
        self.count = count
        self.xPositions = xPositions
        self.yPositions = yPositions
        self.zPositions = zPositions
        self.raddi = raddi
        self.intensities = finalIntensities
        self.octree = octree
        
        for index in 0 ..< count {
            raddi[index] = self.nearestPoint(from: self[index].position).distance / 2
        }
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

extension PointCloud {
    func nearestPoint(from position: float3) -> (distance: Float, index: Index) {
        func recurse(octree: Octree, bounds: OctreeBounds = .base) -> (distance: Float, index: Index) {
            switch octree {
            case let .leaf(points):
                return (zip(points.lazy.map { length(self[$0].position - position) }, points).min { $0.0 < $1.0 })!
            case let .branch(subtrees):
                let subtreeIndex = bounds.subtreeIndex(for: position)
                
                var nearest = withUnsafeBytes(of: &subtrees.contents, { recurse(octree: $0.bindMemory(to: Octree.self)[subtreeIndex], bounds: bounds.subtreeBounds(at: subtreeIndex))})
                
                for alternativeSubtreeIndex in (0 ..< 7).lazy.filter({ $0 != subtreeIndex }) {
                    let alternativeSubtreeBounds = bounds.subtreeBounds(at: alternativeSubtreeIndex)
                    let distanceToAlternativeSubtree = Swift.min(abs(position - alternativeSubtreeBounds.lowerBound).min()!, abs(position - alternativeSubtreeBounds.upperBound).min()!)
                    if distanceToAlternativeSubtree < nearest.distance {
                        let alternativeNearest = withUnsafeBytes(of: &subtrees.contents, { recurse(octree: $0.bindMemory(to: Octree.self)[alternativeSubtreeIndex], bounds: alternativeSubtreeBounds)})
                        if alternativeNearest.distance < nearest.distance {
                            nearest = alternativeNearest
                        }
                    }
                }
                
                return nearest
            default:
                preconditionFailure()
            }
        }
        return recurse(octree: self.octree)
    }
}
