extension ClosedRange {
    func including(_ bound: Bound) -> ClosedRange {
        return (lowerBound > bound ? bound : lowerBound) ... (upperBound < bound ? bound : upperBound)
    }

    mutating func include(_ bound: Bound) {
        self = self.including(bound)
    }
}

extension ClosedRange where Bound: FloatingPoint, Bound: ExpressibleByFloatLiteral {
    var length: Bound { return upperBound - lowerBound }
    var halfLength: Bound { return length / 2 }
    var center: Bound { return lowerBound + halfLength }
}
