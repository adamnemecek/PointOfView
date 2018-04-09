extension ClosedRange {
    func including(_ bound: Bound) -> ClosedRange {
        return (lowerBound > bound ? bound : lowerBound) ... (upperBound < bound ? bound : upperBound)
    }
    
    mutating func include(_ bound: Bound) {
        self = self.including(bound)
    }
}

extension ClosedRange where Bound: FloatingPoint, Bound: ExpressibleByFloatLiteral {
    var center: Bound { return (upperBound - lowerBound) * 0.5 }
}
