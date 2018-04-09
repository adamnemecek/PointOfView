extension Int {
    func aligned(to alignment: Int) -> Int {
        return (self + alignment - 1) / alignment * alignment
    }
}
