import MetalKit

public class ViewController: NSViewController, MTKViewDelegate {
    public private(set) var device: MTLDevice! = nil
    private var commandQueue: MTLCommandQueue! = nil
    private var plottingPipelineState: MTLRenderPipelineState! = nil
    
    public var pointCloud: PointCloud? = nil { willSet {
        pointCloudBuffers = nil
        guard let pointCloud = newValue else { return }
        guard let xPositions = pointCloud.xPositions.withUnsafeBytes({
            device.makeBuffer(bytesNoCopy: UnsafeMutableRawPointer(mutating: $0.baseAddress!), length: $0.count, options: .storageModeShared, deallocator: nil)
        }) else { return }
        guard let yPositions = pointCloud.yPositions.withUnsafeBytes({
            device.makeBuffer(bytesNoCopy: UnsafeMutableRawPointer(mutating: $0.baseAddress!), length: $0.count, options: .storageModeShared, deallocator: nil)
        }) else { return }
        guard let zPositions = pointCloud.zPositions.withUnsafeBytes({
            device.makeBuffer(bytesNoCopy: UnsafeMutableRawPointer(mutating: $0.baseAddress!), length: $0.count, options: .storageModeShared, deallocator: nil)
        }) else { return }
        guard let intensities = pointCloud.intensities.withUnsafeBytes({
            device.makeBuffer(bytesNoCopy: UnsafeMutableRawPointer(mutating: $0.baseAddress!), length: $0.count, options: .storageModeShared, deallocator: nil)
        }) else { return }
        pointCloudBuffers = (xPositions, yPositions, zPositions, intensities)
    }}
    private var pointCloudBuffers: (xPositions: MTLBuffer, yPositions: MTLBuffer, zPositions: MTLBuffer, intensities: MTLBuffer)? = nil
    
    var cameraOrbit: (theta: Float, phi: Float) = (0, 0)
    var cameraDistance: Float = 1
    
    public override func viewDidLoad() {
        guard let view = self.view as? MTKView else {
            fatalError("The ViewController must be instantiated with an MTKView")
        }
        guard let device = MTLCreateSystemDefaultDevice() else {
            fatalError("No usable graphics device found")
        }
        guard let commandQueue = device.makeCommandQueue() else {
            fatalError("Could not make a command queue for the selected graphics device")
        }
        guard let library = device.makeDefaultLibrary() else {
            fatalError("Could not load the bundled shader library")
        }
        
        self.device = device
        self.commandQueue = commandQueue
        
        view.autoResizeDrawable = true
        view.colorPixelFormat = .rgba16Float
        view.depthStencilPixelFormat = .depth32Float
        view.colorspace = CGColorSpace(name: CGColorSpace.linearSRGB)
        view.delegate = self
        view.device = device
        
        do {
            guard let vertexShader = library.makeFunction(name: "v_plotting") else {
                fatalError("The shader 'v_plotting' could not be loaded")
            }
            guard let fragmentShader = library.makeFunction(name: "f_plotting") else {
                fatalError("The shader 'f_plotting' could not be loaded")
            }
            let descriptor = MTLRenderPipelineDescriptor()
            descriptor.vertexFunction = vertexShader
            descriptor.fragmentFunction = fragmentShader
            descriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
            descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
            descriptor.inputPrimitiveTopology = .point
            plottingPipelineState = try device.makeRenderPipelineState(descriptor: descriptor)
        }
        catch let error {
            fatalError("Could not compile the plotting shaders: \(error.localizedDescription)")
        }
    }
    
    public func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        return
    }
    
    public func draw(in view: MTKView) {
        guard let pointCloud = pointCloud else { return }
        guard let pointCloudBuffers = pointCloudBuffers else { return }
        guard let renderPassDescriptor = view.currentRenderPassDescriptor else { return }
        guard let drawable = view.currentDrawable else { return }

        let cameraPosition = float3(xy: .init(0), z: -cameraDistance).rotated(about: .xAxis, by: cameraOrbit.phi).rotated(about: .yAxis, by: cameraOrbit.theta)
        let cameraForward = -cameraPosition
        let cameraUp = float3.yAxis.rotated(about: .xAxis, by: cameraOrbit.phi)
        let cameraMatrix = float4x4.lookat(forward: cameraForward, up: cameraUp) * .translation(by: -cameraPosition)
        var projectionMatrix = float4x4.infinitePerspective(fovy: .pi, nearDistance: 1e-3, aspectRatio: .init(view.frame.width / view.frame.height)) * cameraMatrix

        guard let commandBuffer = commandQueue.makeCommandBuffer() else { return }
        guard let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) else { return }
        renderEncoder.setRenderPipelineState(plottingPipelineState)
        renderEncoder.setVertexBuffers([pointCloudBuffers.xPositions, pointCloudBuffers.yPositions, pointCloudBuffers.zPositions, pointCloudBuffers.intensities], offsets: [0, 0, 0, 0], range: 0 ..< 4)
        renderEncoder.setVertexBytes(&projectionMatrix, length: MemoryLayout<float4x4>.size, index: 4)
        renderEncoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: pointCloud.count)
        renderEncoder.endEncoding()
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }
}

