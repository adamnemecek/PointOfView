import MetalKit

public class ViewController: NSViewController, MTKViewDelegate {
    public private(set) var device: MTLDevice! = nil
    private var commandQueue: MTLCommandQueue! = nil
    private var plottingPipelineState: MTLRenderPipelineState! = nil
    
    public var pointCloud: PointCloud? = nil { willSet {
        pointCloudBuffers = nil
        guard let pointCloud = newValue else { return }
        guard let xPositions = device.makeBuffer(bytesNoCopy: pointCloud.xPositions, length: (pointCloud.count * MemoryLayout<Float>.stride).aligned(to: 4096), options: .storageModeShared, deallocator: nil) else { return }
        guard let yPositions = device.makeBuffer(bytesNoCopy: pointCloud.yPositions, length: (pointCloud.count * MemoryLayout<Float>.stride).aligned(to: 4096), options: .storageModeShared, deallocator: nil) else { return }
        guard let zPositions = device.makeBuffer(bytesNoCopy: pointCloud.zPositions, length: (pointCloud.count * MemoryLayout<Float>.stride).aligned(to: 4096), options: .storageModeShared, deallocator: nil) else { return }
        guard let intensities = device.makeBuffer(bytesNoCopy: pointCloud.intensities, length: (pointCloud.count * MemoryLayout<UInt8>.stride).aligned(to: 4096), options: .storageModeShared, deallocator: nil) else { return }
        pointCloudBuffers = (xPositions, yPositions, zPositions, intensities)
    }}
    private var pointCloudBuffers: (xPositions: MTLBuffer, yPositions: MTLBuffer, zPositions: MTLBuffer, intensities: MTLBuffer)? = nil
    
    private var cameraShift: float3 = .init(0)
    private var cameraOrbit: (theta: Float, phi: Float) = (0, 0)
    private var cameraDistance: Float = 1.5
    
    private var orbitMatrix: float4x4 {
        return .rotation(about: .xAxis, by: -cameraOrbit.phi) * .rotation(about: .yAxis, by: -cameraOrbit.theta)
    }
    
    private var cameraMatrix: float4x4 {
        return .translation(by: .init(xy: .init(0), z: cameraDistance)) * orbitMatrix * .translation(by: cameraShift)
    }
    
    public override func viewDidLoad() {
        guard let view = self.view as? MTKView else {
            fatalError("The ViewController must be instantiated with an MTKView")
        }
        guard let device = MTLCopyAllDevices().first(where: { $0.isLowPower }) ?? MTLCreateSystemDefaultDevice() else {
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
    
    public override var acceptsFirstResponder: Bool {
        return true
    }
    
    public override func scrollWheel(with event: NSEvent) {
        if event.modifierFlags.contains(.shift) {
            let rawShift = float3(x: .init(event.scrollingDeltaX / view.frame.width), y: -.init(event.scrollingDeltaY / view.frame.height), z: 0)
            cameraShift += float3.xAxis.transformed(by: orbitMatrix.inverse) * rawShift.x + float3.yAxis.transformed(by: orbitMatrix.inverse) * rawShift.y
        }
        else {
            cameraOrbit.theta += .init(event.scrollingDeltaX / view.frame.width)
            cameraOrbit.phi += .init(event.scrollingDeltaY / view.frame.height)
        }
    }
    
    public override func magnify(with event: NSEvent) {
        cameraDistance -= .init(event.magnification) * cameraDistance
    }
    
    public func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        return
    }
    
    public func draw(in view: MTKView) {
        guard let pointCloud = pointCloud else { return }
        guard let pointCloudBuffers = pointCloudBuffers else { return }
        guard let renderPassDescriptor = view.currentRenderPassDescriptor else { return }
        guard let drawable = view.currentDrawable else { return }
        
        var projectionMatrix = float4x4.infinitePerspective(fovy: .pi / 2, nearDistance: 1e-3, aspectRatio: .init(view.frame.width / view.frame.height)) * cameraMatrix

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

