import Cocoa

@NSApplicationMain
public class ApplicationDelegate: NSObject, NSApplicationDelegate {
    var pointCloud: PointCloud! = nil
    
    public func applicationWillFinishLaunching(_ notification: Notification) {
        pointCloud = try! PointCloud(contentsOf: Bundle.main.url(forResource: "pointcloud1", withExtension: "fuse")!)
    }
    
    public func applicationDidFinishLaunching(_ notification: Notification) {
        guard let viewController = NSApp.mainWindow?.contentViewController as? ViewController else { return }
        viewController.pointCloud = pointCloud
    }
    
    public func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool {
        return true
    }
}

