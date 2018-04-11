import Cocoa

@NSApplicationMain
public class ApplicationDelegate: NSObject, NSApplicationDelegate {
    var pointCloud: PointCloud! = nil
    var secondPointCloud: PointCloud! = nil
    
    public func applicationWillFinishLaunching(_ notification: Notification) {
        pointCloud = try! PointCloud(contentsOf: Bundle.main.url(forResource: "pointcloud1", withExtension: "fuse")!)
        secondPointCloud = try! PointCloud(contentsOf: Bundle.main.url(forResource: "pointcloud2", withExtension: "fuse")!)
    }
    
    public func applicationDidFinishLaunching(_ notification: Notification) {
        guard let viewController = NSApp.mainWindow?.contentViewController as? ViewController else { return }
        viewController.pointClouds = [pointCloud, secondPointCloud]
    }
    
    public func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool {
        return true
    }
}

