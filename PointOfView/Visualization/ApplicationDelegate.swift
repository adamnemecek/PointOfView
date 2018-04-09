import Cocoa

@NSApplicationMain
public class ApplicationDelegate: NSObject, NSApplicationDelegate {
    public func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool {
        return true
    }
}

