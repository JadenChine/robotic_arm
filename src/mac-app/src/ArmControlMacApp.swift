import SwiftUI

@main
struct ArmControlMacApp: App {
    @StateObject private var bleManager = BLEManager()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(bleManager)
                .frame(minWidth: 760, minHeight: 560)
        }
    }
}
