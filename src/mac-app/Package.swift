// swift-tools-version: 5.10
import PackageDescription
import Foundation

let packageRoot = URL(fileURLWithPath: #filePath).deletingLastPathComponent().path
let appInfoPlistPath = "\(packageRoot)/src/ArmControlMacApp-Info.plist"

let package = Package(
    name: "ArmControlMacApp",
    platforms: [
        .macOS(.v13)
    ],
    products: [
        .executable(
            name: "ArmControlMacApp",
            targets: ["ArmControlMacApp"]
        )
    ],
    targets: [
        .executableTarget(
            name: "ArmControlMacApp",
            path: "src",
            exclude: ["ArmControlMacApp-Info.plist"],
            linkerSettings: [
                .unsafeFlags([
                    "-Xlinker", "-sectcreate",
                    "-Xlinker", "__TEXT",
                    "-Xlinker", "__info_plist",
                    "-Xlinker", appInfoPlistPath
                ])
            ]
        )
    ]
)
