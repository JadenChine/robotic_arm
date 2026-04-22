// swift-tools-version: 5.10
import PackageDescription

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
            path: "Sources/ArmControlMacApp"
        )
    ]
)
