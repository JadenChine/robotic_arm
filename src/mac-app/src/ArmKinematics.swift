import Foundation

enum ArmKinematics {
    private static let baseZMM = 65.0
    private static let link1MM = 101.0
    private static let link2MM = 95.0
    private static let link3MM = 168.0
    private static let totalReachMM = link1MM + link2MM + link3MM
    private static let groundZMM = 0.0
    private static let yawLimit = (-120.0...120.0)
    private static let shoulderLimit = (-90.0...90.0)
    private static let elbowLimit = (-120.0...120.0)
    private static let maxIterations = 36
    private static let toleranceMM = 3.0
    private static let damping = 18.0
    private static let maxStepRadians = 0.080
    private static let determinantEpsilon = 0.000001

    static func tipPositionMM(fromLogicalJointAngles jointAngles: [Double]) -> [Double] {
        guard jointAngles.count == 6 else { return [0.0, 0.0, baseZMM + totalReachMM] }

        let yawRadians = degreesToRadians(jointAngles[5])
        let shoulderRadians = degreesToRadians(jointAngles[4])
        let elbow1Radians = degreesToRadians(jointAngles[3])
        let elbow2Radians = degreesToRadians(jointAngles[2])
        let planar = pitchForward(shoulderRadians: shoulderRadians,
                                   elbow1Radians: elbow1Radians,
                                   elbow2Radians: elbow2Radians)

        return [
            normalizeSmall(planar.radialMM * cos(yawRadians)),
            normalizeSmall(planar.radialMM * sin(yawRadians)),
            normalizeSmall(baseZMM + planar.zRelativeMM),
        ]
    }

    static func logicalJointAngles(forCartesianTargetMM targetMM: [Double], seedJointAngles: [Double]) -> [Double] {
        guard targetMM.count == 3, seedJointAngles.count == 6 else { return seedJointAngles }

        let target = clampCartesianTarget(targetMM)
        let yawRadians = degreesToRadians(clamp(radiansToDegrees(atan2(target[1], target[0])), in: yawLimit))
        let targetRadialMM = hypot(target[0], target[1])
        let targetZRelativeMM = target[2] - baseZMM
        var pitchAnglesRadians = [
            degreesToRadians(seedJointAngles[4]),
            degreesToRadians(seedJointAngles[3]),
            degreesToRadians(seedJointAngles[2]),
        ]
        clampPitchAngles(&pitchAnglesRadians)

        if abs(pitchAnglesRadians[0]) < 0.001,
           abs(pitchAnglesRadians[1]) < 0.001,
           abs(pitchAnglesRadians[2]) < 0.001,
           targetZRelativeMM < (totalReachMM - toleranceMM) {
            pitchAnglesRadians = closedFormPitchSeed(targetRadialMM: targetRadialMM, targetZRelativeMM: targetZRelativeMM)
        }

        for _ in 0..<maxIterations {
            let current = pitchForward(shoulderRadians: pitchAnglesRadians[0],
                                       elbow1Radians: pitchAnglesRadians[1],
                                       elbow2Radians: pitchAnglesRadians[2])
            let radialErrorMM = targetRadialMM - current.radialMM
            let zErrorMM = targetZRelativeMM - current.zRelativeMM
            let errorMM = hypot(radialErrorMM, zErrorMM)
            if errorMM <= toleranceMM {
                break
            }

            let jacobian = pitchJacobian(shoulderRadians: pitchAnglesRadians[0],
                                         elbow1Radians: pitchAnglesRadians[1],
                                         elbow2Radians: pitchAnglesRadians[2])
            let dampingSquared = damping * damping
            let matrix00 = jacobian[0][0] * jacobian[0][0] + jacobian[0][1] * jacobian[0][1] + jacobian[0][2] * jacobian[0][2] + dampingSquared
            let matrix01 = jacobian[0][0] * jacobian[1][0] + jacobian[0][1] * jacobian[1][1] + jacobian[0][2] * jacobian[1][2]
            let matrix11 = jacobian[1][0] * jacobian[1][0] + jacobian[1][1] * jacobian[1][1] + jacobian[1][2] * jacobian[1][2] + dampingSquared
            let determinant = (matrix00 * matrix11) - (matrix01 * matrix01)
            if abs(determinant) < determinantEpsilon {
                break
            }

            let inverse00 = matrix11 / determinant
            let inverse01 = -matrix01 / determinant
            let inverse11 = matrix00 / determinant
            let resolvedError0 = (inverse00 * radialErrorMM) + (inverse01 * zErrorMM)
            let resolvedError1 = (inverse01 * radialErrorMM) + (inverse11 * zErrorMM)

            for jointIndex in 0..<3 {
                var deltaRadians = (jacobian[0][jointIndex] * resolvedError0) + (jacobian[1][jointIndex] * resolvedError1)
                deltaRadians = clamp(deltaRadians, in: -maxStepRadians...maxStepRadians)
                pitchAnglesRadians[jointIndex] += deltaRadians
            }

            clampPitchAngles(&pitchAnglesRadians)
        }

        var updatedAngles = seedJointAngles
        updatedAngles[5] = radiansToDegrees(yawRadians)
        updatedAngles[4] = radiansToDegrees(pitchAnglesRadians[0])
        updatedAngles[3] = radiansToDegrees(pitchAnglesRadians[1])
        updatedAngles[2] = radiansToDegrees(pitchAnglesRadians[2])
        return updatedAngles
    }

    private static func pitchForward(shoulderRadians: Double,
                                     elbow1Radians: Double,
                                     elbow2Radians: Double) -> (radialMM: Double, zRelativeMM: Double) {
        let pitch1 = (Double.pi * 0.5) - shoulderRadians
        let pitch2 = pitch1 - elbow1Radians
        let pitch3 = pitch2 - elbow2Radians
        let radialMM = link1MM * cos(pitch1) + link2MM * cos(pitch2) + link3MM * cos(pitch3)
        let zRelativeMM = link1MM * sin(pitch1) + link2MM * sin(pitch2) + link3MM * sin(pitch3)
        return (radialMM, zRelativeMM)
    }

    private static func pitchJacobian(shoulderRadians: Double,
                                      elbow1Radians: Double,
                                      elbow2Radians: Double) -> [[Double]] {
        let pitch1 = (Double.pi * 0.5) - shoulderRadians
        let pitch2 = pitch1 - elbow1Radians
        let pitch3 = pitch2 - elbow2Radians

        return [
            [
                link1MM * sin(pitch1) + link2MM * sin(pitch2) + link3MM * sin(pitch3),
                link2MM * sin(pitch2) + link3MM * sin(pitch3),
                link3MM * sin(pitch3),
            ],
            [
                -link1MM * cos(pitch1) - link2MM * cos(pitch2) - link3MM * cos(pitch3),
                -link2MM * cos(pitch2) - link3MM * cos(pitch3),
                -link3MM * cos(pitch3),
            ],
        ]
    }

    private static func clampPitchAngles(_ pitchAnglesRadians: inout [Double]) {
        pitchAnglesRadians[0] = degreesToRadians(clamp(radiansToDegrees(pitchAnglesRadians[0]), in: shoulderLimit))
        pitchAnglesRadians[1] = degreesToRadians(clamp(radiansToDegrees(pitchAnglesRadians[1]), in: elbowLimit))
        pitchAnglesRadians[2] = degreesToRadians(clamp(radiansToDegrees(pitchAnglesRadians[2]), in: elbowLimit))
    }

    private static func closedFormPitchSeed(targetRadialMM: Double, targetZRelativeMM: Double) -> [Double] {
        let combinedForearmMM = link2MM + link3MM
        let distanceSquared = (targetRadialMM * targetRadialMM) + (targetZRelativeMM * targetZRelativeMM)
        let cosineElbow = clamp(
            (distanceSquared - (link1MM * link1MM) - (combinedForearmMM * combinedForearmMM)) /
                (2.0 * link1MM * combinedForearmMM),
            in: -1.0...1.0
        )
        let elbowRadians = acos(cosineElbow)
        let targetAngleRadians = atan2(targetRadialMM, targetZRelativeMM)
        let shoulderOffsetRadians = atan2(
            combinedForearmMM * sin(elbowRadians),
            link1MM + combinedForearmMM * cos(elbowRadians)
        )
        var pitchAnglesRadians = [targetAngleRadians - shoulderOffsetRadians, elbowRadians, 0.0]
        clampPitchAngles(&pitchAnglesRadians)
        return pitchAnglesRadians
    }

    private static func clampCartesianTarget(_ targetMM: [Double]) -> [Double] {
        var xMM = targetMM[0]
        var yMM = targetMM[1]
        var zMM = max(groundZMM, targetMM[2])
        let radialMM = hypot(xMM, yMM)
        var zRelativeMM = zMM - baseZMM
        let shoulderDistanceMM = hypot(radialMM, zRelativeMM)

        if shoulderDistanceMM > totalReachMM, shoulderDistanceMM > 0.0 {
            let scale = totalReachMM / shoulderDistanceMM
            xMM *= scale
            yMM *= scale
            zRelativeMM *= scale
            zMM = baseZMM + zRelativeMM
        }

        return [normalizeSmall(xMM), normalizeSmall(yMM), normalizeSmall(zMM)]
    }

    private static func degreesToRadians(_ degrees: Double) -> Double {
        degrees * Double.pi / 180.0
    }

    private static func radiansToDegrees(_ radians: Double) -> Double {
        radians * 180.0 / Double.pi
    }

    private static func clamp(_ value: Double, in range: ClosedRange<Double>) -> Double {
        min(range.upperBound, max(range.lowerBound, value))
    }

    private static func normalizeSmall(_ value: Double) -> Double {
        abs(value) < 0.000001 ? 0.0 : value
    }
}
