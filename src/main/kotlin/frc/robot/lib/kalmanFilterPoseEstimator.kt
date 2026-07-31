package frc.robot.lib

import edu.wpi.first.math.*
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.numbers.N3
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.ms
import frc.robot.lib.extensions.sec
import frc.robot.subsystems.drive.Drive
import kotlin.math.*

val POSE_BUFFER_SIZE = 3.sec

const val X_STD_DEV_PER_SQRT_SECOND = 0.01
const val Y_STD_DEV_PER_SQRT_SECOND = 0.01
const val THETA_STD_DEV_PER_SQRT_SECOND = 0.01
const val TRANSLATION_STD_DEV_PER_SQRT_METER = 0.01
const val THETA_STD_DEV_PER_SQRT_RADIAN = 0.01
const val ACCELERATION_RESIDUAL_DEADBAND = 0.5
const val ACCELERATION_RESIDUAL_STD_DEV_GAIN = 0.01
const val DETECTED_SKID_STD_DEV_PER_SQRT_SECOND = 0.1

data class Estimate(
    var pose: Pose2d = getPose2d(),
    var covariance: Matrix<N3, N3> = Matrix(Nat.N3(), Nat.N3())
)

object PoseEstimator {
    // The estimate has no global field reference until the first trusted vision result arrives.
    private var initialized = false

    // This is the vision-corrected pose and its uncertainty P.
    private var estimate = Estimate()

    // This pose contains only wheel and gyro odometry. It is kept separate so vision corrections
    // do not contaminate the movement history used for latency compensation.
    private var odometryPose = Pose2d.kZero
    private var previousOdometryTimestamp: Double? = null

    // Historical raw odometry poses let us determine how the robot moved between a camera's
    // capture timestamp and the current time.
    private val odometryPoseBuffer: TimeInterpolatableBuffer<Pose2d> =
        TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE[sec])

    private var lastWheelPositions: Array<SwerveModulePosition> = arrayOf(
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition()
    )
    private val kinematics: SwerveDriveKinematics by lazy { SwerveDriveKinematics(*Drive.getModuleTranslations()) }

    // F is the state-transition Jacobian. It describes how uncertainty already present in
    // x, y, and theta changes when the robot applies this odometry movement.
    fun calculateF(poseBefore: Pose2d, twist: Twist2d): Matrix<N3, N3> {
        val theta = poseBefore.rotation.radians
        val dx = twist.dx
        val dy = twist.dy

        return MatBuilder.fill(
            Nat.N3(), Nat.N3(),
            1.0, 0.0, -sin(theta) * dx - cos(theta) * dy,
            0.0, 1.0, cos(theta) * dx - sin(theta) * dy,
            0.0, 0.0, 1.0
        )
    }

    // Q is process noise: new uncertainty introduced by odometry during this update. The terms
    // separately model time, distance, turning, IMU/wheel disagreement, and detected wheel skid.
    // All constants above are initial tuning values and must be validated with robot logs.
    fun calculateQ(
        dt: Double,
        twist: Twist2d,
        accelerationSlipResidual: Double,
        skidding: Boolean
    ): Matrix<N3, N3> {
        val distance = hypot(twist.dx, twist.dy)
        val turn = abs(twist.dtheta)
        val effectiveAccelerationResidual =
            max(0.0, accelerationSlipResidual - ACCELERATION_RESIDUAL_DEADBAND)

        // These terms are variances, so standard-deviation coefficients are squared before adding.
        val distanceVariance = TRANSLATION_STD_DEV_PER_SQRT_METER.pow(2) * distance
        val accelerationSlipVariance =
            (ACCELERATION_RESIDUAL_STD_DEV_GAIN * effectiveAccelerationResidual).pow(2) * dt
        val detectedSkidVariance =
            if (skidding) DETECTED_SKID_STD_DEV_PER_SQRT_SECOND.pow(2) * dt else 0.0

        val xVariance =
            X_STD_DEV_PER_SQRT_SECOND.pow(2) * dt +
                distanceVariance + accelerationSlipVariance + detectedSkidVariance
        val yVariance =
            Y_STD_DEV_PER_SQRT_SECOND.pow(2) * dt +
                distanceVariance + accelerationSlipVariance + detectedSkidVariance
        val thetaVariance =
            THETA_STD_DEV_PER_SQRT_SECOND.pow(2) * dt +
                THETA_STD_DEV_PER_SQRT_RADIAN.pow(2) * turn

        return MatBuilder.fill(
            Nat.N3(), Nat.N3(),
            xVariance, 0.0, 0.0,
            0.0, yVariance, 0.0,
            0.0, 0.0, thetaVariance
        )
    }

    // R is the uncertainty of one vision measurement. The vision subsystem supplies standard
    // deviations, while Kalman equations operate on variances, so every value is squared.
    private fun calculateR(observation: VisionObservation): Matrix<N3, N3> =
        MatBuilder.fill(
            Nat.N3(), Nat.N3(),
            observation.xStdDev * observation.xStdDev, 0.0, 0.0,
            0.0, observation.yStdDev * observation.yStdDev, 0.0,
            0.0, 0.0, observation.thetaStdDev * observation.thetaStdDev
        )

    fun updatePoseBaseOdometry(observation: OdometryObservation) {
        // Step 1: Calculate the real time since the previous odometry sample.
        val previousTimestamp = previousOdometryTimestamp
        previousOdometryTimestamp = observation.timestamp

        // Step 2: The first sample only establishes wheel and gyro baselines. There is no previous
        // wheel reading yet, so attempting to calculate movement would create a fake large jump.
        if (previousTimestamp == null) {
            lastWheelPositions = observation.wheelPositions

            if (observation.yaw != null) {
                odometryPose = Pose2d(odometryPose.translation, observation.yaw)
            }

            estimate.pose = odometryPose
            odometryPoseBuffer.addSample(observation.timestamp, odometryPose)
            return
        }

        val dt = observation.timestamp - previousTimestamp

        // Reject duplicate, reversed, or unreasonably delayed sensor samples.
        if (dt <= 0.0 || dt > 0.2) return

        // Step 3: Convert changes in all four wheel positions into robot-relative dx, dy, dtheta.
        val lastOdometryPose = odometryPose
        val twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions)
        lastWheelPositions = observation.wheelPositions

        // Step 4: Integrate wheel movement into the raw odometry pose.
        odometryPose = odometryPose.exp(twist)

        // The gyro is more trustworthy for heading than rotation calculated from wheel movement,
        // so replace the wheel-derived raw heading when gyro yaw is available.
        if (observation.yaw != null) {
            odometryPose = Pose2d(odometryPose.translation, observation.yaw)
        }

        // Save raw odometry, not the corrected estimate, for delayed camera measurements.
        odometryPoseBuffer.addSample(observation.timestamp, odometryPose)

        // Step 5: Calculate the final movement after the gyro heading replacement, then apply that
        // same movement to the vision-corrected estimate.
        val odometryTwist = lastOdometryPose.log(odometryPose)
        val poseBefore = estimate.pose
        estimate.pose = poseBefore.exp(odometryTwist)

        // Step 6: Before first vision, the pose is only relative and P will be initialized from R.
        // After initialization, propagate existing covariance through F and add new process noise Q.
        if (initialized) {
            val F = calculateF(poseBefore, odometryTwist)
            val Q = calculateQ(
                dt,
                odometryTwist,
                observation.accelerationSlipResidual,
                observation.skidding
            )
            estimate.covariance = F * estimate.covariance * F.transpose() + Q
        }
    }

    fun updatePoseBasedVision(observation: VisionObservation) {
        // Step 1: Reject vision older than the odometry history retained in the buffer.
        try {
            if (odometryPoseBuffer.internalBuffer.lastKey() - POSE_BUFFER_SIZE[sec] > observation.timestamp) return
        } catch (ex: NoSuchElementException) {
            return
        }

        // Step 2: Find the raw odometry pose closest to the image capture timestamp. Reject it if
        // no odometry sample is within 50 ms, because the temporal match would be unreliable.
        val sample = odometryPoseBuffer.internalBuffer.minByOrNull { (it.key - observation.timestamp).absoluteValue }
            ?.takeIf { (it.key - observation.timestamp).absoluteValue < 50.ms[sec] }
            ?.value ?: return

        // This is the recorded odometry movement from image capture time to the present.
        val sampleToOdometryTransform = Transform2d(sample, odometryPose)

        // Step 3: The first trusted camera result establishes the global field pose. Move the
        // camera pose forward using recorded odometry, and initialize P from vision covariance R.
        if (!initialized) {
            estimate.pose = observation.visionPose.toPose2d() + sampleToOdometryTransform
            estimate.covariance = calculateR(observation)
            initialized = true
            return
        }

        // Step 4: Move the current corrected estimate backward to the image capture timestamp.
        val estimateAtTime = estimate.pose + Transform2d(odometryPose, sample)
        val visionPose = observation.visionPose.toPose2d()

        // Step 5: Innovation = actual camera measurement - predicted camera measurement.
        // Heading uses angleModulus so crossing -pi/pi does not create a false 2pi error.
        val innovation = VecBuilder.fill(
            visionPose.x - estimateAtTime.x,
            visionPose.y - estimateAtTime.y,
            MathUtil.angleModulus(visionPose.rotation.radians - estimateAtTime.rotation.radians)
        )

        // Step 6: H maps the state [x, y, theta] into the values the sensor predicts it should see.
        // Vision directly measures all three state values, so h(x) = x and its Jacobian H is I.
        // A sensor that measured only x and y would instead use a 2x3 H that omitted theta.
        val P = estimate.covariance
        val R = calculateR(observation)
        val H = Matrix.eye(Nat.N3())

        // General gain: K = P H^T (H P H^T + R)^-1. Large P/small R trusts vision more;
        // small P/large R trusts the existing estimate more.
        val innovationCovariance = H * P * H.transpose() + R
        val K = P * H.transpose() * innovationCovariance.inv()
        val correction = K * innovation

        // Step 7: Apply the weighted correction at the historical camera timestamp.
        val correctedEstimateAtTime = Pose2d(
            estimateAtTime.x + correction[0, 0],
            estimateAtTime.y + correction[1, 0],
            Rotation2d.fromRadians(estimateAtTime.rotation.radians + correction[2, 0])
        )

        // Replay recorded odometry movement to bring the corrected historical pose back to now.
        estimate.pose = correctedEstimateAtTime + sampleToOdometryTransform

        // Step 8: Reduce covariance after receiving information from vision. The Joseph form is
        // more numerically stable than P = (I - KH)P.
        val identityMinusKH = Matrix.eye(Nat.N3()) - K * H
        estimate.covariance =
            identityMinusKH * P * identityMinusKH.transpose() + K * R * K.transpose()

        // Floating-point operations can make P microscopically asymmetric; force symmetry back.
        estimate.covariance =
            (estimate.covariance + estimate.covariance.transpose()) * 0.5
    }
}
data class OdometryObservation(
    val timestamp: Double,
    val wheelPositions: Array<SwerveModulePosition>,
    val roll: Rotation2d,
    val pitch: Rotation2d,
    val yaw: Rotation2d,
    val accelerationSlipResidual: Double,
    val skidding: Boolean
)

data class VisionObservation(
    val visionPose: Pose3d,
    val timestamp: Double,
    val xStdDev: Double,
    val yStdDev: Double,
    val thetaStdDev: Double
)
