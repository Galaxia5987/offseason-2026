// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib;

import static frc.robot.subsystems.drive.Drive.getModuleTranslations;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.*;

// Taken from 6238 Mechanical Advantage
public class BetterPoseEstimator {
    // Vision measurements older than this cannot be corrected because their matching odometry
    // samples are no longer stored.
    private static final double poseBufferSizeSec = 2.0;

    // Our fixed estimate of how noisy wheel/gyro odometry is in x, y, and heading. Unlike a full
    // Kalman filter, this class does not update or save a covariance matrix over time.
    private static final Matrix<N3, N1> odometryStateStdDevs =
            new Matrix<>(VecBuilder.fill(0.3, 0.3, 0.01));

    // odometryPose contains only wheel and gyro integration. estimatedPose follows odometry but
    // also includes corrections from vision.
    private Pose2d odometryPose = Pose2d.kZero;
    private Pose2d estimatedPose = Pose2d.kZero;

    // Recent odometry is saved so a delayed camera result can be applied at the time when its
    // image was captured instead of incorrectly applying it at the current time.
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    // Roll, pitch, and yaw history is stored separately for users that need the robot's past 3D
    // orientation. It is not used in the 2D vision correction below.
    private final TimeInterpolatableBuffer<Rotation3d> rotationBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    // These are actually the fixed odometry variances (standard deviation squared). They are kept
    // as primitive values to avoid repeated matrix lookups when vision observations arrive.
    private final double qStdDevX;
    private final double qStdDevY;
    private final double qStdDevTheta;

    // Kinematics converts the distance traveled by each swerve wheel into one robot movement.
    private final SwerveDriveKinematics kinematics;

    // The previous wheel readings are required to calculate how far each wheel moved since the
    // last odometry observation.
    private SwerveModulePosition[] lastWheelPositions =
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };

    // This keeps a pose reset's heading aligned with future raw gyro measurements.
    private Rotation2d gyroOffset = Rotation2d.kZero;

    private ChassisSpeeds robotVelocity = new ChassisSpeeds();
    private ChassisSpeeds robotSetpointVelocity = new ChassisSpeeds();

    private static BetterPoseEstimator instance;

    public static BetterPoseEstimator getInstance() {
        if (instance == null) instance = new BetterPoseEstimator();
        return instance;
    }

    private BetterPoseEstimator() {
        // Variance = standard deviation squared. These values remain fixed for the lifetime of the
        // estimator; only the vision measurement variance changes for each observation.
        qStdDevX = Math.pow(odometryStateStdDevs.get(0, 0), 2);
        qStdDevY = Math.pow(odometryStateStdDevs.get(1, 0), 2);
        qStdDevTheta = Math.pow(odometryStateStdDevs.get(2, 0), 2);

        kinematics = new SwerveDriveKinematics(getModuleTranslations());
    }

    public ChassisSpeeds getRobotVelocity() {
        return robotVelocity;
    }

    public void setRobotVelocity(ChassisSpeeds robotVelocity) {
        this.robotVelocity = robotVelocity;
    }

    public ChassisSpeeds getRobotSetpointVelocity() {
        return robotSetpointVelocity;
    }

    public void setRobotSetpointVelocity(ChassisSpeeds robotSetpointVelocity) {
        this.robotSetpointVelocity = robotSetpointVelocity;
    }

    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    public void setOdometryPose(Pose2d odometryPose) {
        this.odometryPose = odometryPose;
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public void setEstimatedPose(Pose2d estimatedPose) {
        this.estimatedPose = estimatedPose;
    }

    public void resetPose(Pose2d pose) {
        // Adjust the raw gyro reference so the next gyro update continues from the requested pose
        // instead of snapping back to the old heading.
        gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
        estimatedPose = pose;
        odometryPose = pose;

        // Odometry from before the reset belongs to a different coordinate frame and must not be
        // used for future delayed vision measurements.
        poseBuffer.clear();
    }

    public Rotation2d getRotation() {
        return estimatedPose.getRotation();
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
    }

    public ChassisSpeeds getFieldSetpointVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotSetpointVelocity, getRotation());
    }

    public void addOdometryObservation(OdometryObservation observation) {
        // Step 1: Convert the change in all four wheel positions into a small robot-relative
        // movement (dx, dy, and dtheta).
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        Pose2d lastOdometryPose = odometryPose;

        // Step 2: Integrate that movement on the current 2D pose. Pose2d.exp handles translation
        // and rotation together instead of simply adding x, y, and theta.
        odometryPose = odometryPose.exp(twist);

        if (observation.yaw() != null) {
            // Wheel odometry estimates translation and rotation, but the gyro normally provides a
            // more reliable absolute heading, so replace the wheel-derived heading with gyro yaw.
            Rotation2d angle = observation.yaw().plus(gyroOffset);
            odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        }

        // Step 3: Save timestamped odometry for latency compensation when vision arrives later.
        poseBuffer.addSample(observation.timestamp(), odometryPose);

        if (observation.roll() != null
                && observation.pitch() != null
                && observation.yaw() != null) {
            // Save complete 3D orientation history when all three gyro angles are available.
            rotationBuffer.addSample(
                    observation.timestamp(),
                    new Rotation3d(
                            observation.roll().getRadians(),
                            observation.pitch().getRadians(),
                            observation.yaw().getRadians()));
        }

        // Step 4: Apply exactly the same odometry movement to the vision-corrected estimate. This
        // preserves previous vision corrections while the robot continues moving.
        Twist2d finalTwist = lastOdometryPose.log(odometryPose);
        estimatedPose = estimatedPose.exp(finalTwist);
    }

    public void addVisionObservation(VisionObservation observation) {
        // Step 1: Reject a measurement if it is older than the odometry history we still have.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec
                    > observation.timestamp()) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }

        // Step 2: Interpolate the odometry pose at the exact camera timestamp. This accounts for
        // camera processing and network latency.
        Optional<Pose2d> sampleOpt = poseBuffer.getSample(observation.timestamp());
        if (sampleOpt.isEmpty()) return;
        Pose2d sample = sampleOpt.get();

        // Work out how odometry moved between the camera timestamp and now, then temporarily move
        // the corrected estimate backward to the camera timestamp.
        Transform2d sampleToOdometryTransform = new Transform2d(sample, odometryPose);
        Transform2d odometryToSampleTransform = new Transform2d(odometryPose, sample);
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Step 3: Convert this camera observation's standard deviations into variances. A smaller
        // value means the camera is claiming that its result is more trustworthy.
        double r0 = observation.xStdDev() * observation.xStdDev();
        double r1 = observation.yStdDev() * observation.yStdDev();
        double r2 = observation.thetaStdDev() * observation.thetaStdDev();

        // Step 4: Calculate one steady-state Kalman gain for each axis. A gain near 1 accepts most
        // of the vision correction; a gain near 0 keeps most of the odometry estimate.
        double k0 = (qStdDevX == 0.0) ? 0.0 : qStdDevX / (qStdDevX + Math.sqrt(qStdDevX * r0));
        double k1 = (qStdDevY == 0.0) ? 0.0 : qStdDevY / (qStdDevY + Math.sqrt(qStdDevY * r1));
        double k2 =
                (qStdDevTheta == 0.0)
                        ? 0.0
                        : qStdDevTheta / (qStdDevTheta + Math.sqrt(qStdDevTheta * r2));

        // Step 5: Find the full difference between our estimate and the camera pose at the camera
        // timestamp.
        Transform2d transform =
                new Transform2d(estimateAtTime, observation.visionPose().toPose2d());

        // Accept only the Kalman-weighted fraction of that difference.
        Transform2d scaledTransform =
                new Transform2d(
                        transform.getX() * k0,
                        transform.getY() * k1,
                        Rotation2d.fromRadians(transform.getRotation().getRadians() * k2));

        // Step 6: Correct the historical estimate, then replay the saved odometry movement from
        // the camera timestamp to now. The final result is once again a current-time pose.
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    public Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestamp) {
        // Reconstruct an older corrected pose by applying the old-to-current odometry difference
        // backward from the current corrected estimate.
        Optional<Pose2d> oldOdometryPose = poseBuffer.getSample(timestamp);
        if (oldOdometryPose.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(
                estimatedPose.transformBy(new Transform2d(odometryPose, oldOdometryPose.get())));
    }

    public Optional<Rotation3d> getEstimatedRotation3dAtTimestamp(double timestamp) {
        return rotationBuffer.getSample(timestamp);
    }

    public record OdometryObservation(
            double timestamp,
            SwerveModulePosition[] wheelPositions,
            Rotation2d roll,
            Rotation2d pitch,
            Rotation2d yaw) {}

    public record VisionObservation(
            Pose3d visionPose,
            double timestamp,
            double xStdDev,
            double yStdDev,
            double thetaStdDev) {}
}
