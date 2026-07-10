package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command

val test1TargetPose= Pose2d(0.642, 5.934, Rotation2d.fromDegrees(180.0))


val test1Constraints= PathConstraints(3.0, 4.0, Units.degreesToRadians(540.0), Units.degreesToRadians(700.0))

fun pathFindingCommand(): Command= AutoBuilder.pathfindToPose(
    test1TargetPose,
    test1Constraints,
    0.0,
    )



