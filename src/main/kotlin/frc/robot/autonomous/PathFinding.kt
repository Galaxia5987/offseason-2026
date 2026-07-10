package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.lib.extensions.flipIfNeeded


val test1TargetPoseBlue= Pose2d(4.500, 7.512, Rotation2d.fromDegrees(180.0))
val test1TargetPoseRed= Pose2d(11.950,0.759, Rotation2d.fromDegrees(180.0))
val test1Constraints= PathConstraints(3.0, 4.0, Units.degreesToRadians(540.0), Units.degreesToRadians(700.0))


fun pathFindingCommandRed(): Command= AutoBuilder.pathfindToPose(
    test1TargetPoseRed,
    test1Constraints,
    0.0
    )
fun pathFindingCommandBlue(): Command= AutoBuilder.pathfindToPose(
    test1TargetPoseBlue,
    test1Constraints,
    0.0
)

//fun pathFindingThenFollowPathCommand(): Command= AutoBuilder.pathfindThenFollowPath()

