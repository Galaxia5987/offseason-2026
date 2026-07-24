package frc.robot.lib

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N3

class PoseEstimator() {

    val stateVector: Pose2d = getPose2d()
    val uncertainty: Matrix<N3, N3> = Matrix(Nat.N3(), Nat.N3())
    fun addObservation() {

    }

}