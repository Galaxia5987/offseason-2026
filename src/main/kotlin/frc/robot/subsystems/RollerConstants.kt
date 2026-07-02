package frc.robot.subsystems

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.lib.Gains
import frc.robot.lib.extensions.rps

val PORT = 0
val GEAR_RATIO = 0.0
val SIM_GAINS = Gains(kP = 0.0, kD = 0.0)
val REAL_GAINS = Gains(kP = 0.0, kD = 0.0)

enum class Modes(val value : AngularVelocity) {
    INTAKE(10.rps),
    OUTTAKE((-10).rps),
    STOP(0.rps)
}

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
        Slot0 = REAL_GAINS.toSlotConfig()
        Feedback =
            FeedbackConfigs().apply { SensorToMechanismRatio = GEAR_RATIO }
    }
