package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.extensions.volts

val PORT = 2

val REAL_GAINS = Gains(kP = 0.5, kD = 0.1)

val SIM_GAINS = Gains(kP = 0.5, kD = 0.1)

val GEAR_RATIO = 1.0

val INTAKE = 10.volts
val OUTTAKE = -INTAKE
val STOP = 0.volts

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Coast
            }
        Slot0 = REAL_GAINS.toSlotConfig()
        Feedback =
            FeedbackConfigs().apply { SensorToMechanismRatio = GEAR_RATIO }
    }
