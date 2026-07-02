package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.volts

const val PORT = 2

val REAL_GAINS = Gains(kP = 0.5, kD = 0.1)

val SIM_GAINS = Gains(kP = 0.5, kD = 0.1)

const val GEAR_RATIO = 1.0

val INTAKE_VOLTAGE = 10.volts
val OUTTAKE_VOLTAGE = -INTAKE_VOLTAGE

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Coast
            }
        Slot0 = REAL_GAINS.toSlotConfig()

        CurrentLimits = createCurrentLimits(supplyCurrentLimit = 10.amps)

        Feedback =
            FeedbackConfigs().apply { SensorToMechanismRatio = GEAR_RATIO }
    }
