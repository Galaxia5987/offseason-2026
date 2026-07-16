package frc.robot.subsystems

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Roller : SubsystemBase() {
    private val motor =
        UniversalTalonFX(port = PORT, config = CONFIG, simGains = SIM_GAINS)

    private val velocityVoltageRequest = VelocityVoltage(0.rps)
    private var setpoint: AngularVelocity = 0.rps

    fun setTarget(value: AngularVelocity) {
        setpoint = value
        motor.setControl(velocityVoltageRequest.withVelocity(setpoint))
    }

    fun intake() : Command = runOnce{setTarget(Modes.INTAKE.value)}
    fun outtake() : Command = runOnce{setTarget(Modes.OUTTAKE.value)}
    fun stop() : Command = runOnce{setTarget(Modes.STOP.value)}


    override fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/Roller/setpoint", setpoint)
    }
}
