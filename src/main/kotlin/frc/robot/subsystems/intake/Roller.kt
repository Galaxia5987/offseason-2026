package frc.robot.subsystems.intake

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.Gains
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.UniversalTalonFX
import javax.xml.stream.events.Comment

object Roller : SubsystemBase() {
    private val motor = UniversalTalonFX(
        config = MOTOR_CONFIG,
        port = PORT,
        simGains = SIM_GAINS
    )
    private val setpoint: AngularVelocity = 0.rps
    private val voltageRequest = VoltageOut(0.0)

    val isActive = Trigger {setpoint > 0.rps}

 fun setVoltage (voltage : Voltage) : Command = this.runOnce {
     motor.setControl(voltageRequest.withOutput(voltage))
    }
    fun intake() : Command = setVoltage(INTAKE)
    fun outTake() : Command = setVoltage(OUTTAKE)
    fun stop() : Command = setVoltage(STOP)

    override fun periodic() {
        motor.periodic()
    }
}