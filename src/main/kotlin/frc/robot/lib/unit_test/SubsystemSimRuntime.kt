package frc.robot.lib.unit_test

import edu.wpi.first.hal.HAL
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj.simulation.SimHooks
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec
import frc.robot.lib.universal_motor.LoggedMotorInputs
import frc.robot.lib.universal_motor.UniversalTalonFX
import kotlin.test.AfterTest
import kotlin.test.assertTrue
import org.junit.jupiter.api.AfterAll
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.TestInstance

private const val PERIODIC_TIME: Double = 0.02

val allMotorsFromPorts =
    hashMapOf<Pair<Int, String>, MutableList<UniversalTalonFX>>()

fun getInputs(port: Int, rio: String = "rio"): LoggedMotorInputs =
    allMotorsFromPorts[port to rio]!!.first().inputs

object SubsystemSimRuntime {
    val scheduler: CommandScheduler = CommandScheduler.getInstance()

    fun addCommands(vararg commands: Command): SubsystemSimRuntime {
        scheduler.schedule(*commands)
        return this
    }

    init {
        check(HAL.initialize(500, 0))
        SimHooks.pauseTiming()

        DriverStationSim.setEnabled(true)
        DriverStationSim.notifyNewData()
    }

    fun step() {
        SimHooks.stepTiming(PERIODIC_TIME)
        scheduler.run()
    }

    fun runFor(seconds: Time) {
        repeat((seconds[sec] / PERIODIC_TIME).toInt()) { step() }
        DriverStationSim.notifyNewData()
        reset()
    }

    fun runUntil(timeoutSeconds: Time, condition: () -> Boolean): Boolean {
        repeat((timeoutSeconds[sec] / PERIODIC_TIME).toInt()) {
            step()
            if (condition()) {
                DriverStationSim.notifyNewData()
                reset()
                return true
            }
        }
        DriverStationSim.notifyNewData()
        reset()
        return false
    }

    fun reset() {
        scheduler.cancelAll()
        scheduler.clearComposedCommands()
    }

    fun restartInputs() {
        reset()
        resetRegisteredSimulationState()
        allMotorsFromPorts.values.flatten().distinct().forEach {
            it.resetInputs()
        }
    }
}

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
abstract class Tests {

    @AfterAll
    @DisplayName("check duplicate motors")
    fun `check duplicate motors`() {
        val duplicates = allMotorsFromPorts.filterValues { it.size > 1 }
        assertTrue(duplicates.isEmpty(), "Duplicate motor CAN IDs")
    }

    @AfterTest
    fun `clean inputs`() {
        SubsystemSimRuntime.restartInputs()
    }
}
