package frc.robot.lib.unit_test

import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj.simulation.SimHooks
import edu.wpi.first.wpilibj2.command.CommandScheduler

object SubsystemSimRuntime {
    private const val periodSeconds: Double = 0.02
    private val scheduler = CommandScheduler.getInstance()

    init {
        check(HAL.initialize(500, 0))
        SimHooks.pauseTiming()

        DriverStationSim.setEnabled(true)
        DriverStationSim.notifyNewData()
    }

    fun step() {
        SimHooks.stepTiming(periodSeconds)
        scheduler.run()
    }

    fun runFor(seconds: Double) {
        repeat((seconds / periodSeconds).toInt()) { step() }
        DriverStationSim.notifyNewData()
    }

    fun runUntil(timeoutSeconds: Double, condition: () -> Boolean): Boolean {
        repeat((timeoutSeconds / periodSeconds).toInt()) {
            step()
            if (condition()) {
                DriverStationSim.notifyNewData()
                return true
            }
        }
        DriverStationSim.notifyNewData()
        return false
    }

    fun reset() {
        scheduler.cancelAll()
        scheduler.clearComposedCommands()
    }
}
