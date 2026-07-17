package frc.robot.lib.unit_test

import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.lib.extensions.get
import frc.robot.lib.universal_motor.LoggedMotorInputs
import frc.robot.lib.universal_motor.UniversalTalonFX

val allMotors = hashMapOf<String, UniversalTalonFX>()
val allMotorsFromPorts = hashMapOf<Int, UniversalTalonFX>()

fun getInputs(port: Int): LoggedMotorInputs = allMotorsFromPorts[port]!!.inputs

data class MotorInputChecks(
    val motorPort: Int,
    val positionTest: (Angle) -> Boolean = { true },
    val distanceTest: (Distance) -> Boolean = { true },
    val velocityTest: (AngularVelocity) -> Boolean = { true },
    val voltageTest: (Voltage) -> Boolean = { true },
    val currentTest: (Current) -> Boolean = { true },
    val statorCurrentTest: (Current) -> Boolean = { true }
)

data class CommandTestResult(val passed: Boolean, val failures: List<String>)

class CommandTest(
    val command: Command,
    vararg val motorInputChecks: MotorInputChecks,
    var generalTest: () -> Boolean = { true },
    val duration: Time
) {
    val requiredSubsystems: Set<Subsystem> = command.requirements
    val commandMotors = requiredSubsystems.mapNotNull { allMotors[it.name] }
    val comparedMotors =
        commandMotors
            .filter { motor ->
                motorInputChecks.any { expect ->
                    motor.port == expect.motorPort
                }
            }
            .groupBy { it.port }

    fun test(source: String): CommandTestResult {
        val runtime = SubsystemSimRuntime

        return try {
            runtime.scheduler.schedule(command)
            runtime.runFor(duration)

            println("Command test: $source")
            println("Command: ${command.name}")
            commandMotors.forEach(::printInputs)

            val failures = compareInputs()
            CommandTestResult(failures.isEmpty(), failures)
        } finally {
            command.cancel()
            runtime.reset()
        }
    }

    private fun compareInputs(): List<String> = buildList {
        if (!generalTest()) add("Didn't pass the generalTest !")

        motorInputChecks.forEach { expected ->
            val matchingMotors = comparedMotors[expected.motorPort].orEmpty()
            when {
                matchingMotors.isEmpty() ->
                    add(
                        "Motor ${expected.motorPort} was not found in the command requirements"
                    )
                matchingMotors.size > 1 ->
                    add(
                        "Motor port ${expected.motorPort} matched ${matchingMotors.size} motors"
                    )
                else -> compareMotor(expected, matchingMotors.single(), this)
            }
        }
    }

    private fun compareMotor(
        expected: MotorInputChecks,
        motor: UniversalTalonFX,
        failures: MutableList<String>
    ) {
        val actual = motor.inputs
        if (!expected.positionTest(actual.position))
            failures.add(
                "Motor ${motor.port} position check failed: ${actual.position}"
            )

        if (!expected.distanceTest(actual.distance))
            failures.add(
                "Motor ${motor.port} position check failed: ${actual.distance}"
            )

        if (!expected.velocityTest(actual.velocity))
            failures.add(
                "Motor ${motor.port} position check failed: ${actual.velocity}"
            )

        if (!expected.voltageTest(actual.voltage))
            failures.add(
                "Motor ${motor.port} position check failed: ${actual.voltage}"
            )

        if (!expected.currentTest(actual.current))
            failures.add(
                "Motor ${motor.port} position comparison failed: ${actual.current}"
            )

        if (!expected.statorCurrentTest(actual.statorCurrent))
            failures.add(
                "Motor ${motor.port} position comparison failed: ${actual.statorCurrent}"
            )
    }

    private fun printInputs(motor: UniversalTalonFX) {
        val inputs = motor.inputs
        println(
            "Motor ${motor.port}: " +
                "position=${inputs.position}, " +
                "distance=${inputs.distance}, " +
                "velocity=${inputs.velocity}, " +
                "voltage=${inputs.voltage}, " +
                "current=${inputs.current}, " +
                "statorCurrent=${inputs.statorCurrent}"
        )
    }
}
