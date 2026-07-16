package frc.robot.lib.unit_test

import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec
import frc.robot.lib.universal_motor.UniversalTalonFX

val allMotors = hashMapOf<String, UniversalTalonFX>()
val commandScheduler = CommandScheduler.getInstance()

data class ComparisonInputs(
    val motorPort: Int,
    val positionTest: ((Angle) -> Boolean)? = null,
    val distanceTest: ((Distance) -> Boolean)? = null,
    val velocityTest: ((AngularVelocity) -> Boolean)? = null,
    val voltageTest: ((Voltage) -> Boolean)? = null,
    val currentTest: ((Current) -> Boolean)? = null,
    val statorCurrentTest: ((Current) -> Boolean)? = null
)

data class CommandTestResult(val passed: Boolean, val failures: List<String>)

class CommandTest(
    val command: Command,
    vararg val compareList: ComparisonInputs,
    val duration: Time
) {
    val requiredSubsystems: Set<Subsystem> = command.requirements
    val commandMotors = requiredSubsystems.mapNotNull { allMotors[it.name] }
    val comparedMotors =
        commandMotors
            .filter { motor ->
                compareList.any { expect -> motor.port == expect.motorPort }
            }
            .groupBy { it.port }

    fun test(source: String): CommandTestResult {
        val runtime = SubsystemSimRuntime()

        return try {
            commandScheduler.schedule(command)
            runtime.runFor(duration[sec])

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
        compareList.forEach { expected ->
            val matchingMotors = comparedMotors[expected.motorPort].orEmpty()

            when {
                matchingMotors.isEmpty() ->
                    add(
                        "Motor ${expected.motorPort} was not found in " +
                            "the command requirements"
                    )
                matchingMotors.size > 1 ->
                    add(
                        "Motor port ${expected.motorPort} matched " +
                            "${matchingMotors.size} motors"
                    )
                else -> compareMotor(expected, matchingMotors.single(), this)
            }
        }
    }

    private fun compareMotor(
        expected: ComparisonInputs,
        motor: UniversalTalonFX,
        failures: MutableList<String>
    ) {
        val actual = motor.inputs
        compare(
            motor.port,
            "position",
            actual.position,
            expected.positionTest,
            failures
        )
        compare(
            motor.port,
            "distance",
            actual.distance,
            expected.distanceTest,
            failures
        )
        compare(
            motor.port,
            "velocity",
            actual.velocity,
            expected.velocityTest,
            failures
        )
        compare(
            motor.port,
            "voltage",
            actual.voltage,
            expected.voltageTest,
            failures
        )
        compare(
            motor.port,
            "current",
            actual.current,
            expected.currentTest,
            failures
        )
        compare(
            motor.port,
            "stator current",
            actual.statorCurrent,
            expected.statorCurrentTest,
            failures
        )
    }

    private fun <T> compare(
        port: Int,
        inputName: String,
        actual: T,
        comparison: ((T) -> Boolean)?,
        failures: MutableList<String>
    ) {
        if (comparison != null && !comparison(actual)) {
            failures.add("Motor $port $inputName comparison failed: $actual")
        }
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
