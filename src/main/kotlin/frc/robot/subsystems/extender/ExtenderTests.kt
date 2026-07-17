package frc.robot.subsystems.extender

import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.sec
import frc.robot.lib.unit_test.SubsystemSimRuntime
import frc.robot.lib.unit_test.Tests
import frc.robot.lib.unit_test.getInputs
import frc.robot.subsystems.intake.extender.Extender.setPosition
import frc.robot.subsystems.intake.extender.PORT
import kotlin.test.Test
import kotlin.test.assertTrue

// @AddCommandTests
// val TargetTest =
//    CommandTest(
//        Extender.setPosition(10.m),
//        MotorInputChecks(PORT, distanceTest = { it.isNear(9.m, 0.5.m) }),
//        generalTest = { Extender.inputs.distance.isNear(9.m, 0.3.m) },
//        duration = 5.sec
//    )
//
// @AddCommandTests
// val TargetTest1 =
//    CommandTest(
//        Extender.setPosition(10.m),
//        MotorInputChecks(PORT, distanceTest = { it.isNear(10.m, 0.5.m) }),
//        duration = 5.sec
//    )

class ExtenderTests : Tests() {
    @Test
    fun `motor test`(): Unit {
        SubsystemSimRuntime.apply {
            addCommands(setPosition(10.m))
            runFor(10.sec)
        }
        assertTrue(
            getInputs(PORT).distance.isNear(10.m, 0.2.m),
            "actual distance: ${getInputs(PORT).distance}"
        )
    }

    @Test
    fun `motor test 1`(): Unit {
        SubsystemSimRuntime.apply { runFor(10.sec) }
        assertTrue(
            getInputs(PORT).distance.isNear(0.m, 0.2.m),
            "actual distance: ${getInputs(PORT).distance}"
        )
    }
}
