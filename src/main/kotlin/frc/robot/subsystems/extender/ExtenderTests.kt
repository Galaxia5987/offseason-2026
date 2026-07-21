package frc.robot.subsystems.extender

import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.sec
import frc.robot.lib.unit_test.SubsystemSimRuntime
import frc.robot.lib.unit_test.Tests
import frc.robot.lib.unit_test.getInputs
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.PORT
import kotlin.test.Test
import kotlin.test.assertTrue

class ExtenderTests : Tests() {
    @Test
    fun `motor test`() {
        SubsystemSimRuntime.apply {
            addCommands(Extender.setPosition(10.m))
            runFor(10.sec)
        }
        assertTrue(
            getInputs(PORT).distance.isNear(10.m, 0.2.m),
            "actual distance: ${getInputs(PORT).distance}"
        )
    }

    @Test
    fun `motor test 1`() {
        SubsystemSimRuntime.apply { runFor(10.sec) }
        assertTrue(
            getInputs(PORT).distance.isNear(0.m, 0.2.m),
            "actual distance: ${getInputs(PORT).distance}"
        )
    }
}
