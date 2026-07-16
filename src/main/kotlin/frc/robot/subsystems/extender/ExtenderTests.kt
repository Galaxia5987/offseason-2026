package frc.robot.subsystems.extender

import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.sec
import frc.robot.lib.unit_test.CommandTest
import frc.robot.lib.unit_test.ComparisonInputs
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.PORT
import org.team5987.annotation.unit_test.AddCommandTests

@AddCommandTests
val TargetTest =
    CommandTest(
        Extender.setPosition(10.m),
        ComparisonInputs(PORT, distanceTest = { it.isNear(10.m, 0.5.m) }),
        duration = 5.sec
    )
