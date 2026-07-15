package frc.robot.lib.unit_test

import org.team5987.annotation.unit_test.AddTests

fun add(n1: Float, n2: Float, n3: Float, n4: Float): Float {
    return n1 + n2 + n3 + n4
}

@AddTests
val unitTest =
    TestCase(comparison = { it == 10f }, ::add, 1.0f, 2.0f, 3.0f, 4.0f)

@AddTests
val additionCases =
    listOf(
        TestCase(comparison = { it == 4f }, ::add, 2f, 1f, 1f, 1f),
        TestCase(comparison = { it == 8f }, ::add, 2f, 2f, 2f, 2f),
    )