package frc.robot.lib.unit_test

import kotlin.reflect.KFunction
import kotlin.reflect.full.valueParameters
import org.team5987.annotation.unit_test.AddTests

data class TestResult(
    val name: String,
    val passed: Boolean,
    val inputs: List<Any?>,
    val actual: Any?,
    val functionName: String
)

class UnitTest<O>(
    val comparison: (O) -> Boolean,
    val function: KFunction<O>,
    vararg val inputs: Any?,
    val name: String = "",
) {
    fun test(): TestResult {
        val actual =
            try {
                function.call(*inputs)
            } catch (e: IllegalArgumentException) {
                error(
                    "function input are : (${
                    function.valueParameters.joinToString {
                        "${it.name}: ${
                            it.type.toString().substringAfterLast(".")
                        }"
                    }
                }) wrong inputs"
                )
            }
        return TestResult(
            name = name,
            passed = comparison(actual),
            inputs = inputs.toList(),
            actual = actual,
            functionName = function.name
        )
    }
}

fun add(n1: Float, n2: Float, n3: Float, n4: Float): Float {
    return n1 + n2 + n3 + n4
}

@AddTests
val unitTest =
    UnitTest(comparison = { it == 10f }, ::add, 1.0f, 2.0f, 3.0f, 4.0f)

@AddTests
val additionCases =
    listOf(
        UnitTest(comparison = { it == 4f }, ::add, 2f, 1f, 1f, 1f),
        UnitTest(comparison = { it == 8f }, ::add, 2f, 2f, 2f, 2f),
    )
