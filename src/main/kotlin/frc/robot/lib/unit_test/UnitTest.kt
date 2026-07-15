package frc.robot.lib.unit_test

import kotlin.reflect.KFunction
import kotlin.reflect.full.valueParameters

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
    fun test(src: String): TestResult {
        val actual =
            try {
                function.call(*inputs)
            } catch (e: IllegalArgumentException) {
                error(
                    "\nfunction inputs are : ${function.name}(${
                        function.valueParameters.joinToString {
                            "${it.name}: ${
                                it.type.toString().substringAfterLast(".")
                            }"
                        }
                    }) wrong inputs!\ntest source : $src"
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
