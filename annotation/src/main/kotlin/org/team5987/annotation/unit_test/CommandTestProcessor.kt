package org.team5987.annotation.unit_test

import com.google.devtools.ksp.processing.Dependencies
import com.google.devtools.ksp.processing.KSPLogger
import com.google.devtools.ksp.processing.Resolver
import com.google.devtools.ksp.processing.SymbolProcessor
import com.google.devtools.ksp.processing.SymbolProcessorEnvironment
import com.google.devtools.ksp.processing.SymbolProcessorProvider
import com.google.devtools.ksp.symbol.ClassKind
import com.google.devtools.ksp.symbol.FileLocation
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.devtools.ksp.symbol.KSClassDeclaration
import com.google.devtools.ksp.symbol.KSPropertyDeclaration
import com.google.devtools.ksp.validate
import com.squareup.kotlinpoet.ClassName
import com.squareup.kotlinpoet.FileSpec
import com.squareup.kotlinpoet.FunSpec
import com.squareup.kotlinpoet.LIST
import com.squareup.kotlinpoet.ParameterizedTypeName.Companion.parameterizedBy
import com.squareup.kotlinpoet.TypeSpec
import com.squareup.kotlinpoet.ksp.writeTo

private const val COMMAND_TEST_ANNOTATION_NAME =
    "org.team5987.annotation.unit_test.AddCommandTests"
private const val COMMAND_TEST_NAME =
    "frc.robot.lib.unit_test.CommandTest"
private const val COMMAND_TEST_GENERATED_PACKAGE =
    "frc.robot.lib.unit_test.generated"
private const val COMMAND_TEST_GENERATED_CLASS = "CommandTests"

class CommandTestProcessor(
    environment: SymbolProcessorEnvironment
) : SymbolProcessor {
    private val codeGenerator = environment.codeGenerator
    private val logger: KSPLogger = environment.logger
    private val projectPath = environment.options["unitTests.projectPath"]
    private val enabled =
        environment.options["unitTests.enabled"]?.toBoolean() == true
    private var generated = false

    override fun process(resolver: Resolver): List<KSAnnotated> {
        if (!enabled || generated) return emptyList()

        val symbols =
            resolver
                .getSymbolsWithAnnotation(COMMAND_TEST_ANNOTATION_NAME)
                .toList()
        val deferred = symbols.filterNot { it.validate() }
        if (deferred.isNotEmpty()) return deferred

        val properties = symbols.filterIsInstance<KSPropertyDeclaration>()
        if (properties.isEmpty()) return deferred

        val duplicateNames =
            properties.groupBy { it.simpleName.asString() }.filterValues {
                it.size > 1
            }
        duplicateNames.forEach { (name, declarations) ->
            declarations.forEach {
                logger.error(
                    "@AddCommandTests property names must be unique; " +
                        "'$name' is used more than once",
                    it
                )
            }
        }

        val testClass = TypeSpec.classBuilder(COMMAND_TEST_GENERATED_CLASS)
        val file =
            FileSpec.builder(
                COMMAND_TEST_GENERATED_PACKAGE,
                COMMAND_TEST_GENERATED_CLASS
            )
        val sourceFiles =
            mutableSetOf<com.google.devtools.ksp.symbol.KSFile>()

        properties
            .filterNot { it.simpleName.asString() in duplicateNames }
            .forEach { property ->
                property.containingFile?.let(sourceFiles::add)

                val sourceReference =
                    sourceReference(property, file) ?: return@forEach
                val sourceLocation = property.sourceLocation()

                when (property.testKind()) {
                    TestKind.SINGLE ->
                        testClass.addFunction(
                            createSingleTest(
                                property.simpleName.asString(),
                                sourceReference,
                                sourceLocation
                            )
                        )
                    TestKind.LIST ->
                        testClass.addFunction(
                            createListTests(
                                property.simpleName.asString(),
                                sourceReference,
                                sourceLocation
                            )
                        )
                    null ->
                        logger.error(
                            "@AddCommandTests can only annotate CommandTest " +
                                "or List<CommandTest> properties",
                            property
                        )
                }
            }

        if (testClass.funSpecs.isEmpty()) return deferred

        file.addType(testClass.build())
        file.build()
            .writeTo(
                codeGenerator,
                Dependencies(true, *sourceFiles.toTypedArray())
            )
        generated = true
        return deferred
    }

    private fun sourceReference(
        property: KSPropertyDeclaration,
        file: FileSpec.Builder
    ): String? {
        val owner = property.parentDeclaration
        if (owner == null) {
            file.addImport(
                property.packageName.asString(),
                property.simpleName.asString()
            )
            return property.simpleName.asString()
        }

        val ownerClass = owner as? KSClassDeclaration
        if (ownerClass?.classKind != ClassKind.OBJECT) {
            logger.error(
                "@AddCommandTests properties must be top-level or declared " +
                    "in an object",
                property
            )
            return null
        }

        val qualifiedName = ownerClass.qualifiedName?.asString() ?: return null
        val packageName = ownerClass.packageName.asString()
        val relativeName = qualifiedName.removePrefix("$packageName.")
        val ownerClassName = ClassName(packageName, relativeName.split('.'))
        file.addImport(ownerClassName)
        return "${ownerClassName.simpleName}.${property.simpleName.asString()}"
    }

    private fun KSPropertyDeclaration.testKind(): TestKind? {
        val resolvedType = type.resolve()
        val declarationName =
            resolvedType.declaration.qualifiedName?.asString()

        if (declarationName == COMMAND_TEST_NAME) return TestKind.SINGLE

        val isList =
            declarationName == "kotlin.collections.List" ||
                declarationName == "kotlin.collections.MutableList"
        if (!isList) return null

        val elementName =
            resolvedType.arguments
                .singleOrNull()
                ?.type
                ?.resolve()
                ?.declaration
                ?.qualifiedName
                ?.asString()
        return if (elementName == COMMAND_TEST_NAME) TestKind.LIST else null
    }

    private fun KSPropertyDeclaration.sourceLocation(): String =
        (location as? FileLocation)?.let {
            "${it.filePath.toProjectPath()}:${it.lineNumber}"
        } ?: containingFile?.filePath?.toProjectPath() ?: "unknown source"

    private fun String.toProjectPath(): String {
        val rootPath = projectPath?.trimEnd('/', '\\') ?: return this
        return removePrefix(rootPath).trimStart('/', '\\')
    }

    private fun createSingleTest(
        name: String,
        source: String,
        sourceLocation: String
    ): FunSpec =
        FunSpec.builder(name)
            .addAnnotation(TEST)
            .addCode(assertionCode(source, sourceLocation))
            .build()

    private fun createListTests(
        name: String,
        source: String,
        sourceLocation: String
    ): FunSpec =
        FunSpec.builder(name)
            .addAnnotation(TEST_FACTORY)
            .returns(LIST.parameterizedBy(DYNAMIC_TEST))
            .beginControlFlow(
                "return %L.mapIndexed { index, commandTest ->",
                source
            )
            .beginControlFlow(
                "%T.dynamicTest(%S + index)",
                DYNAMIC_TEST,
                "${name}_"
            )
            .addCode(assertionCode("commandTest", sourceLocation))
            .endControlFlow()
            .endControlFlow()
            .build()

    private fun assertionCode(source: String, sourceLocation: String) =
        com.squareup.kotlinpoet.CodeBlock.builder()
            .addStatement("val result = %L.test(%S)", source, sourceLocation)
            .beginControlFlow("if (!result.passed)")
            .addStatement(
                "throw %T(%S + %S + result.failures.joinToString(%S))",
                ASSERTION_ERROR,
                "\nCommand test source: ",
                "$sourceLocation\n",
                "\n"
            )
            .endControlFlow()
            .build()

    private enum class TestKind {
        SINGLE,
        LIST
    }

    private companion object {
        val TEST = ClassName("org.junit.jupiter.api", "Test")
        val TEST_FACTORY = ClassName("org.junit.jupiter.api", "TestFactory")
        val DYNAMIC_TEST = ClassName("org.junit.jupiter.api", "DynamicTest")
        val ASSERTION_ERROR = ClassName("kotlin", "AssertionError")
    }
}

class CommandTestProcessorProvider : SymbolProcessorProvider {
    override fun create(
        environment: SymbolProcessorEnvironment
    ): SymbolProcessor = CommandTestProcessor(environment)
}
