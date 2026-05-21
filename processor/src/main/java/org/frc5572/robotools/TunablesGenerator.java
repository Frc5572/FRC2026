package org.frc5572.robotools;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.TypeElement;
import javax.lang.model.element.VariableElement;
import javax.lang.model.type.TypeKind;
import javax.lang.model.type.TypeMirror;
import javax.tools.Diagnostic;
import com.squareup.javapoet.ClassName;
import com.squareup.javapoet.CodeBlock;
import com.squareup.javapoet.FieldSpec;
import com.squareup.javapoet.JavaFile;
import com.squareup.javapoet.MethodSpec;
import com.squareup.javapoet.ParameterizedTypeName;
import com.squareup.javapoet.TypeName;
import com.squareup.javapoet.TypeSpec;

/**
 * Annotation Generator for {@code @Tunable}
 */
public class TunablesGenerator implements AnnotationGenerator {

    private ProcessingEnvironment processingEnv;

    @Override
    public void init(ProcessingEnvironment env) {
        this.processingEnv = env;
    }

    @Override
    public String getAnnotationQualifiedName() {
        return "frc.robot.util.Tunable";
    }

    @Override
    public void generate(TypeElement annotation, RoundEnvironment roundEnv) {
        roundEnv.getElementsAnnotatedWith(annotation).forEach(classElement_ -> {
            TypeElement classElement = (TypeElement) classElement_;
            String binderClassName = classElement.getSimpleName() + "NT";
            String binderPackage = Utilities.getPackageName(classElement);

            var spec = TypeSpec.classBuilder(binderClassName)
                .addModifiers(Modifier.PUBLIC, Modifier.FINAL)
                .addSuperinterface(
                    ClassName.get("org.littletonrobotics.junction.inputs", "LoggableInputs"))
                // .addSuperinterface(ClassName.get(Cloneable.class))
                .addField(FieldSpec.builder(ClassName.get(AtomicBoolean.class), "dirtyFlag",
                    Modifier.PRIVATE, Modifier.FINAL).build())
                .addField(FieldSpec
                    .builder(ClassName.get(classElement), "value", Modifier.PRIVATE, Modifier.FINAL)
                    .build())
                .addMethod(MethodSpec.constructorBuilder()
                    .addParameter(TypeName.get(classElement.asType()), "value")
                    .addModifiers(Modifier.PRIVATE).addCode("this.value = value;\n")
                    .addCode("this.dirtyFlag = new AtomicBoolean(false);").build())
                .addMethod(createBinder(classElement, binderPackage,
                    classElement.getSimpleName().toString() + "NT"))
                .addMethod(
                    MethodSpec.methodBuilder("bind").addModifiers(Modifier.PUBLIC, Modifier.STATIC)
                        .addParameter(ClassName.get(String.class), "key")
                        .addParameter(ClassName.get(classElement), "value")
                        .returns(ClassName.get(binderPackage,
                            classElement.getSimpleName().toString() + "NT"))
                        .addCode(classElement.getSimpleName().toString() + "NT ret = new "
                            + classElement.getSimpleName().toString()
                            + "NT(value);\nbind(key, ret.value(), ret.dirtyFlag);\nreturn ret;")
                        .build())
                .addMethod(MethodSpec.methodBuilder("value").addModifiers(Modifier.PUBLIC)
                    .returns(ClassName.get(classElement)).addCode("return value;").build())
                .addMethod(MethodSpec.methodBuilder("dirtyCheckPeriodic")
                    .addModifiers(Modifier.PUBLIC).addParameter(ClassName.get(String.class), "key")
                    .addParameter(ParameterizedTypeName.get(ClassName.get(Consumer.class),
                        ClassName.get(classElement)), "func")
                    .addCode(
                        "$T.processInputs(key, this);\nif(dirtyFlag.getAndSet(false)) {\n  func.accept(this.value);\n}",
                        ClassName.get("org.littletonrobotics.junction", "Logger"))
                    .build())
                .addMethod(fromLog(classElement))
                .addMethod(MethodSpec.methodBuilder("fromLog").addModifiers(Modifier.PUBLIC)
                    .addAnnotation(Override.class)
                    .addParameter(ClassName.get("org.littletonrobotics.junction", "LogTable"),
                        "table")
                    .addCode("fromLog(table, this.value);").build())
                .addMethod(toLog(classElement))
                .addMethod(MethodSpec.methodBuilder("toLog").addModifiers(Modifier.PUBLIC)
                    .addAnnotation(Override.class)
                    .addParameter(ClassName.get("org.littletonrobotics.junction", "LogTable"),
                        "table")
                    .addCode("toLog(table, this.value);").build())
                .build();

            JavaFile file = JavaFile.builder(binderPackage, spec).build();
            try {
                file.writeTo(processingEnv.getFiler());
            } catch (IOException e) {
                processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR,
                    "Failed to write class", classElement);
                e.printStackTrace();
            }
        });
    }

    private MethodSpec toLog(TypeElement type) {
        var code = CodeBlock.builder();

        for (var el : type.getEnclosedElements()) {
            if (el instanceof VariableElement varElement) {
                if (!el.getModifiers().contains(Modifier.PUBLIC)) {
                    continue;
                }
                if (el.getModifiers().contains(Modifier.FINAL)
                    || el.getModifiers().contains(Modifier.STATIC)) {
                    continue;
                }
                String fieldName = varElement.getSimpleName().toString();
                TypeMirror ty = varElement.asType();
                if (ty.getKind() == TypeKind.DOUBLE || ty.getKind() == TypeKind.BOOLEAN
                    || processingEnv.getTypeUtils().isSameType(ty, processingEnv.getElementUtils()
                        .getTypeElement("java.lang.String").asType())) {
                    code.add("table.put($S, value." + fieldName + ");\n", fieldName);
                } else if (isTunable(ty)) {
                    Element e = processingEnv.getTypeUtils().asElement(ty);
                    if (e instanceof TypeElement typeElement) {
                        String package_ = Utilities.getPackageName(typeElement);
                        TypeName typeName =
                            ClassName.get(package_, typeElement.getSimpleName().toString() + "NT");
                        code.add("$T.toLog(table.getSubtable($S), value." + fieldName + ");\n",
                            typeName, fieldName);
                    } else {
                        throw new RuntimeException("fromLog Tunable type is not a typeelement!");
                    }
                } else if (isEnum(ty)) {
                    code.add("switch(value." + fieldName + ") {\n");
                    for (String item : enumValues(ty)) {
                        code.add("  case " + item + ":\n");
                        code.add("    table.put($S, $S);\n", fieldName, item);
                        code.add("    break;\n");
                    }
                    code.add("  default:\n");
                    code.add("    table.put($S, \"null\");\n", fieldName);
                    code.add("    break;\n");
                    code.add("}");
                }
            }
        }

        return MethodSpec.methodBuilder("toLog").addModifiers(Modifier.PUBLIC, Modifier.STATIC)
            .addParameter(ClassName.get("org.littletonrobotics.junction", "LogTable"), "table")
            .addParameter(ClassName.get(type), "value").addCode(code.build()).build();
    }

    private MethodSpec fromLog(TypeElement type) {
        var code = CodeBlock.builder();

        for (var el : type.getEnclosedElements()) {
            if (el instanceof VariableElement varElement) {
                if (!el.getModifiers().contains(Modifier.PUBLIC)) {
                    continue;
                }
                if (el.getModifiers().contains(Modifier.FINAL)
                    || el.getModifiers().contains(Modifier.STATIC)) {
                    continue;
                }
                String fieldName = varElement.getSimpleName().toString();
                TypeMirror ty = varElement.asType();
                if (ty.getKind() == TypeKind.DOUBLE || ty.getKind() == TypeKind.BOOLEAN
                    || processingEnv.getTypeUtils().isSameType(ty, processingEnv.getElementUtils()
                        .getTypeElement("java.lang.String").asType())) {
                    code.add("value." + fieldName + " = table.get($S, value." + fieldName + ");\n",
                        fieldName);
                } else if (isTunable(ty)) {
                    Element e = processingEnv.getTypeUtils().asElement(ty);
                    if (e instanceof TypeElement typeElement) {
                        String package_ = Utilities.getPackageName(typeElement);
                        TypeName typeName =
                            ClassName.get(package_, typeElement.getSimpleName().toString() + "NT");
                        code.add("$T.fromLog(table.getSubtable($S), value." + fieldName + ");\n",
                            typeName, fieldName);
                    } else {
                        throw new RuntimeException("fromLog Tunable type is not a typeelement!");
                    }
                } else if (isEnum(ty)) {
                    code.add("switch(table.get($S, \"null\")) {\n", fieldName);
                    for (String item : enumValues(ty)) {
                        code.add("  case \"" + item + "\":\n");
                        code.add("    value." + fieldName + " = $T." + item + ";\n",
                            TypeName.get(ty));
                        code.add("    break;\n");
                    }
                    code.add("  default:\n");
                    code.add("    value." + fieldName + " = null;\n");
                    code.add("    break;\n");
                    code.add("}");
                }
            }
        }

        return MethodSpec.methodBuilder("fromLog").addModifiers(Modifier.PUBLIC, Modifier.STATIC)
            .addParameter(ClassName.get("org.littletonrobotics.junction", "LogTable"), "table")
            .addParameter(ClassName.get(type), "value").addCode(code.build()).build();
    }

    private MethodSpec createBinder(TypeElement type, String package_, String name) {

        CodeBlock.Builder code = CodeBlock.builder();

        code.add("final $T ntInstance = NetworkTableInstance.getDefault();\n",
            ClassName.get("edu.wpi.first.networktables", "NetworkTableInstance"));

        for (var el : type.getEnclosedElements()) {
            if (el instanceof VariableElement varElement) {
                if (!el.getModifiers().contains(Modifier.PUBLIC)) {
                    continue;
                }
                if (el.getModifiers().contains(Modifier.FINAL)
                    || el.getModifiers().contains(Modifier.STATIC)) {
                    continue;
                }
                String fieldName = varElement.getSimpleName().toString();
                TypeMirror ty = varElement.asType();
                if (ty.getKind() == TypeKind.DOUBLE) {
                    processDouble(code, fieldName);
                } else if (ty.getKind() == TypeKind.BOOLEAN) {
                    processBoolean(code, fieldName);
                } else if (processingEnv.getTypeUtils().isSameType(ty,
                    processingEnv.getElementUtils().getTypeElement("java.lang.String").asType())) {
                    processString(code, name);
                } else if (isTunable(ty)) {
                    processTunable(code, fieldName, ty);
                } else if (isEnum(ty)) {
                    processEnum(code, fieldName, ty);
                } else {
                    System.out.println(fieldName);
                    processingEnv.getMessager().printError("Unsupported type for tunable: "
                        + processingEnv.getTypeUtils().asElement(ty).getSimpleName(), type);
                }
            }
        }

        return MethodSpec.methodBuilder("bind").addModifiers(Modifier.PUBLIC, Modifier.STATIC)
            .addParameter(ClassName.get(String.class), "key")
            .addParameter(ClassName.get(type), "value")
            .addParameter(ClassName.get(AtomicBoolean.class), "dirtyFlag").addCode(code.build())
            .build();
    }

    private void processDouble(CodeBlock.Builder code, String valueName) {
        processBuiltin("Double", code, valueName);
    }

    private void processBoolean(CodeBlock.Builder code, String valueName) {
        processBuiltin("Boolean", code, valueName);
    }

    private void processString(CodeBlock.Builder code, String valueName) {
        processBuiltin("String", code, valueName);
    }

    private void processBuiltin(String builtin, CodeBlock.Builder code, String valueName) {
        code.add("{\n" //
            + "  var topic = ntInstance.get" + builtin + "Topic(key + \"/" + valueName + "\");\n" //
            + "  var publisher = topic.publish();\n" //
            + "  publisher.accept(value." + valueName + ");\n" //
            + "  ntInstance.addListener(topic, $T.of($T.kValueAll), (ev) -> {\n" //
            + "    var newValue = ev.valueData.value.get" + builtin + "();\n" //
            + "    value." + valueName + " = newValue;\n" //
            + "    dirtyFlag.set(true);\n" //
            + "  });\n" //
            + "}\n", ClassName.get("java.util", "EnumSet"),
            ClassName.get("edu.wpi.first.networktables", "NetworkTableEvent", "Kind"));
    }

    private void processTunable(CodeBlock.Builder code, String valueName, TypeMirror ty) {
        Element e = processingEnv.getTypeUtils().asElement(ty);
        if (e instanceof TypeElement typeElement) {
            String package_ = Utilities.getPackageName(typeElement);
            TypeName typeName =
                ClassName.get(package_, typeElement.getSimpleName().toString() + "NT");
            code.add("$T.bind(key + \"/" + valueName + "\", value." + valueName + ", dirtyFlag);\n",
                typeName);
        } else {
            throw new RuntimeException(
                "processTunable called with type that doesn't have a typeelement!");
        }
    }

    private void processEnum(CodeBlock.Builder code, String valueName, TypeMirror ty) {
        // @SuppressWarnings("resource")
        // SendableChooser<GravityTypeValue> chooser = new SendableChooser<>();
        // chooser.addOption("test", GravityTypeValue.Arm_Cosine);
        // chooser.onChange(newValue -> {

        // });
        // SmartDashboard.putData();

        code.add("{\n");
        code.add("  @SuppressWarnings(\"resource\")\n");
        code.add("  $T<$T> chooser = new SendableChooser<>();\n",
            ClassName.get("edu.wpi.first.wpilibj.smartdashboard", "SendableChooser"),
            TypeName.get(ty));
        for (var item : enumValues(ty)) {
            code.add("  chooser.addOption($S, $T." + item + ");\n", item, TypeName.get(ty));
        }
        code.add("  chooser.onChange(newValue -> {value." + valueName + " = newValue;});\n");
        code.add("  $T dataTable = ntInstance.getTable(key + $S);\n",
            ClassName.get("edu.wpi.first.networktables", "NetworkTable"), "/" + valueName);
        code.add("  $T builder = new SendableBuilderImpl();\n",
            ClassName.get("edu.wpi.first.wpilibj.smartdashboard", "SendableBuilderImpl"));
        code.add("  builder.setTable(dataTable);\n");
        code.add("  $T.publish(chooser, builder);\n",
            ClassName.get("edu.wpi.first.util.sendable", "SendableRegistry"));
        code.add("  builder.startListeners();\n");
        code.add("  dataTable.getEntry(\".name\").setString($S);\n", valueName);
        code.add("}\n");
    }

    private boolean isTunable(TypeMirror ty) {
        Element e = processingEnv.getTypeUtils().asElement(ty);
        if (e == null) {
            return false;
        }
        if (e instanceof TypeElement typeElement) {
            for (var anno : typeElement.getAnnotationMirrors()) {
                String name = anno.getAnnotationType().asElement().getSimpleName().toString();
                if (name.equals("Tunable")) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean isEnum(TypeMirror ty) {
        var el = processingEnv.getTypeUtils().asElement(ty);
        if (el == null) {
            return false;
        }
        if (el.getKind() == ElementKind.ENUM) {
            return true;
        }
        return false;
    }

    private String[] enumValues(TypeMirror ty) {
        return processingEnv.getTypeUtils().asElement(ty).getEnclosedElements().stream()
            .filter(e -> e.getKind() == ElementKind.ENUM_CONSTANT)
            .map(e -> e.getSimpleName().toString()).toArray(String[]::new);
    }

}
