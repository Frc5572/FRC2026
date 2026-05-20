package org.frc5572.robotools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.TypeElement;
import javax.lang.model.type.TypeMirror;
import javax.tools.Diagnostic;
import com.squareup.javapoet.JavaFile;
import com.squareup.javapoet.MethodSpec;
import com.squareup.javapoet.ParameterSpec;
import com.squareup.javapoet.TypeName;
import com.squareup.javapoet.TypeSpec;

public class GenerateEmptyIOGenerator implements AnnotationGenerator {

    private ProcessingEnvironment processingEnv;

    @Override
    public void init(ProcessingEnvironment env) {
        this.processingEnv = env;
    }

    @Override
    public String getAnnotationQualifiedName() {
        return "frc.robot.util.GenerateEmptyIO";
    }

    @Override
    public void generate(TypeElement annotation, RoundEnvironment roundEnv) {
        roundEnv.getElementsAnnotatedWith(annotation).forEach(classElement_ -> {
            TypeElement classElement = (TypeElement) classElement_;
            String emptyClassName = classElement.getSimpleName() + "Empty";
            String emptyPackage = Utilities.getPackageName(classElement);

            List<TypeMirror> params = new ArrayList<>();

            for (var mirror : classElement.getAnnotationMirrors()) {
                if (!mirror.getAnnotationType().asElement().getSimpleName().toString()
                    .equals("GenerateEmptyIO")) {
                    continue;
                }
                for (var ev : mirror.getElementValues().entrySet()) {
                    if (ev.getKey().getSimpleName().toString().equals("value")) {
                        Utilities.classListAnnotationValue(ev.getValue(), params);
                    }
                }
            }

            var specBuilder = TypeSpec.classBuilder(emptyClassName)
                .addSuperinterface(TypeName.get(classElement.asType()))
                .addModifiers(Modifier.PUBLIC, Modifier.FINAL);

            AtomicInteger i = new AtomicInteger();
            var constructor = MethodSpec.constructorBuilder().addModifiers(Modifier.PUBLIC)
                .addParameters(params.stream().map(ty -> {
                    return ParameterSpec.builder(TypeName.get(ty), "arg" + i.incrementAndGet())
                        .build();
                }).toList()).build();

            for (var element : classElement.getEnclosedElements()) {
                if (element instanceof ExecutableElement javaMethod) {
                    specBuilder
                        .addMethod(MethodSpec.methodBuilder(javaMethod.getSimpleName().toString())
                            .addModifiers(Modifier.PUBLIC).addAnnotation(Override.class)
                            .returns(TypeName.VOID)
                            .addParameters(javaMethod.getParameters().stream().map(param -> {
                                return ParameterSpec.builder(TypeName.get(param.asType()),
                                    param.getSimpleName().toString()).build();
                            }).toList()).addCode("// Intentionally do nothing").build());
                }
            }

            specBuilder = specBuilder.addMethod(constructor);

            var spec = specBuilder.build();

            JavaFile file = JavaFile.builder(emptyPackage, spec).build();
            try {
                file.writeTo(processingEnv.getFiler());
            } catch (IOException e) {
                processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR,
                    "Failed to write class", classElement);
                e.printStackTrace();
            }
        });
    }

}
