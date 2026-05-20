package org.frc5572.robotools;

import java.io.IOException;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.TypeElement;
import javax.tools.Diagnostic;
import com.squareup.javapoet.ClassName;
import com.squareup.javapoet.JavaFile;
import com.squareup.javapoet.MethodSpec;
import com.squareup.javapoet.TypeSpec;

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
                .addMethod(MethodSpec.constructorBuilder().addModifiers(Modifier.PRIVATE).build())
                .addMethod(createBinder(classElement)).build();

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

    private MethodSpec createBinder(TypeElement type) {
        return MethodSpec.methodBuilder("bind").addModifiers(Modifier.PUBLIC, Modifier.STATIC)
            .addParameter(ClassName.get(String.class), "key")
            .addParameter(ClassName.get(type), "value").returns(ClassName.get(type))
            // TODO add code for binder
            .addCode("return value;").build();
    }

}
