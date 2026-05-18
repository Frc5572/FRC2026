package org.frc5572.robotools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.PackageElement;
import javax.lang.model.element.TypeElement;
import javax.lang.model.element.VariableElement;
import javax.lang.model.type.TypeMirror;
import javax.tools.Diagnostic;
import javax.tools.Diagnostic.Kind;
import com.squareup.javapoet.JavaFile;
import com.squareup.javapoet.MethodSpec;
import com.squareup.javapoet.ParameterSpec;
import com.squareup.javapoet.TypeName;
import com.squareup.javapoet.TypeSpec;

/**
 * Annotation processor for checks. Used by VS Code.
 */
@SupportedAnnotationTypes({"frc.robot.util.typestate.TypeStateBuilder",
    "frc.robot.util.GenerateEmptyIO"})
@SupportedSourceVersion(SourceVersion.RELEASE_11)
public class RobotProcessor extends AbstractProcessor {

    /** Initialization function */
    @Override
    public synchronized void init(ProcessingEnvironment processingEnv) {
        super.init(processingEnv);
    }

    /** Process all elements. */
    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        boolean success = false;
        for (var anno : annotations) {
            // System.out.println("Trying " + anno.getQualifiedName());
            if (anno.getSimpleName().toString().equals("GenerateEmptyIO")) {
                processGenerateEmptyIO(anno, roundEnv);
                success = true;
            } else if (anno.getSimpleName().toString().equals("TypeStateBuilder")) {
                processGenerateTypeStateBuilder(anno, roundEnv);
                success = true;
            }
        }
        return success;
    }

    private void processGenerateTypeStateBuilder(TypeElement annotation,
        RoundEnvironment roundEnv) {
        roundEnv.getElementsAnnotatedWith(annotation).forEach(constructorElement_ -> {
            ExecutableElement constructorElement = (ExecutableElement) constructorElement_;
            Element parent_ = constructorElement.getEnclosingElement();
            if (!(parent_ instanceof TypeElement)) {
                processingEnv.getMessager().printMessage(Kind.ERROR,
                    "TypeStateBuilder constructor must be the direct child of a TypeElement (e.g. class). Instead found "
                        + parent_.getKind().toString() + ".",
                    constructorElement);
            }
            TypeElement parent = (TypeElement) parent_;
            String builderName = parent.getSimpleName() + "Builder";
            String builderPackage = getPackageName(parent);
            boolean isLinear = false;
            // System.out.println("Processing " + builderPackage + "." + builderName);
            for (var mirror : constructorElement.getAnnotationMirrors()) {
                if (!mirror.getAnnotationType().asElement().getSimpleName().toString()
                    .equals("TypeStateBuilder")) {
                    continue;
                }
                for (var ev : mirror.getElementValues().entrySet()) {
                    if (ev.getKey().getSimpleName().toString().equals("value")) {
                        String res = ev.getValue().accept(new StringVisitor(), null);
                        if (res != null) {
                            builderName = res;
                        }
                    } else if (ev.getKey().getSimpleName().toString().equals("linear")) {
                        Boolean res = ev.getValue().accept(new BoolVisitor(), null);
                        if (res != null) {
                            isLinear = res;
                        }
                    }
                }
            }

            List<TypeStateBuilder.Field> fields = new ArrayList<>();
            List<? extends VariableElement> params = constructorElement.getParameters();
            for (int i = 0; i < params.size(); i++) {
                boolean found = false;
                VariableElement param = params.get(i);
                for (var mirror : param.getAnnotationMirrors()) {
                    if (mirror.getAnnotationType().asElement().getSimpleName().toString()
                        .equals("InitField")) {
                        if (found) {
                            processingEnv.getMessager().printMessage(Kind.ERROR,
                                "Each parameter of a TypeStateBuilder constructor can only "
                                    + "have one of @InitField, @RequiredField or @OptionalField",
                                param);
                        }
                        fields.add(new TypeStateBuilder.InitField(param.asType(),
                            param.getSimpleName().toString()));
                        found = true;
                    } else if (mirror.getAnnotationType().asElement().getSimpleName().toString()
                        .equals("RequiredField")) {
                        if (found) {
                            processingEnv.getMessager().printMessage(Kind.ERROR,
                                "Each parameter of a TypeStateBuilder constructor can only "
                                    + "have one of @InitField, @RequiredField or @OptionalField",
                                param);
                        }
                        fields.add(TypeStateBuilder.RequiredField.fromAnnotation(param.asType(),
                            param.getSimpleName().toString(), mirror));
                        found = true;
                    } else if (mirror.getAnnotationType().asElement().getSimpleName().toString()
                        .equals("OptionalField")) {
                        if (found) {
                            processingEnv.getMessager().printMessage(Kind.ERROR,
                                "Each parameter of a TypeStateBuilder constructor can only "
                                    + "have one of @InitField, @RequiredField or @OptionalField",
                                param);
                        }
                        fields.add(TypeStateBuilder.OptionalField.fromAnnotation(param.asType(),
                            param.getSimpleName().toString(), mirror));
                        found = true;
                    }
                }
                if (!found) {
                    processingEnv.getMessager()
                        .printMessage(Kind.ERROR,
                            "Each parameter of a TypeStateBuilder constructor must "
                                + "have one of @InitField, @RequiredField or @OptionalField",
                            param);
                }
            }

            var specBuilder =
                TypeSpec.classBuilder(builderName).addModifiers(Modifier.PUBLIC, Modifier.FINAL);

            TypeStateBuilder typeStateBuilder = new TypeStateBuilder(builderName, isLinear,
                fields.toArray(TypeStateBuilder.Field[]::new), parent.asType());
            typeStateBuilder.apply(specBuilder);

            var spec = specBuilder.build();

            JavaFile file = JavaFile.builder(builderPackage, spec).build();
            try {
                file.writeTo(processingEnv.getFiler());
            } catch (IOException e) {
                processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR,
                    "Failed to write class", constructorElement);
                e.printStackTrace();
            }
        });
    }

    private void processGenerateEmptyIO(TypeElement annotation, RoundEnvironment roundEnv) {
        roundEnv.getElementsAnnotatedWith(annotation).forEach(classElement_ -> {
            TypeElement classElement = (TypeElement) classElement_;
            String emptyClassName = classElement.getSimpleName() + "Empty";
            String emptyPackage = getPackageName(classElement);

            List<TypeMirror> params = new ArrayList<>();

            for (var mirror : classElement.getAnnotationMirrors()) {
                if (!mirror.getAnnotationType().asElement().getSimpleName().toString()
                    .equals("GenerateEmptyIO")) {
                    continue;
                }
                for (var ev : mirror.getElementValues().entrySet()) {
                    if (ev.getKey().getSimpleName().toString().equals("value")) {
                        ev.getValue().accept(new ClassListVisitor(params), null);
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

    private static String getPackageName(Element e) {
        while (e != null) {
            if (e.getKind().equals(ElementKind.PACKAGE)) {
                return ((PackageElement) e).getQualifiedName().toString();
            }
            e = e.getEnclosingElement();
        }

        return null;
    }

}
