package org.frc5572.robotools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.element.AnnotationMirror;
import javax.lang.model.element.Element;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.TypeElement;
import javax.lang.model.element.VariableElement;
import javax.lang.model.type.TypeMirror;
import javax.tools.Diagnostic;
import javax.tools.Diagnostic.Kind;
import com.squareup.javapoet.ClassName;
import com.squareup.javapoet.FieldSpec;
import com.squareup.javapoet.JavaFile;
import com.squareup.javapoet.MethodSpec;
import com.squareup.javapoet.ParameterSpec;
import com.squareup.javapoet.TypeName;
import com.squareup.javapoet.TypeSpec;

public final class TypeStateBuilderGenerator implements AnnotationGenerator {

    private ProcessingEnvironment processingEnv;

    @Override
    public void init(ProcessingEnvironment env) {
        this.processingEnv = env;
    }

    @Override
    public String getAnnotationQualifiedName() {
        return "frc.robot.util.typestate.TypeStateBuilder";
    }

    @Override
    public void generate(TypeElement annotation, RoundEnvironment roundEnv) {
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
            String builderPackage = Utilities.getPackageName(parent);
            boolean isLinear = false;
            // System.out.println("Processing " + builderPackage + "." + builderName);
            for (var mirror : constructorElement.getAnnotationMirrors()) {
                if (!mirror.getAnnotationType().asElement().getSimpleName().toString()
                    .equals("TypeStateBuilder")) {
                    continue;
                }
                for (var ev : mirror.getElementValues().entrySet()) {
                    if (ev.getKey().getSimpleName().toString().equals("value")) {
                        String res = Utilities.stringAnnotationValue(ev.getValue());
                        if (res != null) {
                            builderName = res;
                        }
                    } else if (ev.getKey().getSimpleName().toString().equals("linear")) {
                        Boolean res = Utilities.boolAnnotationValue(ev.getValue());
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

    private static class TypeStateBuilder {

        private final String name;
        private final ArrayList<RequiredField> requiredFields = new ArrayList<>();
        private final ArrayList<InitField> initFields = new ArrayList<>();
        private final ArrayList<OptionalField> optionalFields = new ArrayList<>();
        private final Field[] fields;
        private final TypeMirror result;
        private final boolean isLinear;

        /** Template builder for TypeState Builders */
        public TypeStateBuilder(String name, boolean isLinear, Field[] fields_, TypeMirror result) {
            this.fields = fields_;
            this.result = result;
            this.name = name;
            this.isLinear = isLinear;
            for (var field : fields_) {
                if (field instanceof RequiredField rField) {
                    requiredFields.add(rField);
                } else if (field instanceof OptionalField oField) {
                    optionalFields.add(oField);
                } else if (field instanceof InitField iField) {
                    initFields.add(iField);
                }
            }
        }

        private static boolean advance(boolean[] enabled, boolean isLinear) {
            for (int i = 0; i < enabled.length; i++) {
                if (enabled[i]) {
                    if (!isLinear) {
                        enabled[i] = false;
                    }
                } else {
                    enabled[i] = true;
                    return true;
                }
            }
            return false;
        }

        /** Create typestate builders and write them to a typespec */
        public void apply(TypeSpec.Builder builder) {
            boolean[] enabled = new boolean[requiredFields.size()];

            MethodSpec.Builder constructor = MethodSpec.constructorBuilder();
            for (var field : initFields) {
                builder.addField(FieldSpec.builder(TypeName.get(field.type), field.name + "_",
                    Modifier.PRIVATE, Modifier.FINAL).build());
                constructor.addParameter(
                    ParameterSpec.builder(TypeName.get(field.type), field.name + "_").build());
                constructor.addCode("this." + field.name + "_ = " + field.name + "_;\n");
            }
            for (var field : optionalFields) {
                builder.addField(FieldSpec.builder(TypeName.get(field.type), field.name + "_",
                    Modifier.PRIVATE, Modifier.FINAL).build());
                constructor.addCode("this." + field.name + "_ = " + field.default_code + ";\n");
            }
            builder.addMethod(constructor.addModifiers(Modifier.PUBLIC).build());
            constructor = MethodSpec.constructorBuilder();
            boolean isDifferent = false;
            for (var field : initFields) {
                constructor.addParameter(
                    ParameterSpec.builder(TypeName.get(field.type), field.name + "_").build());
                constructor.addCode("this." + field.name + "_ = " + field.name + "_;\n");
            }
            for (var field : optionalFields) {
                constructor.addParameter(
                    ParameterSpec.builder(TypeName.get(field.type), field.name + "_").build());
                constructor.addCode("this." + field.name + "_ = " + field.name + "_;\n");
                isDifferent = true;
            }
            if (isDifferent) {
                builder.addMethod(constructor.addModifiers(Modifier.PRIVATE).build());
            }
            addMethods(builder, enabled);
            while (advance(enabled, isLinear)) {
                constructor = MethodSpec.constructorBuilder();
                for (int i = 0; i < enabled.length; i++) {
                    if (enabled[i]) {
                        var field = requiredFields.get(i);
                        constructor.addParameter(ParameterSpec
                            .builder(TypeName.get(field.type), field.name + "_").build());
                        constructor.addCode("this." + field.name + "_ = " + field.name + "_;\n");
                    }
                }
                TypeSpec.Builder stepClass = TypeSpec.classBuilder(getName(enabled));
                for (int i = 0; i < enabled.length; i++) {
                    if (enabled[i]) {
                        var field = requiredFields.get(i);
                        stepClass.addField(FieldSpec.builder(TypeName.get(field.type),
                            field.name + "_", Modifier.PRIVATE, Modifier.FINAL).build());
                    }
                }
                for (var field : initFields) {
                    stepClass.addField(FieldSpec.builder(TypeName.get(field.type), field.name + "_",
                        Modifier.PRIVATE, Modifier.FINAL).build());
                    constructor.addParameter(
                        ParameterSpec.builder(TypeName.get(field.type), field.name + "_").build());
                    constructor.addCode("this." + field.name + "_ = " + field.name + "_;\n");
                }
                for (var field : optionalFields) {
                    stepClass.addField(FieldSpec.builder(TypeName.get(field.type), field.name + "_",
                        Modifier.PRIVATE, Modifier.FINAL).build());
                    constructor.addParameter(
                        ParameterSpec.builder(TypeName.get(field.type), field.name + "_").build());
                    constructor.addCode("this." + field.name + "_ = " + field.name + "_;\n");
                }
                stepClass.addMethod(constructor.addModifiers(Modifier.PRIVATE).build());
                addMethods(stepClass, enabled);
                builder.addType(stepClass.addModifiers(Modifier.PUBLIC).build());
            }
        }

        private String getName(boolean[] enabled) {
            StringBuilder builderName = new StringBuilder(this.name);
            boolean any = false;
            for (int i = 0; i < enabled.length; i++) {
                builderName.append(enabled[i] ? "1" : "0");
                any = any || enabled[i];
            }
            if (any) {
                return builderName.toString();
            } else {
                return this.name;
            }
        }

        private void addMethods(TypeSpec.Builder builder, boolean[] enabled) {
            boolean isFinishable = true;
            for (int i = 0; i < enabled.length; i++) {
                if (!enabled[i]) {
                    isFinishable = false;
                    break;
                }
            }
            if (isFinishable) {
                TypeName returnType = TypeName.get(result);
                MethodSpec.Builder finish = MethodSpec.methodBuilder("finish")
                    .addModifiers(Modifier.PUBLIC).returns(returnType);
                String code = "return new " + returnType + "(";
                boolean isFirst = true;
                for (Field field : fields) {
                    if (!isFirst) {
                        code += ", ";
                    }
                    code += field.name + "_";
                    isFirst = false;
                }
                finish.addCode(code + ");\n");
                builder.addMethod(finish.build());
            }
            for (int i = 0; i < enabled.length; i++) {
                if (!enabled[i]) {
                    var field = requiredFields.get(i);
                    boolean[] next = new boolean[enabled.length];
                    System.arraycopy(enabled, 0, next, 0, enabled.length);
                    next[i] = true;
                    String nextName = getName(next);
                    MethodSpec.Builder method =
                        MethodSpec.methodBuilder(field.name).returns(ClassName.get("", nextName));
                    method.addParameter(
                        ParameterSpec.builder(TypeName.get(field.type), field.name + "_").build());
                    String code = "return new " + nextName + "(";
                    boolean isFirst = true;
                    for (int j = 0; j < enabled.length; j++) {
                        if (enabled[j] || i == j) {
                            if (!isFirst) {
                                code += ", ";
                            }
                            code += requiredFields.get(j).name + "_";
                            isFirst = false;
                        }
                    }
                    for (var field_ : initFields) {
                        if (!isFirst) {
                            code += ", ";
                        }
                        code += field_.name + "_";
                        isFirst = false;
                    }
                    for (var field_ : optionalFields) {
                        if (!isFirst) {
                            code += ", ";
                        }
                        code += field_.name + "_";
                        isFirst = false;
                    }
                    method.addCode(code + ");\n");
                    builder.addMethod(method.addModifiers(Modifier.PUBLIC).build());

                    if (field.alt != null) {
                        method = MethodSpec.methodBuilder(field.name)
                            .returns(ClassName.get("", nextName));
                        method.addParameter(ParameterSpec
                            .builder(TypeName.get(field.alt.type), field.alt.parameterName)
                            .build());
                        code = "return new " + nextName + "(";
                        isFirst = true;
                        for (int j = 0; j < enabled.length; j++) {
                            if (i == j) {
                                if (!isFirst) {
                                    code += ", ";
                                }
                                code += field.alt.code;
                                isFirst = false;
                                continue;
                            }
                            if (enabled[j]) {
                                if (!isFirst) {
                                    code += ", ";
                                }
                                code += requiredFields.get(j).name + "_";
                                isFirst = false;
                            }
                        }
                        for (var field_ : initFields) {
                            if (!isFirst) {
                                code += ", ";
                            }
                            code += field_.name + "_";
                            isFirst = false;
                        }
                        for (var field_ : optionalFields) {
                            if (!isFirst) {
                                code += ", ";
                            }
                            code += field_.name + "_";
                            isFirst = false;
                        }
                        method.addCode(code + ");\n");
                        builder.addMethod(method.addModifiers(Modifier.PUBLIC).build());
                    }
                    if (isLinear) {
                        break;
                    }
                }
            }
            String thisName = getName(enabled);
            for (var field : optionalFields) {
                MethodSpec.Builder method =
                    MethodSpec.methodBuilder(field.name).returns(ClassName.get("", thisName));
                method.addParameter(
                    ParameterSpec.builder(TypeName.get(field.type), field.name + "_").build());
                String code = "return new " + thisName + "(";
                boolean isFirst = true;
                for (int i = 0; i < enabled.length; i++) {
                    if (enabled[i]) {
                        if (!isFirst) {
                            code += ", ";
                        }
                        code += requiredFields.get(i).name + "_";
                        isFirst = false;
                    }
                }
                for (var field_ : initFields) {
                    if (!isFirst) {
                        code += ", ";
                    }
                    code += field_.name + "_";
                    isFirst = false;
                }
                for (var field_ : optionalFields) {
                    if (!isFirst) {
                        code += ", ";
                    }
                    code += field_.name + "_";
                    isFirst = false;
                }
                method.addCode(code + ");\n");
                builder.addMethod(method.addModifiers(Modifier.PUBLIC).build());

                if (field.alt != null) {
                    method =
                        MethodSpec.methodBuilder(field.name).returns(ClassName.get("", thisName));
                    method.addParameter(ParameterSpec
                        .builder(TypeName.get(field.alt.type), field.alt.parameterName).build());
                    code = "return new " + thisName + "(";
                    isFirst = true;
                    for (int i = 0; i < enabled.length; i++) {
                        if (enabled[i]) {
                            if (!isFirst) {
                                code += ", ";
                            }
                            code += requiredFields.get(i).name + "_";
                            isFirst = false;
                        }
                    }
                    for (var field_ : initFields) {
                        if (!isFirst) {
                            code += ", ";
                        }
                        code += field_.name + "_";
                        isFirst = false;
                    }
                    for (var field_ : optionalFields) {
                        if (!isFirst) {
                            code += ", ";
                        }
                        if (field_ == field) {
                            code += field.alt.code;
                        } else {
                            code += field_.name + "_";
                        }
                        isFirst = false;
                    }
                    method.addCode(code + ");\n");
                    builder.addMethod(method.addModifiers(Modifier.PUBLIC).build());
                }
            }
        }

        /** Base class for fields */
        public static class Field {
            /** Field type */
            public final TypeMirror type;
            /** Field name */
            public final String name;

            /** Base class for fields */
            public Field(TypeMirror type, String name) {
                this.type = type;
                this.name = name;
            }
        }

        /** Field that must be provided when creating a builder */
        public static class InitField extends Field {
            /** Field that must be provided when creating a builder */
            public InitField(TypeMirror type, String name) {
                super(type, name);
            }
        }

        /** A non-init field */
        public static class MethodField extends Field {
            /** An alternative method for fulfilling this field. */
            public final AltMethod alt;

            /** A non-init field */
            public MethodField(TypeMirror type, String name, AltMethod alt) {
                super(type, name);
                this.alt = alt;
            }

            /** A non-init field */
            public MethodField(TypeMirror type, String name) {
                this(type, name, null);
            }
        }

        /** A field that is required to finish the builder. */
        public static class RequiredField extends MethodField {

            /** A field that is required to finish the builder. */
            public RequiredField(TypeMirror type, String name, AltMethod alt) {
                super(type, name, alt);
            }

            /** A field that is required to finish the builder. */
            public RequiredField(TypeMirror type, String name) {
                super(type, name);
            }

            /** A field that is required to finish the builder. */
            public static RequiredField fromAnnotation(TypeMirror type, String name,
                AnnotationMirror mirror) {
                AltMethod alt = null;
                for (var ev : mirror.getElementValues().entrySet()) {
                    if (ev.getKey().getSimpleName().toString().equals("alt")) {
                        AnnotationMirror altMirror =
                            Utilities.annotationAnnotationValue(ev.getValue());
                        alt = AltMethod.fromAnnotation(altMirror, name);
                    }
                }
                return new RequiredField(type, name, alt);
            }
        }

        /** A field that has a default in case it is not specified. */
        public static class OptionalField extends MethodField {
            /** Java expression that provides the default value. */
            public final String default_code;

            /** A field that has a default in case it is not specified. */
            public OptionalField(TypeMirror type, String name, AltMethod alt, String default_code) {
                super(type, name, alt);
                this.default_code = default_code;
            }

            /** A field that has a default in case it is not specified. */
            public OptionalField(TypeMirror type, String name, String default_code) {
                super(type, name);
                this.default_code = default_code;
            }

            /** A field that has a default in case it is not specified. */
            public static OptionalField fromAnnotation(TypeMirror type, String name,
                AnnotationMirror mirror) {
                String defaultCode = "";
                AltMethod alt = null;
                for (var ev : mirror.getElementValues().entrySet()) {
                    if (ev.getKey().getSimpleName().toString().equals("value")) {
                        defaultCode = Utilities.stringAnnotationValue(ev.getValue());
                    } else if (ev.getKey().getSimpleName().toString().equals("alt")) {
                        AnnotationMirror altMirror =
                            Utilities.annotationAnnotationValue(ev.getValue());
                        alt = AltMethod.fromAnnotation(altMirror, name);
                    }
                }
                return new OptionalField(type, name, alt, defaultCode);
            }
        }

        /** Alternative method for a field. */
        public static record AltMethod(TypeMirror type, String parameterName, String code) {
            /** Alternative method for a field. */
            public static AltMethod fromAnnotation(AnnotationMirror mirror, String defaultName) {
                if (mirror == null) {
                    System.out.println("annotation is null");
                    return null;
                }

                if (!mirror.getAnnotationType().asElement().getSimpleName().toString()
                    .equals("AltMethod")) {
                    System.out.println("annotation name doesn't match "
                        + mirror.getAnnotationType().asElement().getSimpleName().toString());
                    return null;
                }
                TypeMirror type = null;
                String parameterName = defaultName;
                String code = null;

                for (var ev : mirror.getElementValues().entrySet()) {
                    if (ev.getKey().getSimpleName().toString().equals("type")) {
                        type = Utilities.classAnnotationValue(ev.getValue());
                    } else if (ev.getKey().getSimpleName().toString().equals("parameter_name")) {
                        parameterName = Utilities.stringAnnotationValue(ev.getValue());
                    } else if (ev.getKey().getSimpleName().toString().equals("value")) {
                        code = Utilities.stringAnnotationValue(ev.getValue());
                    }
                }

                if (type == null) {
                    System.out.println("Missing type");
                    return null;
                }
                if (code == null) {
                    System.out.println("Missing code");
                    return null;
                }

                return new AltMethod(type, parameterName, code);
            }
        }

    }
}

