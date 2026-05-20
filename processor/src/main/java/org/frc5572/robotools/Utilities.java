package org.frc5572.robotools;

import java.util.ArrayList;
import java.util.List;
import javax.lang.model.element.AnnotationMirror;
import javax.lang.model.element.AnnotationValue;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.PackageElement;
import javax.lang.model.type.TypeMirror;
import javax.lang.model.util.SimpleAnnotationValueVisitor8;

public class Utilities {

    private Utilities() {}

    public static String getPackageName(Element e) {
        while (e != null) {
            if (e.getKind().equals(ElementKind.PACKAGE)) {
                return ((PackageElement) e).getQualifiedName().toString();
            }
            e = e.getEnclosingElement();
        }

        return null;
    }

    private static class StringVisitor extends SimpleAnnotationValueVisitor8<String, Void> {

        @Override
        public String visitString(String arg0, Void arg1) {
            return arg0;
        }

    };

    public static String stringAnnotationValue(AnnotationValue value) {
        return value.accept(new StringVisitor(), null);
    }

    private static class ClassVisitor extends SimpleAnnotationValueVisitor8<TypeMirror, Void> {

        @Override
        public TypeMirror visitType(TypeMirror arg0, Void arg1) {
            return arg0;
        }

    }

    public static TypeMirror classAnnotationValue(AnnotationValue value) {
        return value.accept(new ClassVisitor(), null);
    }

    private static class ClassListVisitor extends SimpleAnnotationValueVisitor8<Void, Void> {

        private final List<TypeMirror> mirrors;

        /** Find all uses of a type in annotation values */
        public ClassListVisitor(List<TypeMirror> mirrors) {
            super();
            this.mirrors = mirrors;
        }

        @Override
        public Void visitType(TypeMirror arg0, Void arg1) {
            mirrors.add(arg0);
            return null;
        }

        @Override
        public Void visitArray(List<? extends AnnotationValue> arg0, Void arg1) {
            for (var item : arg0) {
                this.visit(item);
            }
            return null;
        }

    }

    public static void classListAnnotationValue(AnnotationValue value, List<TypeMirror> out) {
        value.accept(new ClassListVisitor(out), null);
    }

    public static List<TypeMirror> classListAnnotationValue(AnnotationValue value) {
        List<TypeMirror> out = new ArrayList<>();
        classListAnnotationValue(value, out);
        return out;
    }

    private static class BoolVisitor extends SimpleAnnotationValueVisitor8<Boolean, Void> {

        @Override
        public Boolean visitBoolean(boolean arg0, Void arg1) {
            return arg0;
        }

    }

    public static Boolean boolAnnotationValue(AnnotationValue value) {
        return value.accept(new BoolVisitor(), null);
    }

    private static class AnnotationMirrorVisitor
        extends SimpleAnnotationValueVisitor8<AnnotationMirror, Void> {

        @Override
        public AnnotationMirror visitAnnotation(AnnotationMirror arg0, Void arg1) {
            return arg0;
        }

        @Override
        public AnnotationMirror visitArray(List<? extends AnnotationValue> arg0, Void arg1) {
            for (var item : arg0) {
                var res = this.visit(item, arg1);
                if (res != null) {
                    return res;
                }
            }
            return null;
        }

    }

    public static AnnotationMirror annotationAnnotationValue(AnnotationValue value) {
        return value.accept(new AnnotationMirrorVisitor(), null);
    }

}
