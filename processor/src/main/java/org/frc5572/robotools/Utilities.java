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

/**
 * Utility methods for annotation processors or other compile-time tools.
 *
 * <p>
 * All methods are static; this class is not intended to be instantiated.
 */
public class Utilities {

    private Utilities() {}

    /**
     * Returns the fully qualified package name that contains the given element.
     *
     * <p>
     * The method walks up the enclosing-element chain until a {@link ElementKind#PACKAGE} is found.
     * If no package element is found (which is unlikely in normal use), {@code null} is returned.
     *
     * @param e the element whose package name should be determined; may be {@code null}
     * @return the fully qualified package name of the element, or {@code null} if no package
     *         element can be found
     */
    public static String getPackageName(Element e) {
        while (e != null) {
            if (e.getKind().equals(ElementKind.PACKAGE)) {
                return ((PackageElement) e).getQualifiedName().toString();
            }
            e = e.getEnclosingElement();
        }

        return null;
    }

    /**
     * Extracts a {@link String} value from an {@link AnnotationValue}.
     *
     * <p>
     * This is typically used for annotation members declared as {@code String}.
     *
     * @param value the annotation value to read
     * @return the contained string value, or {@code null} if the value is not a string
     */
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

    /**
     * Extracts a single {@link TypeMirror} (class) value from an {@link AnnotationValue}.
     *
     * <p>
     * This is typically used for annotation members declared as a single {@code Class} type.
     *
     * @param value the annotation value to read
     * @return the contained {@link TypeMirror}, or {@code null} if the value is not a single type
     */
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

    /**
     * Extracts all {@link TypeMirror} (class) values from the given {@link AnnotationValue} into an
     * existing list.
     *
     * <p>
     * This method supports both single class values and arrays of classes from annotation members
     * declared as {@code Class} or {@code Class[]}.
     *
     * @param value the annotation value to read
     * @param out the list into which all discovered {@link TypeMirror}s are added; must not be
     *        {@code null}
     */
    public static void classListAnnotationValue(AnnotationValue value, List<TypeMirror> out) {
        value.accept(new ClassListVisitor(out), null);
    }

    /**
     * Extracts all {@link TypeMirror} (class) values from the given {@link AnnotationValue}.
     *
     * <p>
     * This is a convenience overload of {@link #classListAnnotationValue(AnnotationValue, List)}
     * that creates and returns a new list.
     *
     * @param value the annotation value to read
     * @return a list of all {@link TypeMirror}s contained in the annotation value; never
     *         {@code null}, but may be empty
     */
    public static List<TypeMirror> classListAnnotationValue(AnnotationValue value) {
        List<TypeMirror> out = new ArrayList<>();
        classListAnnotationValue(value, out);
        return out;
    }

    /**
     * Extracts a {@code boolean} value from an {@link AnnotationValue}.
     *
     * <p>
     * This is typically used for annotation members declared as {@code boolean}.
     *
     * @param value the annotation value to read
     * @return the contained boolean value, or {@code null} if the value is not a boolean
     */
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

    /**
     * Extracts a nested {@link AnnotationMirror} from an {@link AnnotationValue}.
     *
     * <p>
     * This is typically used for annotation members declared as another annotation type, possibly
     * in an array.
     *
     * <p>
     * If the value is an array, the first nested annotation found is returned.
     *
     * @param value the annotation value to read
     * @return the contained {@link AnnotationMirror}, or {@code null} if the value does not contain
     *         an annotation
     */
    public static AnnotationMirror annotationAnnotationValue(AnnotationValue value) {
        return value.accept(new AnnotationMirrorVisitor(), null);
    }

}
