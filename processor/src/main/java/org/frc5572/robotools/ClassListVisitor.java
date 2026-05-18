package org.frc5572.robotools;

import java.util.List;
import javax.lang.model.element.AnnotationValue;
import javax.lang.model.type.TypeMirror;
import javax.lang.model.util.SimpleAnnotationValueVisitor8;

/** Find all uses of a type in annotation values */
public class ClassListVisitor extends SimpleAnnotationValueVisitor8<Void, Void> {

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
