package org.frc5572.robotools;

import javax.lang.model.type.TypeMirror;
import javax.lang.model.util.SimpleAnnotationValueVisitor8;

/** Find all uses of a type in annotation values */
public class ClassVisitor extends SimpleAnnotationValueVisitor8<TypeMirror, Void> {

    @Override
    public TypeMirror visitType(TypeMirror arg0, Void arg1) {
        return arg0;
    }

}
