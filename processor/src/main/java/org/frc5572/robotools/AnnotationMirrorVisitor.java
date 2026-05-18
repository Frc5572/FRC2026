package org.frc5572.robotools;

import java.util.List;
import javax.lang.model.element.AnnotationMirror;
import javax.lang.model.element.AnnotationValue;
import javax.lang.model.util.SimpleAnnotationValueVisitor8;

/** Find all uses of an annotation in annotation values */
public class AnnotationMirrorVisitor extends SimpleAnnotationValueVisitor8<AnnotationMirror, Void> {

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
