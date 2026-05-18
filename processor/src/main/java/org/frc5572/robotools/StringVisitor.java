package org.frc5572.robotools;

import javax.lang.model.util.SimpleAnnotationValueVisitor8;

/** Find all uses of a string in annotation values */
public class StringVisitor extends SimpleAnnotationValueVisitor8<String, Void> {

    @Override
    public String visitString(String arg0, Void arg1) {
        return arg0;
    }

}
