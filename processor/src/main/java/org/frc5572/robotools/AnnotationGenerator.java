package org.frc5572.robotools;

import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.element.TypeElement;

public interface AnnotationGenerator {

    public void init(ProcessingEnvironment env);

    /**
     * @return the full canonical name of the annotation type this generator handles e.g.
     *         "frc.robot.util.GenerateEmptyIO"
     */
    public String getAnnotationQualifiedName();

    /**
     * Called from the processor for each annotation type on each round.
     */
    public void generate(TypeElement annotation, RoundEnvironment roundEnv);

}
