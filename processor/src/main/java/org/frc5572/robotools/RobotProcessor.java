package org.frc5572.robotools;

import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;
import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.TypeElement;

/**
 * Annotation processor for checks. Used by VS Code.
 */
@SupportedSourceVersion(SourceVersion.RELEASE_11)
public class RobotProcessor extends AbstractProcessor {

    private AnnotationGenerator[] generators = new AnnotationGenerator[] {
        // @formatter:off
        new GenerateEmptyIOGenerator(), 
        new TypeStateBuilderGenerator(),
        // @formatter:on
    };

    /** Initialization function */
    @Override
    public synchronized void init(ProcessingEnvironment processingEnv) {
        super.init(processingEnv);
        for (var gen : generators) {
            gen.init(processingEnv);
        }
    }

    @Override
    public Set<String> getSupportedAnnotationTypes() {
        return Arrays.stream(generators).map(x -> x.getAnnotationQualifiedName())
            .collect(Collectors.toSet());
    }

    /** Process all elements. */
    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        boolean handledAny = false;
        for (TypeElement annotation : annotations) {
            String name = annotation.getQualifiedName().toString();
            for (var gen : generators) {
                if (gen.getAnnotationQualifiedName().equals(name)) {
                    gen.generate(annotation, roundEnv);
                    handledAny = true;
                }
            }
        }
        return handledAny;
    }

}
