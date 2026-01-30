package frc.robot.subsystems.vision;

import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.typestate.AltMethod;
import frc.robot.util.typestate.OptionalField;
import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

/** Constants for a camera */
@NullMarked
public class CameraConstants {

    public final String name;
    public final int height;
    public final int width;
    public final Rotation2d horizontalFieldOfView;
    public final Frequency simFps;
    public final Time simLatency;
    public final Time simLatencyStdDev;
    public final double calibrationErrorMean;
    public final double calibrationErrorStdDev;
    public final Transform3d robotToCamera;
    public final double translationError;
    public final double rotationError;
    public final double singleTagError;
    public final boolean isTurret;

    /** Constants for a camera */
    @TypeStateBuilder("CameraConstantsBuilder")
    public CameraConstants(@RequiredField String name, @RequiredField int height,
        @RequiredField int width,
        @RequiredField(alt = @AltMethod(type = double.class, parameter_name = "degrees",
            value = "edu.wpi.first.math.geometry.Rotation2d.fromDegrees(degrees)")) Rotation2d horizontalFieldOfView,
        @RequiredField(alt = @AltMethod(type = double.class, parameter_name = "hz",
            value = "edu.wpi.first.units.Units.Hertz.of(hz)")) Frequency simFps,
        @RequiredField(alt = @AltMethod(type = double.class, parameter_name = "seconds",
            value = "edu.wpi.first.units.Units.Seconds.of(seconds)")) Time simLatency,
        @RequiredField(alt = @AltMethod(type = double.class, parameter_name = "seconds",
            value = "edu.wpi.first.units.Units.Seconds.of(seconds)")) Time simLatencyStdDev,
        @RequiredField double calibrationErrorMean, @RequiredField double calibrationErrorStdDev,
        @RequiredField Transform3d robotToCamera, @OptionalField("0.3") double translationError,
        @OptionalField("edu.wpi.first.math.util.Units.degreesToRadians(3)") double rotationError,
        @OptionalField("0.2") double singleTagError, @OptionalField("false") boolean isTurret) {
        this.name = name;
        this.height = height;
        this.width = width;
        this.horizontalFieldOfView = horizontalFieldOfView;
        this.simFps = simFps;
        this.simLatency = simLatency;
        this.simLatencyStdDev = simLatencyStdDev;
        this.calibrationErrorMean = calibrationErrorMean;
        this.calibrationErrorStdDev = calibrationErrorStdDev;
        this.robotToCamera = robotToCamera;
        this.translationError = translationError;
        this.rotationError = rotationError;
        this.singleTagError = singleTagError;
        this.isTurret = isTurret;
    }



}
