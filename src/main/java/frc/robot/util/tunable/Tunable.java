package frc.robot.util.tunable;

import java.util.EnumSet;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public interface Tunable {

    public static void setupTunable(String name, Object obj, Class<?> clazz, Runnable markDirty) {
        final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        if (Constants.tunable) {
            try {
                for (var item : clazz.getDeclaredFields()) {
                    if (item.getType().equals(double.class)) {
                        var topic = ntInstance.getDoubleTopic(name + "/" + item.getName());
                        var publisher = topic.publish();
                        var value = (double) item.get(obj);
                        publisher.accept(value);
                        ntInstance.addListener(topic, EnumSet.of(Kind.kValueAll), (ev) -> {
                            try {
                                var newValue = ev.valueData.value.getDouble();
                                item.set(obj, newValue);
                                markDirty.run();
                            } catch (IllegalArgumentException | IllegalAccessException e) {
                                e.printStackTrace();
                            }
                        });
                    } else if (item.getType().equals(boolean.class)) {
                        var topic = ntInstance.getBooleanTopic(name + "/" + item.getName());
                        var publisher = topic.publish();
                        var value = (boolean) item.get(obj);
                        publisher.accept(value);
                        ntInstance.addListener(topic, EnumSet.of(Kind.kValueAll), (ev) -> {
                            try {
                                var newValue = ev.valueData.value.getBoolean();
                                item.set(obj, newValue);
                                markDirty.run();
                            } catch (IllegalArgumentException | IllegalAccessException e) {
                                e.printStackTrace();
                            }
                        });
                    } else if (item.getType().isAssignableFrom(String.class)) {
                        var topic = ntInstance.getStringTopic(name + "/" + item.getName());
                        var publisher = topic.publish();
                        var value = (String) item.get(obj);
                        publisher.accept(value);
                        ntInstance.addListener(topic, EnumSet.of(Kind.kValueAll), (ev) -> {
                            try {
                                var newValue = ev.valueData.value.getString();
                                item.set(obj, newValue);
                                markDirty.run();
                            } catch (IllegalArgumentException | IllegalAccessException e) {
                                e.printStackTrace();
                            }
                        });
                    } else if (Rotation2d.class.isAssignableFrom(item.getType())) {
                        // will do this later
                    } else if (Enum.class.isAssignableFrom(item.getType())) {
                        // will do this later
                    } else if (Tunable.class.isAssignableFrom(item.getType())) {
                        setupTunable(name + "/" + item.getName(), item.get(obj), item.getType(),
                            markDirty);
                    }
                }
            } catch (IllegalArgumentException | IllegalAccessException e) {
                e.printStackTrace();
            }
        }
    }

}
