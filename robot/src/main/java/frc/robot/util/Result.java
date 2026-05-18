package frc.robot.util;

public sealed interface Result<T, E> {
    record Ok<T, E>(T value) implements Result<T, E> {
    }
    record Err<T, E>(T value) implements Result<T, E> {
    }
}
