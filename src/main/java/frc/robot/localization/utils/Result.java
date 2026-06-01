package frc.robot.localization.utils;

public sealed interface Result<T, E> permits Result.Ok, Result.Err {
    record Ok<T, E>(T value) implements Result<T, E> {
    }
    record Err<T, E>(E error) implements Result<T, E> {
    }

    static <T, E> Ok<T, E> ok(T value) {
        return new Ok<>(value);
    }

    static <T, E> Err<T, E> err(E error) {
        return new Err<>(error);
    }
}
