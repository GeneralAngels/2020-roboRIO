package frc.robot.base.utils;

/**
 * Copyright (c) 2019 General Angels
 * https://github.com/GeneralAngels/RIO20
 */

public class Tuple<T1, T2> {
    private T1 first;
    private T2 second;

    public Tuple() {}

    public Tuple(T1 first, T2 second) {
        setFirst(first);
        setSecond(second);
    }

    public void setFirst(T1 first) {
        this.first = first;
    }

    public void setSecond(T2 second) {
        this.second = second;
    }

    public T1 getFirst() {
        return first;
    }

    public T2 getSecond() {
        return second;
    }
}
