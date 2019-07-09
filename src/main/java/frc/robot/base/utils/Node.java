package frc.robot.base.utils;

public class Node<T> {
    private Node<T> next;
    private T value;

    public Node() {
    }

    public Node(T value) {
        setValue(value);
    }

    public Node(Node<T> next) {
        setNext(next);
    }

    public Node(T value, Node<T> next) {
        setValue(value);
        setNext(next);
    }

    public Node<T> getNext() {
        return this.next;
    }

    public void setNext(Node<T> next) {
        this.next = next;
    }

    public T getValue() {
        return this.value;
    }

    public void setValue(T value) {
        this.value = value;
    }

    public boolean hasNext() {
        return this.next != null;
    }

    public Node<T> getLast() {
        if (hasNext()) {
            return getNext().getLast();
        }
        return this;
    }

    @Override
    public String toString() {
        return getValue().toString();
    }
}
