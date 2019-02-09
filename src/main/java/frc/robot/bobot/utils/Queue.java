package frc.robot.bobot.utils;

public class Queue<T> {
    private Node<T> head, tail;

    public void insert(T value) {
        Node<T> current = new Node<>(value);
        if (!isEmpty()) {
            this.tail.setNext(current);
        } else {
            this.head = current;
        }
        this.tail = current;
    }

    public T remove() {
        if (!isEmpty()) {
            T value = peek();
            this.head = this.head.getNext();
            if (isEmpty()) this.tail = null;
            return value;
        }
        return null;
    }

    public boolean isEmpty() {
        return this.head == null;
    }

    public T peek() {
        if (!isEmpty()) {
            return this.head.getValue();
        }
        return null;
    }

    @Override
    public String toString() {
        String result = "";
        Node<T> tempHead = head;
        while (tempHead != null) {
            result += tempHead.getValue().toString();
            if (tempHead.hasNext()) result += ">";
            tempHead = tempHead.getNext();
        }
        return result;
    }
}
