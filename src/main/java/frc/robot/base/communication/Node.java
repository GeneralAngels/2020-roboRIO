package frc.robot.base.communication;

import java.util.ArrayList;
import java.util.HashMap;

public class Node {

    protected String id;
    protected HashMap<String, String> variables = new HashMap<>();

    private ArrayList<Node> slaves = new ArrayList<>();
    private HashMap<String, Command> commands = new HashMap<>();

    protected Node(String id) {
        this.id = id;
    }

    protected ArrayList<Node> getSlaves() {
        return new ArrayList<>(slaves);
    }

    protected HashMap<String, Command> getCommands() {
        return new HashMap<>(commands);
    }

    public String getID() {
        return id;
    }

    public Node find(String name) {
        if (name.toLowerCase().equals("master") || name.toLowerCase().equals(id.toLowerCase())) {
            return this;
        } else {
            for (Node node : slaves) {
                Node found = node.find(name);
                if (found != null)
                    return found;
            }
        }
        return null;
    }

    public Tuple<Boolean, String> execute(String command, String parameter) throws Exception {
        Command executable = commands.get(command);
        if (executable != null) {
            return executable.execute(parameter);
        }
        return null;
    }

    protected void set(String name, String value) {
        variables.put(name, value);
    }

    protected String get(String name) {
        if (variables.containsKey(name)) {
            return variables.get(name);
        }
        return null;
    }

    protected Node enslave(Node slave) {
        slaves.add(slave);
        return slave;
    }

    protected Command command(String name, Command command) {
        commands.put(name, command);
        return command;
    }

    public interface Command {
        Tuple<Boolean, String> execute(String parameter) throws Exception;
    }

    public static class Tuple<A, B> {
        private A a;
        private B b;

        public Tuple(A a, B b) {
            this.a = a;
            this.b = b;
        }

        public A getA() {
            return a;
        }

        public B getB() {
            return b;
        }
    }
}
