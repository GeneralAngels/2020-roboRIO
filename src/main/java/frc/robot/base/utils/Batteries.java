package frc.robot.base.utils;

public class Batteries extends frc.robot.base.Module {
    public Batteries() {
        super("batteries");
        command("update", new Command() {
            @Override
            public String execute(String s) throws Exception {
                set("laptop", s);
                return "OK";
            }
        });
        command("percentage", new Command() {
            @Override
            public String execute(String s) throws Exception {
                return get("laptop") + " " + getRobotBattery();
            }
        });
    }

    private String getRobotBattery(){
        return "100";
    }
}
