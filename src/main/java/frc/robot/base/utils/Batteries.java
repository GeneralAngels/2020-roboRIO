package frc.robot.base.utils;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class Batteries extends frc.robot.base.Module {
    public Batteries() {
        super("batteries");
        set("laptop", "-1");
        set("robot", "-1");
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
                return get("laptop") + " " + get("robot");
            }
        });
    }

    public void updateRobot(PowerDistributionPanel pdp) {
        set("robot", String.valueOf(pdp.getTotalPower()));
    }
}
