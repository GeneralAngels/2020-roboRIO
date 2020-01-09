package frc.robot.kobi.systems;

public class KobiShooter extends frc.robot.base.Module {
    public KobiShooter() {
        super("shooter");
        addCommand("shoot", new Command() {
            @Override
            public String execute(String s) throws Exception {
                return "NI";
            }
        });
        addCommand("move", new Command() {
            @Override
            public String execute(String s) throws Exception {
                return "NI";
            }
        });
    }
}
