package frc.robot.kobi.systems;

public class KobiShooter extends frc.robot.base.Module {
    public KobiShooter() {
        super("shooter");
        command("shoot", new Command() {
            @Override
            public String execute(String s) throws Exception {
                return "NI";
            }
        });
        command("move", new Command() {
            @Override
            public String execute(String s) throws Exception {
                return "NI";
            }
        });
    }
}
