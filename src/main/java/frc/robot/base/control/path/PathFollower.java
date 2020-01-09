package frc.robot.base.control.path;

public class PathFollower extends frc.robot.base.Module {
    public PathFollower() {
        super("follower");
        addCommand("createpath", new Command() {
            @Override
            public String execute(String s) throws Exception {
                String[] split = s.split(" ");
                if (split.length == 5) {
                    createPath(Double.parseDouble(split[0]), Double.parseDouble(split[1]), Double.parseDouble(split[2]), Double.parseDouble(split[3]), Double.parseDouble(split[3]));
                }
                return "OK";
            }
        });
    }

    public void createPath(double a, double b, double c, double d, double k){

    }
}
