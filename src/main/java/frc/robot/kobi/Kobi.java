package frc.robot.kobi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.base.Bot;
import frc.robot.base.control.path.PathManager;
import frc.robot.base.drive.DifferentialDrive;
import frc.robot.base.rgb.RGB;
import frc.robot.base.utils.auto.Autonomous;
import frc.robot.kobi.systems.KobiDrive;
import frc.robot.kobi.systems.KobiFeeder;
import frc.robot.kobi.systems.KobiShooter;
import org.json.JSONArray;
import org.json.JSONObject;

public class Kobi extends Bot {

    // Joystick
    private Joystick driver;

    // Modules
    private KobiDrive drive;
    private KobiFeeder feeder;
    private KobiShooter shooter;
    private RGB rgb;

    private Autonomous autonomous;

    // PDP
    private PowerDistributionPanel pdp;

    public Kobi() {
        // Commands
        command("get_trajectory", new Command() {
            @Override
            public Tuple<Boolean, String> execute(String s) throws Exception {
                if (PathManager.getTrajectory() != null) {
                    JSONArray array = new JSONArray();
                    for (Trajectory.State state : PathManager.getTrajectory().getStates()) {
                        JSONObject object = new JSONObject();
                        object.put("x", state.poseMeters.getTranslation().getX());
                        object.put("y", state.poseMeters.getTranslation().getY());
                        object.put("angle", state.poseMeters.getRotation().getDegrees());
                        array.put(object);
                    }
                    return new Tuple<>(true, array.toString());
                }
                return new Tuple<>(true, "[]");
            }
        });

        // Joystick
        driver = new Joystick(0);

        // Modules
        autonomous = new Autonomous(this);
        drive = new KobiDrive();
        shooter = new KobiShooter();

        rgb = new RGB();

        // Ah yes, enslaved modules
        enslave(shooter);
        enslave(autonomous);
        enslave(drive);
        enslave(rgb);

        // Resets
        drive.resetTrajectory();
        drive.updateOdometry();
        drive.setMode(DifferentialDrive.Mode.Manual);

        // RGB mode
        rgb.setMode(RGB.Mode.Slide);
    }

    @Override
    public void autonomous() {
        autonomous.loop();
    }

    @Override
    public void teleop() {
//        set("left_velocity", String.valueOf(drive.motorControlLeftVelocity));
//        set("right_velocity", String.valueOf(drive.motorControlRightVelocity));
        set("time", String.valueOf(millis()));
//        log("Enc " + shooter.getPosition());
        shooter.setVelocity(60);
//        shooter.setVelocity((1000.0 / 60.0) * driver.getY());

        //drive.setNoPID(-driver.getY() / 5, driver.getX() / 5);

//        drive
//        drive.loop();
    }
}