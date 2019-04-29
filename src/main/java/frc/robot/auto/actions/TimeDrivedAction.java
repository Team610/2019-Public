package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Stilts;
import frc.util.DriveSignal;

/**
 * Action to wait for a given amount of time
 * To use this Action, call runAction(new WaitAction(your_time))
 */
public class TimeDrivedAction implements Action {

    private double timeToWait;
    private double startTime;
    private double speed;
    private DriveTrain dt;

    public TimeDrivedAction(double timeToWait, double speed) {
        this.timeToWait = timeToWait;
        this.speed = speed;
        dt = DriveTrain.getInstance();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= timeToWait;
    }

    @Override
    public void update() {
        SmartDashboard.putString("Action", "Timed Drive Action");
        dt.setSpeed(new DriveSignal(speed, speed));
    }

    @Override
    public void done() {
        dt.setSpeed(new DriveSignal(0, 0));
        Stilts.getInstance().resetTicks();
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        Stilts.getInstance().setQuad(false);
    }
}
