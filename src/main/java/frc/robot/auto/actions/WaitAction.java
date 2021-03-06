package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time
 * To use this Action, call runAction(new WaitAction(your_time))
 */
public class WaitAction implements Action {

    private double timeToWait;
    private double startTime;

    public WaitAction(double timeToWait) {
        this.timeToWait = timeToWait;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= timeToWait;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }
}
