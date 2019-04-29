package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto.actions.Action;

/**
 * AutoBase
 */
public abstract class AutoBase {
    protected double updateRate = 1.0 / 50.0;
    protected boolean active = false;

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        active = true;

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }

        done();
    }

    public void done() {
        DriverStation.reportWarning("Auto mode done", false);
    }

    public void stop() {
        active = false;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException {
        action.start();

        while (isActive() && !action.isFinished()) {
            action.update();
            long waitTime = (long) (updateRate * 1000.0);

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }

}