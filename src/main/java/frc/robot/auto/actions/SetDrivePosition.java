package frc.robot.auto.actions;

import frc.robot.subsystems.RobotStateEstimator;
import frc.util.Point;

/**
 * SetDrivePosition
 */
public class SetDrivePosition extends RunOnceAction {

    private Point pose;

    /** 
    In degrees
    */
    public SetDrivePosition(Point pose) {
        this.pose = pose;
    }

    @Override
    public void runOnce() {
        RobotStateEstimator.getInstance().zeroSensors(pose);
    }

    
}