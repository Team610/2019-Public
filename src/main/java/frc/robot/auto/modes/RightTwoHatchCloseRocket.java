package frc.robot.auto.modes;

import java.util.Arrays;

import frc.robot.Constants.Units;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveToPoint;
import frc.robot.auto.actions.VisionDriveTillDistance;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.SetArmFlagAction;
import frc.robot.auto.actions.SetDrivePosition;
import frc.robot.auto.actions.SetHatchIntake;
import frc.robot.auto.actions.SetSubsystemTotalState;
import frc.robot.subsystems.SubsystemManager.TotalState;
import frc.util.Point;

/**
 * RightTwoHatchCloseRocket
 */
public class RightTwoHatchCloseRocket extends AutoBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(Arrays.asList(
                new SetDrivePosition(new Point(0,0,0)),
                new SetSubsystemTotalState(TotalState.NO_CHANGE)
            ))
        );

        //Score first hatch
        runAction(
            new SeriesAction(
                Arrays.asList(
                    new ParallelAction(Arrays.asList(
                        new SetArmFlagAction("ROC_FOR_HAT_MID"),
                        new DriveToPoint(75 * Units.METERS_PER_INCH, 125 * Units.METERS_PER_INCH, 61, -0.3, 0.5, 0.3, 0.05, 25, true, 90)
                    )),
                    new VisionDriveTillDistance(0.3),
                    new SetHatchIntake(true)
                )
            )
        );
    }

    
}