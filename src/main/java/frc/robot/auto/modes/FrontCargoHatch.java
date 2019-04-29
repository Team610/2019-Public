package frc.robot.auto.modes;

import java.util.Arrays;

import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveToPoint;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.SetArmFlagAction;
import frc.robot.auto.actions.SetDrivePosition;
import frc.robot.auto.actions.SetSubsystemTotalState;
import frc.robot.subsystems.SubsystemManager.TotalState;
import frc.util.Point;

public class FrontCargoHatch extends AutoBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetDrivePosition(new Point(0,0,0)));

        //Front cargo
        runAction(
            new SeriesAction(
                Arrays.asList(
                    new SetSubsystemTotalState(TotalState.NO_CHANGE),
                    new SetArmFlagAction("CRS_FOR_HAT"),
                    new DriveToPoint(2, 0, 0, -0.5, 0.5, 0.3, 0.02, 25, true, 90)
                )
            )
        );
    }

}