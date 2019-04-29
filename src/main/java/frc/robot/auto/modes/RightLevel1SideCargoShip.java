package frc.robot.auto.modes;

import java.util.Arrays;

import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveToPointFinalHeadingControl;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.RunOnceAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.SetArmFlagAction;
import frc.robot.auto.actions.SetDrivePosition;
import frc.robot.auto.actions.SetHatchIntake;
import frc.robot.auto.actions.SetSubsystemTotalState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveState;
import frc.robot.subsystems.SubsystemManager.TotalState;
import frc.util.Point;

/**
 * RightSideTwoCargoShipHatch
 */
public class RightLevel1SideCargoShip extends AutoBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        //Init auto states
        runAction(
            new SeriesAction(Arrays.asList(
                new SetDrivePosition(new Point(0,0,90)),
                new SetSubsystemTotalState(TotalState.NO_CHANGE),
                new RunOnceAction(){
                    @Override
                    public void runOnce() {
                        DriveTrain.getInstance().setState(DriveState.OFF);
                    }
                }
            ))
        );

        //Score first hatch
        runAction(
            new SeriesAction(Arrays.asList(
                new ParallelAction(Arrays.asList(
                        new SetArmFlagAction("CEN"),
                        new DriveToPointFinalHeadingControl(0.4, 4, 90, -0.2, 0.4, 0.5, 0.05, 2, 25, true, 90)
                )),
                new SetArmFlagAction("CRS_FOR_HAT"),
                new DriveToPointFinalHeadingControl(0.4, 4, 180, -0.2, 0.4, 0.5, 0.05, 2, 25, true, 90),
                // new SetArmFlagAction("CRS_FOR_HAT"),
                // new VisionDriveTillDistance(6.1),
                new SetHatchIntake(true)
                // new SetSubsystemTotalState(TotalState.AUTO),
                // new RunOnceAction(){
                //     @Override
                //     public void runOnce() {
                //         System.out.println("set open loop");
                //         DriveTrain.getInstance().setState(DriveState.OPEN_LOOP);
                //     }
                // }
            ))
        );

        // //Get second hatch
        // runAction(
        //     new SeriesAction(Arrays.asList(
        //         new ParallelAction(Arrays.asList(
        //             new SetArmFlagAction("CEN"),
        //             new DriveToPoint(0, 75 * Units.METERS_PER_INCH, 90, -0.5, 0.2, 0.05, 25, true, 90)
        //         )),
        //         new DriveToPoint(100 * Units.METERS_PER_INCH, 90 * Units.METERS_PER_INCH, 0, 0.2, 0.5, 0.05, 25, true, 90),
        //         new ParallelAction(Arrays.asList(
        //             new SetArmFlagAction("INT_FOR_HAT"),
        //             new DriveToPoint(213 * Units.METERS_PER_INCH, -100 * Units.METERS_PER_INCH, -90, -0.2, 0.5, 0.05, 25, true, 90)
        //         )),
        //         new VisionDriveTillDistance(0.3),
        //         new SetHatchIntake(false)
        //     ))
        // );

        //Score second hatch
    }

    
}