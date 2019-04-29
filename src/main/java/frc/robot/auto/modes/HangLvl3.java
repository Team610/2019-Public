package frc.robot.auto.modes;

import java.util.Arrays;

import frc.robot.Constants;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.DeadReckonArmRotationAction;
import frc.robot.auto.actions.HangAction;
import frc.robot.auto.actions.HangDriveAction;
import frc.robot.auto.actions.ResetAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.TimeDrivedAction;
import frc.robot.subsystems.AFrameRGBs;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.AFrameRGBs.RGBState;
import frc.robot.subsystems.DriveTrain.DriveState;
import frc.robot.subsystems.SubsystemManager.TotalState;

/**
 * AutoModeTest
 */
public class HangLvl3 extends AutoBase {

    public static boolean errored = false;

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                Arrays.asList(
                    new DeadReckonArmRotationAction(true, -298),
                    new TimeDrivedAction(0.35, 0.1),
                    new HangAction(Constants.Stilts.MAX_SAFE_EXTENSION),
                    new HangDriveAction(0.15, 0.8),
                    new ResetAction(Constants.Stilts.MAX_SAFE_EXTENSION),
                    new Action(){
                    
                        @Override
                        public void update() {
                            
                        }
                    
                        @Override
                        public void start() {

                        }
                    
                        @Override
                        public boolean isFinished() {
                            return true;
                        }
                    
                        @Override
                        public void done() {
                            SubsystemManager.getInstance().setState(TotalState.MANUAL);
                            DriveTrain.getInstance().setState(DriveState.OPEN_LOOP);
                            AFrameRGBs.getInstance().setColour(0,1,0);
                            AFrameRGBs.getInstance().setState(RGBState.PULSATING);
                        }
                    }
                )
            )
        );
    }

    

}