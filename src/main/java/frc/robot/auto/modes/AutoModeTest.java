package frc.robot.auto.modes;

import java.util.Arrays;

import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.WaitAction;

/**
 * AutoModeTest
 */
public class AutoModeTest extends AutoBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                Arrays.asList(
                    new SeriesAction(
                        Arrays.asList(
                            new WaitAction(5),
                            new WaitAction(5)
                        )
                    ),
                    new ParallelAction(
                        Arrays.asList(
                            new WaitAction(5),
                            new WaitAction(5)
                        )
                    )
                )
            )
        );
    }

    

}