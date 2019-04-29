package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.XBox360;
import frc.util.TimedRumble;

public class Controls {

    private static Controls instance = null;

    private Joystick driver, operator;
    private TimedRumble driverRumble, operatorRumble;

    public static Controls getInstance() {
        if(instance == null) {
            instance = new Controls();
        }
        return instance;
    }

    private Controls() {   
        driver = new Joystick(0);
        operator = new Joystick(1);
        driverRumble = new TimedRumble(driver, 0, 0);
        operatorRumble = new TimedRumble(operator, 0, 0);
    }

    public void driverRumble(double power, double time) {
        driverRumble.set(driver, time, power);
        driverRumble.start();
    }

    public void operatorRumble(double power, double time) {
        operatorRumble.set(operator, time, power);
        operatorRumble.start();
    }

    public void updateRumble() {
        driverRumble.update();
        operatorRumble.update();
    }

    //MANUAL
    public boolean shouldGoManual() {
        return operator.getRawButton(XBox360.BTN_LEFT_JOYSTICK);
    }

    public boolean shouldGoAuto() {
        return operator.getRawButton(XBox360.BTN_RIGHT_JOYSTICK);
    }

    public boolean shouldLightBeOn() {
        return !driver.getRawButton(XBox360.BTN_START);
    }

    public double stiltsSpeed() {
        return driver.getPOV(0) == 0 ? -0.7 : driver.getPOV(0) == 180 ? 0.7 : 0;
    }

    public boolean noLimit() {
        return driver.getRawButton(XBox360.BTN_L1);
    }

    public boolean shouldStiltsLock() {
        return operator.getRawButton(XBox360.BTN_A);
    }

    public boolean shouldTrimBack() {
        return operator.getPOV(0) == 270;
    }

    public boolean shouldTrimForward() {
        return operator.getPOV(0) == 90;
    }

    public double getIntakePower() {
        if(driver.getRawAxis(Constants.XBox360.AXIS_R2) > 0.5) {
            return 0.8;
        }
        if(driver.getRawAxis(Constants.XBox360.AXIS_L2) > 0.5) {
            return -0.65;
        }
        return 0;
    }

    public double getManipRotationPower() {
        double power = 0.5;
        if (operator.getRawButton(Constants.XBox360.BTN_L1)) {
            power *= -1;
        } else if (!operator.getRawButton(Constants.XBox360.BTN_R1)) {
            power = 0;
        }
        return power;
    }

    public double getExtensionPower() {
        return operator.getRawAxis(Constants.XBox360.AXIS_RIGHT_Y);
    }

    public double getArmRotationPower() {
        double power = 0.45;
        if (operator.getRawAxis(Constants.XBox360.AXIS_L2) > 0.5) {
            power *= -1;
        } else if (!(operator.getRawAxis(Constants.XBox360.AXIS_R2) > 0.5)) {
            power = 0;
        }
        return power;
    }

    public boolean brake() {
        return driver.getRawButton(XBox360.BTN_B);
    }

    public boolean shouldVisionAssist() {
        return driver.getRawButton(XBox360.BTN_A);
    }

    public boolean frontCamera() {
        return driver.getRawButton(XBox360.BTN_X);
    }
    public boolean backCamera() {
        return driver.getRawButton(XBox360.BTN_B);
    }

    public boolean shouldVisionIntake() {
        return driver.getRawButton(XBox360.BTN_Y);
    }

    public boolean hatch() {
        return driver.getRawButton(XBox360.BTN_R1);
    }

    public double getThrottle() {
        return -driver.getRawAxis(Constants.XBox360.AXIS_LEFT_Y);
    }    

    public double getCurvature() {
        return driver.getRawAxis(Constants.XBox360.AXIS_RIGHT_X) * 0.82;
    }

    //PRESETS
    public boolean isForwardPreset() {
        return operator.getRawAxis(XBox360.AXIS_R2) < 0.5;
    }

    public boolean toggleGameObject() {
        return operator.getRawButton(XBox360.BTN_L1);
    }

    public boolean isCargoShip() {
        return operator.getRawButton(XBox360.BTN_BACK);
    }

    public boolean shouldCenterUp() {
        return operator.getRawButton(XBox360.BTN_START);
    }

    public boolean shouldGoToIntakePosition() {
        return operator.getRawButton(XBox360.BTN_A);
    }

    public boolean shouldGoToTopLevelRocket() {
        return operator.getRawButton(XBox360.BTN_Y);
    }

    public boolean shouldGoToMidLevelRocket() {
        return operator.getRawButton(XBox360.BTN_X);
    }

    public boolean shouldGoToLowScoring() {
        return operator.getRawButton(XBox360.BTN_B);
    }

    public boolean overrideArmSoftLimit() {
        return operator.getPOV(0) == 270;
    }

    public boolean shouldAutoHangLvl3() {
        return driver.getRawButton(Constants.XBox360.BTN_Y);
    }

    public boolean shouldAutoHangLvl2() {
        return driver.getRawButton(Constants.XBox360.BTN_X);
    }

    public boolean pidArmParallel() {
        return driver.getRawButton(XBox360.BTN_BACK);
    }

    public boolean shouldAutoHang() {
        return operator.getPOV(0) == 180 && driver.getPOV(0) == 180;
    }

    public boolean interruptHang() {
        return driver.getPOV(0) == 90 || operator.getPOV(0) == 0;
    }

    public boolean cycleUpAuto() {
        return operator.getRawButton(XBox360.BTN_Y);
    }

    public boolean cycleDownAuto() {
        return operator.getRawButton(XBox360.BTN_A);
    }
}