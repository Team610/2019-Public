/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.loops.LoopManager;
import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.HangLvl2;
import frc.robot.auto.modes.HangLvl3;
import frc.robot.auto.modes.RightHabDriveToFrontRightCargo;
import frc.robot.auto.modes.RightLevel1SideCargoShip;
import frc.robot.subsystems.AFrameRGBs;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Stilts;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.DriveTrain.DriveState;
import frc.robot.subsystems.SubsystemManager.TotalState;
import frc.util.SinglePress;
import frc.util.Util;

/**
 * Before every match 1. Level the robot and enable test mode with the arm
 * facing forward level to the floor
 */
public class Robot extends TimedRobot {

  private SubsystemManager sm;
  private LoopManager looper;
  private Controls controls;
  private Stilts stilts;
  private AutoModeExecutor autoExecutor;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    autoExecutor = new AutoModeExecutor();

    // Call all Subsystems getInstance to add them to the Subsystem Manager
    sm = SubsystemManager.getInstance();
    AFrameRGBs.getInstance();
    stilts = Stilts.getInstance();
    Arm.getInstance();
    DriveTrain.getInstance();
    RobotStateEstimator.getInstance();
    Manipulator.getInstance();
    RobotStateEstimator.getInstance();
    Limelight.getInstance();
    sm.setState(TotalState.AUTO);

    controls = Controls.getInstance();

    looper = LoopManager.getLooper();

    // Teleop Auto Modes
    looper.addLoop(new Loop() {

      private AutoModeExecutor autoHang = new AutoModeExecutor();

      @Override
      public void onFirstStart(double time) {
        autoHang.setAutoMode(new HangLvl3());
      }

      @Override
      public void onEnable(double time) {
        
      }

      @Override
      public void onDisable(double time) {

      }

      @Override
      public void enabledLoop(double time) {
        if (HangLvl3.errored && sm.getState() == TotalState.HANG) {
          stilts.setQuad(true);
          stilts.set(-0.1);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
          }
          stilts.set(0.1);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
          }
          stilts.set(-0.1);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
          }
          stilts.set(0.1);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
          }
          stilts.set(0);
        }
        if (controls.interruptHang()) {
          HangLvl3.errored = true;
          autoHang.stop();
          autoHang.setAutoMode(new HangLvl3());
          sm.setState(TotalState.MANUAL);
          DriverStation.reportWarning("Attempted to stop hang", false);
          Stilts.getInstance().set(0);
          Arm.getInstance().setRotationMotor(0);
        } else if (controls.shouldAutoHangLvl2()) {
          autoHang.setAutoMode(new HangLvl2());
        } else if (controls.shouldAutoHangLvl3()) {
          autoHang.setAutoMode(new HangLvl3());
        } else if (controls.shouldAutoHang()) {
          HangLvl3.errored = false;
          DriverStation.reportWarning("Started auto HANG", false);
          sm.setState(TotalState.HANG);
          autoHang.start();
        }
      }

      @Override
      public void disabledLoop(double time) {

      }

      @Override
      public void allLoop(double time) {

      }

    });

    sm.zeroSensors();
    looper.start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    looper.callOnDisable();
    if (autoExecutor != null) {
      autoExecutor.stop();
    }
  }

  private SinglePress autoUp = new SinglePress();
  private SinglePress autoDown = new SinglePress();
  private List<AutoBase> autos = new ArrayList<>(Arrays.asList(null, new RightHabDriveToFrontRightCargo(), new RightLevel1SideCargoShip()));
  private int autoIndex = 0;

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    autoUp.update(controls.cycleUpAuto());
    autoDown.update(controls.cycleDownAuto());
    if (autoUp.getState()) {
      if (autoIndex == autos.size() - 1) {
        autoIndex = 0;
      } else {
        autoIndex++;
      }
    }
    if (autoDown.getState()) {
      if (autoIndex == 0) {
        autoIndex = autos.size() - 1;
      } else {
        autoIndex--;
      }
    }
    SmartDashboard.putNumber("Auto Value", autoIndex);
    if(autos.get(autoIndex) == null) {
      SmartDashboard.putString("Selected Auto", "No Auto (NULL)");
    } else {
      SmartDashboard.putString("Selected Auto", autos.get(autoIndex).getClass().getName());
    }
    if(autoExecutor != null) {
      autoExecutor.setAutoMode(autos.get(autoIndex));
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    looper.callOnEnable();

    //Start auto
    autoExecutor.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    //Exit auto if driver overrides
    if (autoExecutor != null && (Math.abs(Util.deadband(controls.getThrottle(), 0.5)) > 0
        || Math.abs(Util.deadband(controls.getCurvature(), 0.5)) > 0)) {
      System.out.println("Im trying to stop auto");
      sm.setState(TotalState.AUTO);
      Limelight.getInstance().setVisionModeState(false);
      Limelight.getInstance().setTrackingMode(false);
      DriveTrain.getInstance().setState(DriveState.OPEN_LOOP);
      autoExecutor.stop();
      autoExecutor = null;
    }
  }

  @Override
  public void teleopInit() {
    looper.callOnEnable();
    if (autoExecutor != null) {
      autoExecutor.stop();
    }
    sm.setState(TotalState.AUTO);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    looper.callOnEnable();
    sm.setState(TotalState.MANUAL);
  }

  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Arm", Arm.getInstance().getTickRotation());
  }

}
