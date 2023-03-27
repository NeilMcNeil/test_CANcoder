package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import com.ctre.phoenixpro.hardware.CANcoder;
import edu.wpi.first.hal.HAL;
import com.ctre.phoenix.sensors.CANCoder;


public class Robot extends TimedRobot {
  private final Joystick m_controller = new Joystick(0);
  private final GenericHID second_controller = new GenericHID(1);


  PWMSparkMax r1 = new PWMSparkMax(9);
  PWMSparkMax r2 = new PWMSparkMax(8);
  PWMSparkMax r3 = new PWMSparkMax(7);
  TalonSRX r4 = new TalonSRX(0);
  MotorControllerGroup rightSide = new MotorControllerGroup(r1, r2, r3);

  
  PWMSparkMax l1 = new PWMSparkMax(0);
  PWMSparkMax l2 = new PWMSparkMax(1);
  PWMSparkMax l3 = new PWMSparkMax(2);
  TalonSRX l4 = new TalonSRX(1);
  MotorControllerGroup leftSide = new MotorControllerGroup(l1, l2, l3);

  Encoder encoder = new Encoder(5, 7, false, CounterBase.EncodingType.k4X);
  SendableChooser<Command> m_Chooser = new SendableChooser<>();
  CANCoder _coder = new CANCoder(1);
  int _loopCount = 0;

  private final Timer clock = new Timer();
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftSide,rightSide);

  
  // final double SET_DELTA = 0.1;

  // CANcoder cancoder;


  @Override
  public void robotInit() {
    leftSide.setInverted(true);

    encoder.setSamplesToAverage(5);
    encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
    encoder.setMinRate(1.0);

    // assert HAL.initialize(500, 0);

    // cancoder = new CANcoder(0);
  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    m_robotDrive.stopMotor();
  }

  @Override
  public void autonomousInit() {
    clock.restart();
  }

  @Override
  public void autonomousPeriodic() {
    if (clock.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.8, 0.0, false);
    } else if (clock.get() < 3.5){
      m_robotDrive.arcadeDrive(0.5,0.81, false); // stop robot
    }
  }

  @Override
  public void teleopInit() {}


  @Override
  public void teleopPeriodic() {
    // l4.changeControlMode(ControlMode.Position); //Change control mode of talon, default is PercentVbus (-1.0 to 1.0)
    // l4.setFeedbackDevice(FeedbackDevice.QuadEncoder); //Set the feedback device that is hooked up to the talon
    // l4.setPID(0.5, 0.001, 0.0); //Set the PID constants (p, i, d)
    // l4.enableControl(); //Enable PID control on the talon


    SmartDashboard.putNumber("Encoder Distance", encoder.getDistance());
    SmartDashboard.putNumber("Encoder Rate", encoder.getRate());

    m_robotDrive.arcadeDrive(m_controller.getY(), m_controller.getX());


    if (second_controller.getRawAxis(1) == -1) {
      r4.set(ControlMode.PercentOutput, 1);
    } else if (second_controller.getRawAxis(4) == 0) {
      r4.set(ControlMode.PercentOutput, 0);
    } else {
      r4.set(ControlMode.PercentOutput, 0);
    }

    if (second_controller.getRawAxis(1) == 1) {
      r4.set(ControlMode.PercentOutput, -1);
    } else if (second_controller.getRawAxis(4) == 0) {
      r4.set(ControlMode.PercentOutput, 0);
    } else {
      r4.set(ControlMode.PercentOutput, 0);
    }


    if (second_controller.getRawAxis(2) == 1) {
      l4.set(ControlMode.PercentOutput, 1);
    } else if (second_controller.getRawAxis(2) == 0) {
      l4.set(ControlMode.PercentOutput, 0);
    } else {
      r4.set(ControlMode.PercentOutput, 0);
    }


    if (second_controller.getRawAxis(2) == -1) {
      l4.set(ControlMode.PercentOutput, -1);
    } else if (second_controller.getRawAxis(2) == 0) {
      l4.set(ControlMode.PercentOutput, 0);
    } else {
      r4.set(ControlMode.PercentOutput, 0);
    }

    
 
    // final double firstSet = 0;
    // final double secondSet = 24;
    // var posGetter = cancoder.getPosition();
    // var cfg = cancoder.getConfigurator();

    // cfg.setPosition(firstSet);
    // SmartDashboard.putNumber("First set:", firstSet);
    // System.out.println("First set: " + posGetter.waitForUpdate(1) + " vs " + firstSet);
    // SmartDashboard.putNumber("First set - ", posGetter.getValue());
    // System.out.println(posGetter.getValue() +  firstSet + SET_DELTA);
    // cfg.setPosition(secondSet);
    // System.out.println("First set: " + posGetter.waitForUpdate(1) + " vs " + firstSet);
    // SmartDashboard.putNumber("First set:", posGetter.getValue());
    // System.out.println(posGetter.getValue()+ secondSet+ SET_DELTA);
    if(_loopCount++ > 10)
    {
        _loopCount = 0;
        double degrees = _coder.getPosition();
        System.out.println("CANCoder position is: " + degrees);
    }

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}