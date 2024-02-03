// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private DigitalInput m_proxSensor = new DigitalInput(IntakeConstants.PROX_SENSOR_PORT);
  private CANSparkMax m_motor;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    m_motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_1, MotorType.kBrushless);
  }


  public Boolean getSensor() {
    return !m_proxSensor.get();
  }
  public void startMotor() {
    System.out.println("starting");
    m_motor.set(.5);
  }
  public void stopMotor(boolean interrupted) {
    System.out.print("stopping");
    m_motor.set(0);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
