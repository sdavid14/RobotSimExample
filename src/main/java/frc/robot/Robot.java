// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
//import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sim.PhysicsSim;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  private final int ELEVATOR_MASTER = 2;
  private  boolean simDriveTrain = false;
  private  boolean simElevator = true;

  // Create our gyro object like we would on a real robot.
private AnalogGyro m_gyro = new AnalogGyro(1);

// Create the simulated gyro object, used for setting the gyro
// angle. Like EncoderSim, this does not need to be commented out
// when deploying code to the roboRIO.
private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  private Encoder m_leftEncoder = new Encoder(2, 3);
  private Encoder m_rightEncoder = new Encoder(4, 5);

  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder); 

  	/* Hardware */
	WPI_TalonFX _talon = new WPI_TalonFX(ELEVATOR_MASTER, "elevator_master"); 

	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

	/** save the last Point Of View / D-pad value */
	int _pov = -1;

//  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_stick = new Joystick(0);
  private final Joystick _joy = new Joystick(1);


  @Override
  public void robotInit() {
    if (simDriveTrain) {
      // We need to invert one side of the drivetrain so that positive voltages
      // result in both sides moving forward. Depending on how your robot's
      // gearbox is constructed, you might have to invert the left side instead.
      m_rightMotor.setInverted(true);
    }

    SmartDashboard.putNumber("MotorSetpointRotations", 0);

    if (simElevator) {
      // Initiallize talon
      initTalon();
    }
  }

  @Override
  public void teleopPeriodic() {
    if (simDriveTrain) {
      // Drive with arcade drive.
      // That means that the Y axis drives forward
      // and backward, and the X turns left and right.
      //m_robotDrive.arcadeDrive(-m_stick.getY(), m_stick.getX());
    }

    if (simElevator) {
      doElevOperatorControl();
    }
  }

  //DifferentialDrivetrainSim m_driveSim;

  private void doElevOperatorControl() {
/* Get gamepad axis - forward stick is positive */
double leftYstick = -1.0 * m_stick.getY(); 
double rghtYstick = -1.0 * m_stick.getX(); 
if (Math.abs(leftYstick) < 0.10) { leftYstick = 0; } /* deadband 10% */
if (Math.abs(rghtYstick) < 0.10) { rghtYstick = 0; } /* deadband 10% */
  DecimalFormat df = new DecimalFormat("0.00");

/* Get current Talon FX motor output */
double motorOutput = _talon.getMotorOutputPercent();

/**
 * Perform Motion Magic when Button 1 is held, else run Percent Output, which can
 * be used to confirm hardware setup.
 */
if (m_stick.getRawButton(1)) {
  /* Motion Magic */

  /* 2048 ticks/rev * 10 Rotations in either direction */
//  double targetPos = rghtYstick * 2048 * 10.0;
  double targetPos = SmartDashboard.getNumber("MotorSetpointRotations",0) * 2048;
    //SmartDashboard.putNumber("MotorSetpoint", 100);
  _talon.set(TalonFXControlMode.MotionMagic, targetPos);

/* Prepare line to print */
_sb.append("\tOut%:");
_sb.append(df.format(motorOutput));
_sb.append("\tVel:");
_sb.append(df.format(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx)));

  /* Append more signals to print when in speed mode */
  _sb.append("\terr:");
  _sb.append(df.format(_talon.getClosedLoopError(Constants.kPIDLoopIdx)));
  _sb.append("\ttrg:");
  _sb.append(df.format(targetPos));
} else {
  /* Percent Output */

  _talon.set(TalonFXControlMode.PercentOutput, leftYstick);
}
if (m_stick.getRawButton(2)) {
/* Prepare line to print */
_sb.append("\tRawBtn2");
_sb.append("\tOut%:");
_sb.append(df.format(motorOutput));
_sb.append("\tVel:");
_sb.append(df.format(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx)));
  /* Zero sensor positions */
  _talon.setSelectedSensorPosition(0);
}

int pov = m_stick.getPOV();
if (_pov == pov) {
  /* no change */
} else if (_pov == 180) { // D-Pad down
  /* Decrease smoothing */
  _smoothing--;
  if (_smoothing < 0)
    _smoothing = 0;
  _talon.configMotionSCurveStrength(_smoothing);

  System.out.println("Smoothing is set to: " + _smoothing);
} else if (_pov == 0) { // D-Pad up
  /* Increase smoothing */
  _smoothing++;
  if (_smoothing > 8)
    _smoothing = 8;
  _talon.configMotionSCurveStrength(_smoothing);

  System.out.println("Smoothing is set to: " + _smoothing);
}
_pov = pov; /* save the pov value for next time */

  /* Instrumentation */
  Instrum.Process(_talon, _sb);
  }

  @Override
  public void simulationInit(){
    if (simDriveTrain) {
    // Create the simulation model of our drivetrain.
/*    m_driveSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
      7.29,                    // 7.29:1 gearing reduction.
      7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
      60.0,                    // The mass of the robot is 60 kg.
      Units.inchesToMeters(3), // The robot uses 3" radius wheels.
      0.7112,                  // The track width is 0.7112 meters.

      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
*/
    }


    if (simElevator) {
      // Add talon FX to simulator
      PhysicsSim.getInstance().addTalonFX(_talon, 0.5, 5100);
    }
  }


  private void initTalon() {
    /* Factory default hardware to prevent unexpected behavior */
		_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		_talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(false);
		_talon.setInverted(false);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _talon.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		_talon.configMotionAcceleration(6000, Constants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  private static int i=0;
  @Override
  public void simulationPeriodic(){

/*    if (simDriveTrain) {
        // Set the inputs to the system. Note that we need to convert
      // the [-1, 1] PWM signal to voltage by multiplying it by the
      // robot controller voltage.
      m_driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
      m_rightMotor.get() * RobotController.getInputVoltage());

      // Advance the model by 20 ms. Note that if you are running this
      // subsystem in a separate thread or have changed the nominal timestep
      // of TimedRobot, this value needs to match it.
      m_driveSim.update(0.02);

      // Update all of our sensors.
      m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
      m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
      m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
      m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
      m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

    }
*/

  if (simElevator) {
    PhysicsSim.getInstance().run();
  }
  }
}
