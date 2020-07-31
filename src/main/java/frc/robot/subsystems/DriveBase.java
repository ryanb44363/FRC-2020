package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase {

    public XboxController controller = new XboxController(0);
    public XboxController controller1 = new XboxController(1);
    
    public static final XboxController.Hand kLeft = XboxController.Hand.kLeft;
    public static final XboxController.Hand kRight = XboxController.Hand.kRight;  

    public double intakeVelocity = 0.0;  // Value to be set after testing.
    public double extakeVelocity = 0.0; // Value to be set after testing.
    public double liftVelocity = 0.0;  // Value to be set after testing.
    public double lowerVelocity = -0.0;// Value to be set after testing.
    public double holdVelocity = 0.0;// Value to be set after testing.
    
    private DigitalInput limitSwitchhigher = new DigitalInput(0);
    private DigitalInput limitSwitchlower = new DigitalInput(1);

    private CANSparkMax leftfrontmotor, leftrearmotor, rightfrontmotor, rightrearmotor;
    private WPI_VictorSPX elevator = new WPI_VictorSPX(12);
    private WPI_TalonSRX winch =  new WPI_TalonSRX(11);

    private WPI_TalonSRX Arm = new WPI_TalonSRX(22);
    private WPI_VictorSPX Mouth = new WPI_VictorSPX(21);

    private AHRS gyro = new AHRS();

    private final double pTurn = 0.005;
    private final double iTurn = 0;
    private final double dTurn = 0;

    private final double pDrive = 0.016;
    private final double iDrive = 0;
    private final double dDrive = 0;

    private PIDController turnController = new PIDController(pTurn, iTurn, dTurn);
    private PIDController driveController = new PIDController(pDrive, iDrive, dDrive);
    private PIDController ballTurnController = new PIDController(pTurn, iTurn, dTurn);
    private PIDController ballDriveController = new PIDController(pDrive, iDrive, dDrive);
    
    private double tx_prev = 0;

    public DriveBase(int leftfrontmotorPort, int leftrearmotorPort, int rightfrontmotorPort, int rightrearmotorPort) {
        this.leftfrontmotor = new CANSparkMax(leftfrontmotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.leftrearmotor = new CANSparkMax(leftrearmotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rightfrontmotor = new CANSparkMax(rightfrontmotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rightrearmotor = new CANSparkMax(rightrearmotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public void initialize() {
        leftfrontmotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftrearmotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightfrontmotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightrearmotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        leftfrontmotor.setOpenLoopRampRate(0.5);
        leftrearmotor.setOpenLoopRampRate(0.5);
        rightfrontmotor.setOpenLoopRampRate(0.5);
        rightrearmotor.setOpenLoopRampRate(0.5);

        leftfrontmotor.getEncoder().setPositionConversionFactor(34.19);
        leftrearmotor.getEncoder().setPositionConversionFactor(34.19);
        rightfrontmotor.getEncoder().setPositionConversionFactor(34.19);
        rightrearmotor.getEncoder().setPositionConversionFactor(34.19);

        turnController.setSetpoint(0);
        turnController.setTolerance(2);

        driveController.setSetpoint(0);
        driveController.setTolerance(5);

        ballTurnController.setSetpoint(0);
        ballTurnController.setTolerance(0.25);

        ballDriveController.setSetpoint(10);
        ballDriveController.setTolerance(0.5);
        
        reset();
    }

    public void reset() {
        leftfrontmotor.getEncoder().setPosition(0);
        leftrearmotor.getEncoder().setPosition(0);
        rightfrontmotor.getEncoder().setPosition(0);
        rightrearmotor.getEncoder().setPosition(0);

        gyro.reset();

        turnController.setSetpoint(0);
        driveController.setSetpoint(0);

        tx_prev = 0;
    }

    public void arcadeDrive(XboxController controller, GenericHID.Hand left, GenericHID.Hand right) {
        double throttle = 0;
        double turn = 0;
        
        rightfrontmotor.setInverted(true);

        if (controller.getY(left) > 0.05 || controller.getY(left) < -0.05) {
            throttle = controller.getY(left);
            throttle = throttle;  //to southpaw or not.  currently not.

        } 

        if (controller.getX(right) > 0.05 || controller.getX(right) < -0.05) {
            turn = controller.getX(right);
            turn = -turn;
        } 

        leftfrontmotor.set(turn + throttle);
        leftrearmotor.set(turn + throttle);

        rightfrontmotor.set(turn - throttle);
        rightrearmotor.set(turn - throttle);
    }
    
    public void angleLockedDrive(XboxController controller, GenericHID.Hand left) {
        double throttle = 0;
        double turn = turnController.calculate(getAngle());

        if (controller.getY(left) > 0.05 || controller.getY(left) < -0.05) {
            throttle = controller.getY(left);
            throttle = -throttle; //to not southpaw
        } else if (controller.getY(left) < 0.05 || controller.getY(left) > -0.05) {
            throttle = 0;
        }
    }
    
    public void climber(XboxController controller1, GenericHID.Hand left, GenericHID.Hand right) {
        System.out.println("Initializing Climber...");
        double raiseSlow = .15;
        double raiseMedium = .30;
        double raiseFast = .45;

        double raiseVelocity = .7;
        double lowerVelocity = -.10;
        // Controls for the Winch, or lifter: 
        if (controller1.getTriggerAxis(left) > 0 || controller1.getTriggerAxis(left) < .25) {
            winch.set(raiseSlow);
        }
        if (controller1.getTriggerAxis(left) > .25 || controller1.getTriggerAxis(left) < .5) {
            winch.set(raiseMedium);
        }
        if (controller1.getTriggerAxis(left) > .5) {
            winch.set(raiseFast);
        }
        else if (controller1.getTriggerAxis(right) > 0) 
            
        { winch.set(lowerVelocity); }
        
        else { winch.set(0); }

        // Controls for the Elevator, or stick hoist:
        if (controller1.getY(left) > 0.1) {
            elevator.set(-raiseVelocity);
        }
        else if (controller1.getY(left) < -0.1) 
            {elevator.set(-lowerVelocity);
            }
        else {
            elevator.set(0);
        } 
    }
    public void arm (XboxController controller1, GenericHID.Hand left, GenericHID.Hand right) {
        double raiseVelocity = .2;
        double lowerVelocity = -.5;
        double intakeVelocity = .5;
        double extakeVelocity = -.75;

        if (controller1.getAButtonPressed()){
            Arm.set(raiseVelocity); 
        }

        if (controller1.getAButtonReleased()){
            Arm.set(0.0);
        }

        if (controller1.getBButtonPressed()){
            Arm.set(lowerVelocity);
        }

        if (controller1.getBButtonReleased()){
            Arm.set(0.0);
        }

        if (controller1.getXButtonPressed()) {
            Mouth.set(intakeVelocity);
        }

        if (controller1.getXButtonReleased()) {
            Mouth.set(0.0);
        }

        if (controller1.getYButtonPressed()) {
            Mouth.set(extakeVelocity); 
        }

        if (controller1.getYButtonReleased()) {
            Mouth.set(0.0);
        }
  }

    public void snapToAngle() {
        double turn = turnController.calculate(getAngle());

        leftfrontmotor.set(turn);
        leftrearmotor.set(turn);

        rightfrontmotor.set(turn);
        rightrearmotor.set(turn);
    }
    
    public void distanceDrive() {
        double throttle = driveController.calculate(getLeftPosition());

        leftfrontmotor.set(-throttle);
        leftrearmotor.set(-throttle);

        rightfrontmotor.set(throttle);
        rightrearmotor.set(throttle);
    }

    public void turnDrive() {
        double throttle = driveController.calculate(getLeftPosition());

        leftfrontmotor.set(throttle);
        leftrearmotor.set(throttle);

        rightfrontmotor.set(throttle);
        rightrearmotor.set(throttle);
    }

    public void setAngle(double angle) {
        turnController.setSetpoint(angle);
    }

    public void setDistance(double degrees) {
        driveController.setSetpoint(degrees);
    }

    public double getLeftPosition() {
        return leftfrontmotor.getEncoder().getPosition();
    }

    public double getRightPosition() {
        return rightfrontmotor.getEncoder().getPosition();
    }

    public double getLeftVelocity() {
        return leftfrontmotor.getEncoder().getVelocity();
    }

    public double getRightVelocity() {
        return rightfrontmotor.getEncoder().getVelocity();
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public boolean turnOnTarget() {
        return turnController.atSetpoint();
    }

    public boolean driveOnTarget() {
        return driveController.atSetpoint();
    }

    public boolean ballTurnOnTarget() {
        return ballTurnController.atSetpoint();
    }

    public boolean ballDriveOnTarget() {
        return ballDriveController.atSetpoint();
    }

    public void dashboard() {
        SmartDashboard.putData("Turn Controller", turnController);
        SmartDashboard.putData("Drive Controller", driveController);
        SmartDashboard.putData("Ball Turn Controller", ballTurnController);
        SmartDashboard.putData("Ball Drive Controller", ballDriveController);

        SmartDashboard.putNumber("Left Position", getLeftPosition());
        SmartDashboard.putNumber("Right Position", getRightPosition());
        SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
        SmartDashboard.putNumber("Right Velocity", getRightVelocity());
    }
}
