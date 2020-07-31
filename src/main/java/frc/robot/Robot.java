package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.actions.DriveAction;
import frc.robot.actions.DriveActionTurn;
import frc.robot.subsystems.DriveBase;


public class Robot extends TimedRobot {
    // Xbox Controller
    public static XboxController controller = new XboxController(0);
    public static XboxController controller1 = new XboxController(1);
    public static final GenericHID.Hand left = GenericHID.Hand.kLeft;
    public static final GenericHID.Hand right = GenericHID.Hand.kRight;

    // Motors
    public CANSparkMax leftfrontmotor;
    public CANSparkMax leftrearmotor;
    public CANSparkMax rightfrontmotor;
    public CANSparkMax rightrearmotor;

    // Drive Base
    public static DriveBase base = new DriveBase(1, 2, 3, 4);

    // Autonomous
    public DriveAction autoBaby = new DriveAction(2);
    public DriveActionTurn autoTwo = new DriveActionTurn(3);

    @Override
    public void robotInit() {
        base.initialize();
    }

	@Override
    public void teleopInit() {
        base.initialize();
        base.reset();   
    }

    @Override
    public void teleopPeriodic() {
        reseter();
        
        drive();
        
        armStrong();
      
        climber();

        dashboard();

    }

    public void armStrong() {
        //System.out.print("armStrong Called");
        //if (controller.getAButtonPressed()){
            base.arm(controller1, left, right);
        //}
    }

    public void climber() {
        base.arm(controller1, left, right);
    }

    public void drive() {
        base.arcadeDrive(controller, left, right);
    }

    public void dashboard() {
        base.dashboard();
    }

    public void reseter() {
    }

    @Override 
    public void autonomousInit() {    
        base.initialize();
        base.reset();
    }

    @Override
    public void autonomousPeriodic() {
        Timer Timer = new Timer();
        Timer.start();

        System.out.println("Timer Started!");
        autoTwo.start();
        Timer.delay(2);
        autoTwo.isComplete();

        autoBaby.start();
        Timer.delay(2);
        autoBaby.isComplete();
        
    }
}