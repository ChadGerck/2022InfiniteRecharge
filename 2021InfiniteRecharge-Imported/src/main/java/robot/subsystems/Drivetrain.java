package robot.subsystems;

//import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import robot.Robot;
import robot.SwerveModule;
import robot.TurnModule;

public class Drivetrain extends SubsystemBase {
  public TurnModule turning;    
  private static final Translation2d m_frontLeftLocation = new Translation2d(0.381, -0.381);
  private static final Translation2d m_frontRightLocation = new Translation2d(0.381, 0.381);
  private static final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private static final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  public static CANCoder abeFL = new CANCoder(0), abeFR = new CANCoder(1),
                         abeBL = new CANCoder(2), abeBR = new CANCoder(3); 
  
  //abeFL = new AnalogPotentiometer(0, 360, 59), abeFR = new AnalogPotentiometer(1, 360, 279), 
  //                            abeBL = new AnalogPotentiometer(2, 360, 402), abeBR = new AnalogPotentiometer(3, 360, -17); 

  static double kSwerveP = .8, kSwerveD = .1; 
  private static SwerveModule 
  moduleFL = new SwerveModule(5, 6, abeFL, kSwerveP, kSwerveD, false), moduleFR = new SwerveModule(1, 2, abeFR, kSwerveP, kSwerveD, false),
  moduleBL = new SwerveModule(3, 4, abeBL, kSwerveP, kSwerveD, false), moduleBR = new SwerveModule(7, 8, abeBR, kSwerveP, kSwerveD, false);
  
  private static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  public static final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0));
  
   public Drivetrain(){
    turning = new TurnModule(); 
  }
  public static void setModule(String loc,double degrees,double power){
    switch(loc){case "FL":moduleFL.set(degrees,power);break; case "FR":moduleFR.set(degrees,power);break;
                case "BL":moduleBL.set(degrees,power);break; case "BR":moduleBR.set(degrees,power);break;
    }
  }public SwerveModule getModuleNW(){ return moduleFL;}
  public  SwerveModule getModuleNE(){ return moduleFR;}
	public  SwerveModule getModuleSW(){ return moduleBL;}
  public  SwerveModule getModuleSE(){ return moduleBR;}
  public static Rotation2d getAngle() { return Rotation2d.fromDegrees(Robot.NavAngle()); }
  public void setAllAngle(double degrees){
    moduleFL.setSteeringDegrees(degrees); moduleFR.setSteeringDegrees(degrees);
    moduleBL.setSteeringDegrees(degrees); moduleBR.setSteeringDegrees(degrees);
  }public void setAllPower(double power){
    moduleFL.setDrivePower(power); moduleFR.setDrivePower(power);
    moduleBL.setDrivePower(power); moduleBR.setDrivePower(power);
  }
  public void setALLBrake(boolean brake){
    moduleFL.setBrakeOn(brake); moduleFR.setBrakeOn(brake);
    moduleBL.setBrakeOn(brake); moduleBR.setBrakeOn(brake);
  }
	// public double getLiftVelocity() { return Elevator.getLiftVelocity(); }
	// public double getLiftPosition() { return Elevator.getLiftPosition(); }
  public void updateDashboard(){ 
    SmartDashboard.putNumber("ODOX", ODOX()); 
    SmartDashboard.putNumber("ODOY", ODOY());
  }
  public static void updateOdometry() {
    m_odometry.update(
        getAngle(),
        moduleFL.getState(), moduleFR.getState(),
        moduleBL.getState(), moduleBR.getState()
    );
  }
  public double ODOX() { return m_odometry.getPoseMeters().getTranslation().getY(); }
  public double ODOY() { return m_odometry.getPoseMeters().getTranslation().getX(); }
  public void OdoReset(){ m_odometry.resetPosition(new Pose2d(new Translation2d(0.0,0.0), Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));}
}

