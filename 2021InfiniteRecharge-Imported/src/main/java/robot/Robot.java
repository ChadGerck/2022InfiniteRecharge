 package robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.I2C;

public class Robot extends TimedRobot {
  public static Timer myTimer = new Timer();
  public static final OI oi = new OI();
  private static final String 
  FarL = "FarL", Left = "Left", Mid = "Mid", Front = "Front", FarR = "FarR",
  Default = "Default", P2 = "A1.2", P3 = "A1.3", HailMary = "HailMary", Defense = "Defense"; 
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_chosen = new SendableChooser<>();
  public static AHRS nav; 
  public boolean flag = true; 
  static double finalAngle, directMag, steering_adjust, x, rotMag;
  static Boolean fixRotation;
  private Counter m_LIDAR;
  static double SteerP = -0.025;
  final double off  = 10; //offset for sensor. test with tape measure
  @Override public void robotInit() { 
    m_LIDAR = new Counter(0); //plug the lidar into PWM 0
    m_LIDAR.setMaxPeriod(1.00); //set the max period that can be measured
    m_LIDAR.setSemiPeriodMode(true); //Set the counter to period measurement
    m_LIDAR.reset();
    nav = new AHRS(I2C.Port.kMXP); 
    
    nav.reset();
    //CameraServer.startAutomaticCapture();

    m_chooser.setDefaultOption("FarL", FarL); m_chooser.addOption("Left", Left); m_chooser.addOption("Mid", Mid); m_chooser.addOption("Front", Front);  m_chooser.addOption("FarR", FarR);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    m_chosen.setDefaultOption("Default", Default); m_chosen.addOption("PlayerStation", P2);
    m_chosen.addOption("P3", P3); m_chosen.addOption("HailMary", HailMary);
    m_chosen.addOption("Defense", Defense);
    SmartDashboard.putData("the Auto: ", m_chosen);
  }
  @Override public void robotPeriodic() { 
    CommandScheduler.getInstance().run();
    double dist;
    if(m_LIDAR.get() < 1) dist = 0;
    else dist = (m_LIDAR.getPeriod()*1000000.0/10.0) - off; //convert to distance. sensor is high 10 us for every centimeter. 
    SmartDashboard.putNumber("Distance", dist); //put the distance on the dashboard
    
  }
  @Override public void teleopInit() { 
  /*swerve.SetElevatorStatus(); swerve.ConfigElevator();*/
 }
  @Override public void autonomousInit() { 

  }
  public static void LimeAlign(){
  }
  
  @Override public void autonomousPeriodic() {
  }
  @Override public void teleopPeriodic() { }
  @Override public void testPeriodic() {}
  public static double NavAngle() {return NavAngle(0);}
  public static double NavAngle(double add){double angle = Robot.nav.getAngle()+add;
    while(angle>180)angle-=360;while(angle<-180)angle+=360;return angle; 
  }
}
