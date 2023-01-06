package robot;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.SwerveMath;
import robot.subsystems.Drivetrain;

import static robot.Robot.oi;

public class DriveModule {

  private Notifier DrivingPID; 
  static final double kP = 1, kD = .1; 
  public void initialize() { }
  double finalAngle, rotMag, rightArc, directMag, steering_adjust, x; 
  double SteerP = -0.025;
  boolean fixRotation, evadeMode = false; 
  double speedThrottle = .25; 
  //ShooterThrottles
  double k = 0.315;
  double e = 270;
  public DriveModule() {
           
  DrivingPID = new Notifier(() ->  {

    if(oi.RSClickDown(1)){evadeMode=true;} else{evadeMode=false;}
        
    if(oi.LSClickDown(1)){speedThrottle = 1;}
    else if(oi.YButtonDown(1)){speedThrottle = .25;} 
    else{speedThrottle = 0.5;}
        
    SmartDashboard.putBoolean("evademode: ", evadeMode); 
    SmartDashboard.putNumber("NavAngle: ", Robot.NavAngle()); 
    if( oi.RightMag(1)>.3){ rotMag = -0.5*oi.RightX(1); }
    else if(oi.RightMag(1) > .7 || oi.DpadUp(1) || oi.DpadDown(1) || oi.DpadLeft(1) || oi.DpadRight(1)){
      if(oi.RightMag(1) > .7) { rightArc = -oi.RightArc(1); }
      else if(oi.DpadUp(1)){ rightArc = 0; } else if(oi.DpadRight(1)){ rightArc = 270; }
      else if(oi.DpadDown(1)){ rightArc = 180; } else if(oi.DpadLeft(1)){ rightArc = 90; }
      try { Robot.swerve.turning.setYaw(rightArc);} catch (Exception e) {}
      rotMag = Robot.swerve.turning.getPIDOutput();
    } else{ rotMag = 0; }
    try { Robot.swerve.turning.setYaw(rightArc + Robot.NavAngle());} catch (Exception e) {}
     if( oi.AButtonDown(1)){ 
       x = oi.LimelightTx(); if(x >= -3 && x <= 3){ steering_adjust = 0; }else{ steering_adjust = SteerP*-x; } 
       finalAngle = Math.toDegrees(Math.atan2(oi.LeftY(1),steering_adjust))-90; directMag = (Math.abs(steering_adjust) + Math.abs(oi.LeftY(1)))/2; 
       oi.LEDOn();
     } 
    //if(oi.LeftMag(1) >= .2){ oi.LEDOff(); finalAngle = Math.toDegrees(Math.atan2(oi.LeftY(1), (-1)*oi.LeftX(1))-90); directMag = speedThrottle*oi.LeftMag(1); }
    if(oi.LeftMag(1) >= .2){ oi.LEDOff(); finalAngle = Math.toDegrees(Math.atan2(oi.LeftY(1), oi.LeftX(1))) - Robot.NavAngle()-90; directMag = speedThrottle*oi.LeftMag(1); }
    else if(oi.RightBumperDown(1)) { oi.LEDOff(); finalAngle = 270; directMag = .05; } else if(oi.LeftBumperDown(1)) { finalAngle = 90; directMag = .05; }
    else if(oi.LeftTrigger(1) > .1) { oi.LEDOff(); finalAngle = 180; directMag = .05; } else if(oi.RightTrigger(1) > .1) {finalAngle = 0; directMag = .05; }
    else { oi.LEDOff(); directMag = 0; }
    
    if(oi.LeftBumperDown(1) || oi.RightBumperDown(1) || oi.RightTrigger(1) > .1 || oi.LeftTrigger(1) > .1 || oi.LeftMag(1) >= 0.2 || oi.RightMag(1) > 0.3 || oi.AButtonDown(1) || oi.DpadUp(1) || oi.DpadDown(1) || oi.DpadLeft(1) || oi.DpadRight(1)) {
      fixRotation = false; 
    } else{fixRotation = true;}
    SwerveMath.ComputeSwerve(finalAngle, directMag, rotMag, fixRotation); 
    
    SmartDashboard.putNumber ("fl", Drivetrain.getModuleNW().getSteeringEncoder());

    if(oi.StartButton(1)) { Robot.nav.reset(); } 
    
  }); 
  
  DrivingPID.startPeriodic(0.05);
  }
}
    
    

