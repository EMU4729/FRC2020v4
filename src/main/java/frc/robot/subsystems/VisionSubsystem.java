/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.*;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.stream.Collectors;
import java.util.Optional;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;

/**
 * Add your docs here.
 */
public class VisionSubsystem implements Subsystem {
  ArrayList<Double> distances;
  RotatedRect rect;
  double aov;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void startCamera() {
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(160, 120);
      camera.setExposureManual(7);
      
      
      

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Auto", 160, 120);

      Mat source = new Mat();
      Mat output = new Mat();
      distances = new ArrayList<Double>();
      while (!Thread.interrupted()) {
          cvSink.grabFrame(source);
          if (!source.empty()) {
              processFrame(source, output);
              outputStream.putFrame(output);
          }
      }
    }).start();
  }

  public void stopCamera() {
    CameraServer.getInstance().removeCamera("USB Camera 0");
    CameraServer.getInstance().removeCamera("Auto");
  }

  private int elementType = Imgproc.CV_SHAPE_ELLIPSE;
  private int kernelSize = 2;
  // Mat element = Imgproc.getStructuringElement(elementType, new Size(2 * kernelSize + 1, 2 * kernelSize + 1), new Point(kernelSize, kernelSize));
  
  private void processFrame(Mat source, Mat output) {
    int aov = 65; //Approx angle of view of the camera that i measured
    Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
    Imgproc.threshold(output, output, 127, 255, Imgproc.THRESH_BINARY);

    List<MatOfPoint> contours = new ArrayList<>();
    Imgproc.findContours(output, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

    List<RotatedRect>rects = contours.stream().map(
        contour -> Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()))
    ).collect(Collectors.toList());
    
    if (rects.size() > 0) {
      rect = rects.stream().max((a, b) -> {
        Double difference = (a.size.width * a.size.height) - (b.size.width * b.size.height);
        return difference.intValue();
      }).get();
      
      Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2BGR);
      Imgproc.ellipse(output, rect, new Scalar(0, 0, 255));
      /*
      
      SmartDashboard.putNumber("Real Width",28);
      SmartDashboard.putNumber("Real Height",10);
      SmartDashboard.putNumber("Distance", 1/Math.tan(rect.size.width/160*aov)*28);
      */
      SmartDashboard.putNumber("angle", rect.angle);
      SmartDashboard.putString("rect", rect.size.toString());
      SmartDashboard.putString("center", rect.center.toString());
      double calibrationWidth = 28.0;      
      int calibrationDistance = 25;
      double f = calibrationDistance*160/calibrationWidth;
      double realDistance = calibrationWidth*f/rect.size.width;

        if (distances.size() > 30) {
          distances.add(realDistance);
          distances.remove(0);
        } else {
          distances.add(realDistance);
        }
        

        ArrayList<Double> sortedDistances = new ArrayList<Double>(distances);
        Collections.sort(sortedDistances);
        double medianDistance = sortedDistances.get(sortedDistances.size()/2);
        if (rect.center.x - rect.size.width/2 < 5 || rect.center.x + rect.size.width/2 > 155) {
          SmartDashboard.putNumber("real distance", 0);
        } else {
          SmartDashboard.putNumber("real distance", medianDistance);
        }

      
    }
    
  }
  public double getAdjustmentAngle() {

    aov = 65;
    if (rect != null) {
      Point center = rect.center;
      double difference = center.x - 80.5;
      double angle = difference/80.5*aov;
      SmartDashboard.putNumber("yaw", angle);
    return angle;
    } else {
      return 0;
    }
  }
}
