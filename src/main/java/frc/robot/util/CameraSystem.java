package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShuffleboardConstants;;

@SuppressWarnings("unused")
public class CameraSystem extends SubsystemBase {
        private UsbCamera usbCamera, usbCamera2;
        private MjpegServer mjpegServer1, mjpegServer2;
        private VideoSink server;

        public CameraSystem() {
        }

        public void init() {
                // Start capturing first USB Camera
                usbCamera = CameraServer.startAutomaticCapture();
                usbCamera.setResolution(320, 240);
                usbCamera.setFPS(30);
                usbCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

                // Start capturing second USB Camera
                usbCamera2 = CameraServer.startAutomaticCapture();
                usbCamera2.setResolution(320, 240);
                usbCamera2.setFPS(30);
                usbCamera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

                // Start capturing from the Limelight camera
                HttpCamera limelightCamera = new HttpCamera(CameraConstants.kLimelightName,"http://roborio-3527-frc.local:1181/stream.mjpg");

                // Start camera server
                mjpegServer1 = new MjpegServer("server_usb Camera 0", 1181);
                mjpegServer1.setSource(usbCamera);

                // Start second sercer
                mjpegServer2 = new MjpegServer("server_Limelight", 1182);
                mjpegServer2.setSource(limelightCamera);

                // test server
                server = CameraServer.getServer();
                server.setSource(usbCamera);

                if (OIConstants.kDebug) {
                        ShuffleboardConstants.kDebugTab.add("Camera Server", mjpegServer1.getSource())
                                        .withWidget(BuiltInWidgets.kCameraStream)
                                        .withPosition(2, 3)
                                        .withSize(2, 2);
                        ShuffleboardConstants.kDebugTab.add("Limelight Stream", mjpegServer2.getSource())
                                        .withWidget(BuiltInWidgets.kCameraStream)
                                        .withPosition(4, 3)
                                        .withSize(2, 2);
                } else {
                        ShuffleboardConstants.kDriverTab.add("Camera Server", mjpegServer1.getSource())
                                        .withWidget(BuiltInWidgets.kCameraStream)
                                        .withPosition(2, 0)
                                        .withSize(2, 2);
                        ShuffleboardConstants.kDriverTab.add("Limelight Stream", mjpegServer2.getSource())
                                        .withWidget(BuiltInWidgets.kCameraStream)
                                        .withPosition(2, 2)
                                        .withSize(2, 2);
                }
        }

}
