package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;;

@SuppressWarnings("unused")
public class CameraSystem extends SubsystemBase {
    private static final String SERVER_NAME = "CameraServer";
    private static final String WIDGET_NAME = "Driver Camera";
    private ShuffleboardTab driverTab;
    private HttpCamera limelightStream;
    private MjpegServer cameraServer;
    @SuppressWarnings("unused")
    private GenericEntry nameEntry;
    
    public CameraSystem(ShuffleboardTab driverTab) {
        this.driverTab = driverTab;

        nameEntry = this.driverTab.add("Camera", CameraConstants.kLimelightName)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .getEntry();
        
        cameraServer = CameraServer.addSwitchedCamera(SERVER_NAME);
        limelightStream = new HttpCamera(CameraConstants.kLimelightName, "http://10.35.27.11:5800/stream.mjpg");
        this.driverTab.add(WIDGET_NAME, cameraServer.getSource())
        .withWidget(BuiltInWidgets.kCameraStream)
        .withSize(6, 6);
        cameraServer.setSource(limelightStream);
    }
    
}
