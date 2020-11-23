/*
package org.firstinspires.ftc.teamcode.Development.CR.vision.TestOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.intel.realsense.librealsense.DepthFrame;
import com.intel.realsense.librealsense.DeviceList;
import com.intel.realsense.librealsense.DeviceListener;
import com.intel.realsense.librealsense.Extension;
import com.intel.realsense.librealsense.Frame;
import com.intel.realsense.librealsense.FrameSet;
import com.intel.realsense.librealsense.Pipeline;
import com.intel.realsense.librealsense.ProductLine;
import com.intel.realsense.librealsense.RsContext;
import com.intel.realsense.librealsense.StreamType;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.text.DecimalFormat;

//@Disabled
@Config
@TeleOp(name = "CR-IntelRealSenseD435TestOpMode", group = "CR-Vision")
public class IntelRealSenseD435TestOpMode extends LinearOpMode {
    Thread streamingThread = null;
    private volatile double depthValue = 0.0;

    @Override
    public void runOpMode()  {
        RsContext.init(hardwareMap.appContext);

        streamingThread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    stream();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        RsContext rsContext = new RsContext();
        rsContext.setDevicesChangedCallback(new DeviceListener() {
            @Override
            public void onDeviceAttach() {
                streamingThread.start();
            }

            @Override
            public void onDeviceDetach() {
                streamingThread.interrupt();
            }
        });

        DeviceList devices = rsContext.queryDevices(ProductLine.T200);

        telemetry.addLine("Initialized");
        telemetry.addData("RsDevices", rsContext.queryDevices(ProductLine.T200));
        telemetry.addData("depthValue", depthValue);
        telemetry.update();

        waitForStart();
        streamingThread.start();

        while(!isStopRequested()) {
            telemetry.addData("RsDevices", rsContext.getDeviceCount());
            telemetry.addData("depthValue", depthValue);
            telemetry.update();
        }
    }

    //Start streaming and print the distance of the center pixel in the depth frame.
    private void stream() throws Exception {
        Pipeline pipe = new Pipeline();
        pipe.start();
        final DecimalFormat df = new DecimalFormat("#.##");
        while (!streamingThread.isInterrupted()) {
            try (FrameSet frames = pipe.waitForFrames()) {
                try (Frame f = frames.first(StreamType.POSE)) {
                    sleep(1);
//                    DepthFrame depth =  f;
//                    depthValue = depth.getDistance(depth.getWidth() / 2, depth.getHeight() / 2);
                }
                catch (Exception ex) {
                    ex.printStackTrace();
                }
            }
            catch (Exception ex) {
                ex.printStackTrace();
            }
        }
        pipe.stop();
    }
}
*/