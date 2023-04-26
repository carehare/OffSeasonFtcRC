package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

class RobotVision {

    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 960; // modify for your camera

    OpenCvWebcam webcam;
//    StoneOrientationAnalysisPipeline pipeline;
    MyOpenCvPipeline pipeline;
    int pipelineOption;

    public RobotVision(int pipelineOption) {

        this.pipelineOption = pipelineOption;
        // Create camera instance
        org.firstinspires.ftc.teamcode.framework.Utilities utilities = org.firstinspires.ftc.teamcode.framework.Utilities.getSharedUtility();
        HardwareMap hardwareMap = utilities.getSharedUtility().hardwareMap;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "WebcamMain"); // put your camera"s name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Open async and start streaming inside opened callback
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                checkWebcamCapability(webcam);

                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                if (pipelineOption == 0) {
                    pipeline = new StoneOrientationAnalysisPipeline();
                } else {
                    pipeline = new ConeDetectionPipeline();
                }
                webcam.setPipeline(pipeline);

                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }

            @Override
            public void onError( int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }

    public void checkWebcamCapability(OpenCvWebcam webcam) {
        ExposureControl exposureControl = webcam.getExposureControl();
        Log.v("webcam control", String.format("Exposure control supported?: %b", exposureControl.isExposureSupported()));

        GainControl gainControl = webcam.getGainControl();
        Log.v("webcam control", String.format("Gain control: %d", gainControl.getGain()));
        Log.v("webcam control", String.format("Gain control range: %d - %d", gainControl.getMinGain(), gainControl.getMaxGain()));

        WhiteBalanceControl whiteBalanceControl = webcam.getWhiteBalanceControl();
        Log.v("webcam control", String.format("WhiteBalance control temperature: %d", whiteBalanceControl.getWhiteBalanceTemperature()));
        Log.v("webcam control", String.format("WhiteBalance control range: %d - %d", whiteBalanceControl.getMinWhiteBalanceTemperature(), whiteBalanceControl.getMaxWhiteBalanceTemperature()));

        FocusControl focusControl = webcam.getFocusControl();
        Log.v("webcam control", String.format("Focus control mode: %s", focusControl.getMode().toString()));
        Log.v("webcam control", String.format("Focus control: .2%f", focusControl.getFocusLength() ));
        Log.v("webcam control", String.format("Focus control range: .2%f - .2%f", focusControl.getMinFocusLength(), focusControl.getMaxFocusLength()));
    }

    public ArrayList<StoneOrientationAnalysisPipeline.AnalyzedStone> getDetectedStones() {
        if (pipelineOption == 0) {

            return ((StoneOrientationAnalysisPipeline)pipeline).getDetectedStones();
        } else {
            return null;
        }
    }

    public Point getConePosition() {

        if (pipelineOption == 1) {
            return ((ConeDetectionPipeline)pipeline).conePosition;
        }
        return null;
    }

    public int getSampleColor() {

        if (pipelineOption == 1) {
            return ((ConeDetectionPipeline)pipeline).sampleColor;
        }
        return -1;
    }

    public void stopDetection() {
        pipeline.stop();
    }
}
