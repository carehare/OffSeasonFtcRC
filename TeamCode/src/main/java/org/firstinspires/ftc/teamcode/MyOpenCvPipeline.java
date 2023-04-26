package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

interface Stop {
    void stop();
}

public class MyOpenCvPipeline extends OpenCvPipeline implements Stop {

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }

    @Override
    public void stop() {
        return;
    }
}
