package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class CVClass2 extends OpenCvPipeline{
    Mat output = new Mat();//output mat

    Rect yellowbox;//draws box

    int signal = 0; //1 left, 2 mid, 3 right. 0 means default to left.

    @Override
    public Mat processFrame(Mat input){
        /*
        step 1: get max yellow
        step 2: if there is yellow, get max black. else return 0
        step 3: if there is no black, then return 2. else return 1.
        */
        //NOTE!!!!!
        //H:[0,179]
        //S:[0,255]
        //V:[0,255]
        //.... thanks opencv
        //https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
        //also test using RETR_TREE idk what the difference is. probably just method todo

        List<MatOfPoint> yellowContours = getYellow(input);
        input.copyTo(output);

        //need to draw the rectangles first...

        if(yellowContours.size() > 0){
            double max = 0;
            int maxInd = 0;
            for(int i = 0; i < yellowContours.size(); i++){//loop through all the contours, and find the largest box
                double area = Imgproc.contourArea(yellowContours.get(i));
                if(area > max){
                    max = area;
                    maxInd = i;
                }
            }
            //draw a box of the largest one of single color
            Rect largestRect = Imgproc.boundingRect(yellowContours.get(maxInd));
            yellowbox = largestRect;
            Scalar boxColor = new Scalar(0, 255, 255);//not white... something!!
            Imgproc.rectangle(output, yellowbox, boxColor, 3, 8, 0);//Currently boxed based on rectangle
        }


        if(yellowContours.size() > 0){//in other words, it found yellow
            if (yellowbox.height < 100){
                signal = 2;
            }
            else{
                signal = 3;
            }
        }
        else{
            signal = 1;
        }
        return output;
    }

    //related
    //https://alloyui.com/examples/color-picker/hsv.html
    public List<MatOfPoint> getYellow(Mat input){
        Mat output = new Mat();//output mat
        Mat hsv = new Mat();//after hsv
        Mat blur = new Mat();//after blur
        Mat singleColor = new Mat();//after single color
        Mat hierarchy = new Mat();//after ?

        List<MatOfPoint> contours = new ArrayList<>();

        input.copyTo(output);//don't modify inputs


        //step 1: blur
        Imgproc.GaussianBlur(output, blur, new Size(5,5), 0);//source, destination, size of blur, sigmax (not too important) todo

        //step 2: rgb to hsv
        //or bgr???
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);//source, dest, color swap choice (is an int technically)

        //find contours:
        //using HSV
        double hue = 25;
        double sensitivity = 10;//to adjust for slight variations. Ewww that's a giant sensitivity...

//        Scalar lowBound = new Scalar((hue/2)-sensitivity,100,50);//lower bound... may be wrong???
        Scalar lowBound = new Scalar(hue-sensitivity,100,127);//lower bound
        Scalar hiBound = new Scalar(hue+sensitivity,255,255);//higher bound
        Core.inRange(hsv, lowBound, hiBound, singleColor);//source, low bound, high bound, destination. Gets all color within given range, removes all others
        Imgproc.findContours(singleColor, contours, hierarchy, Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);//source, contours list, hierarchy mat, int for code, and int method

        return contours;
    }

    public int getSignal() {
        return signal;
    }
    public int getHeight(){return yellowbox.height;}
}
