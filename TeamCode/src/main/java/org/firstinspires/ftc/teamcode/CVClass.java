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
public class CVClass extends OpenCvPipeline{
    public static double satlow = 0;
    public static double sathi = 50;
    public static double vallow = 100;
    public static double valhi = 255;

    List<MatOfPoint> contours = new ArrayList<>();

    Mat output = new Mat();//output mat

    Rect whitebox,yellowbox;//draws box

    int signal = 0; //1 left, 2 mid, 3 right. 0 means default to left.

    @Override
    public Mat processFrame(Mat input){
        /*
        step 1: get max yellow
        step 2: if there is yellow, get max white. else return 0
        step 3: if there is no white, then return 2. else return 1.
        * */

        /*input.copyTo(output);//don't modify inputs

        contours.clear();//clear

        //using HSV
        //https://www.w3schools.com/colors/colors_hsl.asp
        hue = 53;//53 is yellow. Can also do 60ish
        hue = hue * 179/255;//????
        sensitivity = 30;//to adjust for slight variations. Ewww that's a giant sensitivity...

        //step 1: blur
        Imgproc.GaussianBlur(input, blur, new Size(5,5), 0);//source, destination, size of blur, sigmax (not too important) todo

        //step 2: rgb to hsv
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);//source, dest, color swap choice (is an int technically)

        //find contours:
//        Scalar lowBound = new Scalar((hue/2)-sensitivity,100,50);//lower bound... may be wrong???
        Scalar lowBound = new Scalar(hue-sensitivity,100,50);//lower bound
        Scalar hiBound = new Scalar(hue+sensitivity,255,255);//higher bound
        Core.inRange(hsv, lowBound, hiBound, singleColor);//source, low bound, high bound, destination. Gets all color within given range, removes all others
        Imgproc.findContours(singleColor, contours, hierarchy, Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);//source, contours list, hierarchy mat, int for code, and int method
        //draws the contours given the single (bi) colored mat
        */
        //NOTE!!!!!
        //H:[0,179]
        //S:[0,255]
        //V:[0,255]
        //.... thanks opencv
        //https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
        //also test using RETR_TREE idk what the difference is. probably just method todo

        List<MatOfPoint> yellowContours = getYellow(input);
        List<MatOfPoint> whiteContours = getWhite(input);//may make this black?
//        input.copyTo(output);

        if(yellowContours.size() > 0){//in other words, it found yellow
            if(whiteContours.size() > 0){//found white
                //yellow and white
                signal = 2;
            }
            else {
                //just yellow
                signal = 3;
            }
        }
        else{
            //no yellow, signal should be 1
            signal = 1;
        }

        //just for fun, let's draw boxes

//        if(yellowContours.size() > 0){
//            double max = 0;
//            int maxInd = 0;
//            for(int i = 0; i < yellowContours.size(); i++){//loop through all the contours, and find the largest box
//                double area = Imgproc.contourArea(yellowContours.get(i));
//                if(area > max){
//                    max = area;
//                    maxInd = i;
//                }
//            }
//            //draw a box of the largest one of single color
//            Rect largestRect = Imgproc.boundingRect(yellowContours.get(maxInd));
//            yellowbox = largestRect;
//            Scalar boxColor = new Scalar(255, 255, 255);//box should be white?
//            Imgproc.rectangle(output, yellowbox, boxColor, 3, 8, 0);//Currently boxed based on rectangle
//        }

        if(whiteContours.size()> 0){
            double max = 0;
            int maxInd = 0;
            for(int i = 0; i < whiteContours.size(); i++){//loop through all the contours, and find the largest box
                double area = Imgproc.contourArea(whiteContours.get(i));
                if(area > max){
                    max = area;
                    maxInd = i;
                }
            }
            //draw a box of the largest one of single color
            Rect largestRect = Imgproc.boundingRect(whiteContours.get(maxInd));
            whitebox = largestRect;
            Scalar boxColor = new Scalar(255, 255, 255);//box should be white?
            Imgproc.rectangle(output, whitebox, boxColor, 3, 8, 0);//Currently boxed based on rectangle
        }

        return output;
    }
//blog.jcole.us/2017/04/13/wireless-programming-for-ftc-robots
    public List<MatOfPoint> getYellow(Mat input){
        Mat output = new Mat();//output mat
        Mat hsv = new Mat();//after hsv
        Mat blur = new Mat();//after blur
        Mat singleColor = new Mat();//after single color
        Mat hierarchy = new Mat();//after ?

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        input.copyTo(output);//don't modify inputs


        //step 1: blur
        Imgproc.GaussianBlur(output, blur, new Size(5,5), 0);//source, destination, size of blur, sigmax (not too important) todo

        //step 2: rgb to hsv
        //or bgr???
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);//source, dest, color swap choice (is an int technically)

        //find contours:
        //using HSV
        //https://www.w3schools.com/colors/colors_hsl.asp
        double hue = 25;
        double sensitivity = 10;//to adjust for slight variations. Ewww that's a giant sensitivity...

//        Scalar lowBound = new Scalar((hue/2)-sensitivity,100,50);//lower bound... may be wrong???
        Scalar lowBound = new Scalar(hue-sensitivity,100,127);//lower bound
        Scalar hiBound = new Scalar(hue+sensitivity,255,255);//higher bound
        Core.inRange(hsv, lowBound, hiBound, singleColor);//source, low bound, high bound, destination. Gets all color within given range, removes all others
        Imgproc.findContours(singleColor, contours, hierarchy, Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);//source, contours list, hierarchy mat, int for code, and int method

        return contours;
    }

    public List<MatOfPoint> getWhite(Mat input){
//        Mat output = new Mat();//output mat
        Mat hsv = new Mat();//after hsv
        Mat blur = new Mat();//after blur
        Mat singleColor = new Mat();//after single color
        Mat hierarchy = new Mat();//after ?

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        double saturation, value;
        double sensitivity;

        input.copyTo(output);//don't modify inputs

        //using HSV
        //https://www.w3schools.com/colors/colors_hsl.asp
        //https://alloyui.com/examples/color-picker/hsv.html
        //for white, sat and value matter but hue doesn't really.
        saturation = 0;
        value = 255;
        sensitivity = 30;//to adjust for slight variations. Ewww that's a giant sensitivity...

        //step 1: blur
        Imgproc.GaussianBlur(output, blur, new Size(5,5), 0);//source, destination, size of blur, sigmax (not too important) todo

        //step 2: rgb to hsv
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);//source, dest, color swap choice (is an int technically)

        //find contours:
//        Scalar lowBound = new Scalar((hue/2)-sensitivity,100,50);//lower bound... may be wrong???
        Scalar lowBound = new Scalar(0,satlow,vallow);//saturation,value - sensitivity);//lower bound
        Scalar hiBound = new Scalar(20,sathi,valhi);//saturation + sensitivity,value);//higher bound
        Core.inRange(hsv, lowBound, hiBound, singleColor);//source, low bound, high bound, destination. Gets all color within given range, removes all others
        Imgproc.findContours(singleColor, contours, hierarchy, Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);//source, contours list, hierarchy mat, int for code, and int method
        singleColor.copyTo(output);

        return contours;
    }

    public int getSignal() {
        return signal;
    }

}
