//OpenCV Annotator
//v 1.0.0
//by Aaron Schmitz

#include <opencv2/opencv.hpp>
#include <iostream>
#include <windows.h>
#include <limits.h>
#include <fstream>

//must be global variables for mouse event
int mouseX, mouseY;

class cvAnnotated
{
public:
	//definition of vars
	cv::Mat frame;	//current image being annotated
	std::string directory;	//directory of images
	std::string pattern;	//pattern to ID img files
	std::vector<std::string> imagelist;	//list of images in the directory
	
	const static int imgMax = 3;		//max number of images
	const static int keypointNum = 3;	//number of keypoints to be annotated for each

	int keypoints[imgMax][keypointNum][2];	//3D array: [image][keypoint][x/y]

	int endlength = 4;	//length of image file name (.jpg)

	void getDir()	//get img directory
	{
		std::cout << "Please enter the desired filepath of jpg files: " << std::endl;
		std::cout << "Current working directory is " << getwd() << std::endl;
		std::getline(std::cin, directory);

		//append *.jpg to limit search to image files
		pattern = directory + "*.jpg";

		readDir();
	}

	//annotate images
	void annotate()
	{
		//cycle through directory
		for (int i = 0; i < imagelist.size(); i++)
		{
			frame = cv::imread(imagelist[i]);
			//err check
			if (frame.empty())
			{
				std::cout << "ERROR: frame " << imagelist[i] << " failed to load" << std::endl;
				continue;
			}
			//assign mousePTR to window
			cv::namedWindow("CV Annotation", 1);
			cv::setMouseCallback("CV Annotation", mousePTR);

			cv::imshow("CV Annotation", frame);

			std::cout << "Click to annotate! Press a key to move to the next keypoint!" << std::endl;
			for (int j = 0; j < keypointNum; j++)
			{
				//waits until key is pressed to move to next point
				cv::waitKey(0);

				std::cout << "Keypoint " << j << " Selected!" << std::endl;
				//assign last mouse click to matrix
				keypoints[i][j][0] = mouseX;
				keypoints[i][j][1] = mouseY;
			}
		}
	}

	//output to csv
	void output()
	{
		//loop thru files, different .csv for each image
		std::ofstream myfile;
		for (int i = 0; i < imagelist.size(); i++)
		{
			std::string curfile = imagelist[i].substr(0, imagelist[i].size() - endlength);	//remove .jpg and add .csv
			curfile = curfile + ".csv";
			myfile.open(curfile);
			//loop through keypoint vals for each image
			for (int j = 0; j < keypointNum; j++)
			{
				myfile << keypoints[i][j][0] << ',';
				myfile << keypoints[i][j][1] << ',';
				myfile << '\n';
			}
			myfile.close();
		}
		std::cout << "Keypoints Saved!" << std::endl;
		std::cout << "Output .csv files are in the image directory." << std::endl;
	}

	//handle when a user clicks the mouse
	static void mousePTR(int event, int x, int y, int flags, void* userdata)
	{
		if (event == cv::EVENT_LBUTTONDOWN)
		{
			std::cout << x << " " << y << std::endl;
			mouseX = x;
			mouseY = y;
		}
	}

private:
	//get current working directory for reference
	std::string getwd()
	{
		char buff[MAX_PATH];
		GetModuleFileName(NULL, buff, MAX_PATH);
		std::string::size_type position = std::string(buff).find_last_of( "\\/" );
		return std::string(buff).substr(0, position);
	}
	
	//read directory as list of strings
	void readDir()
	{
		cv::glob(pattern, imagelist, false);
		for (int i = 0; i < imagelist.size(); i++)
		{
			std::cout << "Image list: " << std::endl;
			std::cout << imagelist[i] << std::endl;
		}
	}
};


int main()
{
	cvAnnotated annotation;

	annotation.getDir();
	
	annotation.annotate();

	annotation.output();
}