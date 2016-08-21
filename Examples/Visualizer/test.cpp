#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <mutex>

class Map{

	cv::Point3f GetColor(const float& error, const float& maxError)
	{
		//std::cerr<<error<<" / "<<maxError<<std::endl;

		if(error > maxError)
			return cv::Point3f(1,0,0);

		cv::Vec3f startColor(0,1,0); // RGB (G)
		cv::Vec3f midColor (0,0,1);  // RGB (B)
		cv::Vec3f endColor (1,0,0); // RGB  (R)

		cv::Vec3f c1 = startColor + (midColor - startColor)  *  (maxError - error)/ maxError;
		cv::Vec3f c2 = midColor   + (endColor - midColor  )  *  error/ maxError;

		float alpha = error/maxError;

		c1 = startColor * (1-alpha);
		c2 = endColor *   ( alpha );

		cv::Vec3f c = c1+c2;
/*
		if(error < maxError/2)
			c = c1;
		else
			c = c2;

*/
		return cv::Point3f(c[0],c[1],c[2]);

	}

public:

	typedef std::vector< cv::Point3f > PointCloud;
	std::vector< PointCloud > pointClouds;
	std::vector < cv::Point3f > colors;

	std::mutex mapMutex;

	void addPointCloud(const PointCloud& cloud, const cv::Point3f& color)
	{
		std::unique_lock<std::mutex> lock(mapMutex);
		pointClouds.push_back(cloud);
		colors.push_back(color);
	}
	void showPoints(const bool& showLines, const int& subSample, const int& pointSize, const bool& showDiff, const float& maxError,  const bool& menuShowNormals,
					const bool& menuShowSmoothness)
	{
		std::unique_lock<std::mutex> lock(mapMutex);

		std::vector<bool> showClouds;
		showClouds.push_back(menuShowNormals);
		showClouds.push_back(menuShowSmoothness);

		if(showDiff)
		{
			glPointSize(pointSize);
			for(size_t i=0; i < pointClouds[0].size(); i+=subSample)
			{
				cv::Point3f diff = pointClouds[0][i]-pointClouds[1][i];
				float error = std::sqrt(diff.x*diff.x + diff.y*diff.y + diff.z* diff.z);
				cv::Point3f color = GetColor(error, maxError);
				glColor3f(color.x,color.y,color.z);
				glBegin(GL_POINTS);
				glVertex3f(pointClouds[0][i].x,pointClouds[0][i].y,pointClouds[0][i].z);
				glEnd();
			}

		}
		else
		{
			for(size_t i=0; i< colors.size(); i++)
			{
				if(!showClouds[i]) continue;
				glPointSize(pointSize);
				glBegin(GL_POINTS);
				glColor3f(colors[i].x,colors[i].y,colors[i].z);

				for(int j=0 ; j< pointClouds[i].size(); j+=subSample)
				{
					glVertex3f(pointClouds[i][j].x,pointClouds[i][j].y,pointClouds[i][j].z);
				}
				glEnd();
			}
			if(showLines or showDiff)
			{
				glLineWidth(0.5);
				glColor3f(0.5, 0.5, 0.5);
				glBegin(GL_LINES);
				for(size_t i=0; i< pointClouds[0].size(); i+=subSample)
				{
					const auto& p1 = pointClouds[0][i];
					const auto& p2 = pointClouds[1][i];
					glVertex3f(p1.x, p1.y, p1.z);
					glVertex3f(p2.x, p2.y, p2.z);
				}
				glEnd();
			}
		}
	}
};

struct Viewer
{
	Map* mapPtr;
	Viewer(Map* map): mapPtr(map){}
	void Run()
	{
/*
		float KeyFrameSize = 0.1;
		float KeyFrameLineWidth = 1;
		float GraphLineWidth = 1;
		float PointSize = 2;
		float CameraSize = 0.15;
		float CameraLineWidth = 2;
		*/
		float ViewpointX = 0;
		float ViewpointY = 0;
		float ViewpointZ = -2;
		float ViewpointF = 2000;

		pangolin::CreateWindowAndBind("PointCloudViewer",1024,768);
		glEnable(GL_DEPTH_TEST);

		// Issue specific OpenGl we might need
		glEnable (GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


		pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
		pangolin::Var<bool> menuShowLines("menu.Show Lines",false,true);
		pangolin::Var<bool> menuShowDiff("menu.Show Difference",false,true);
		pangolin::Var<bool> menuShowNormals("menu.Normal Cloud",false,true);
		pangolin::Var<bool> menuShowSmoothness("menu.Smooth Cloud",false,true);
		pangolin::Var<int> menuSubSample("menu.Subsample",1,1,10);
		pangolin::Var<int> menuPointSize("menu.Point Size",1,1,10);
		pangolin::Var<float> menuMaxError("menu.Max Error",.5,0,1);
		//pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
		//pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
		//pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
		pangolin::Var<bool> menuReset("menu.Exit",false,false);

		pangolin::OpenGlRenderState s_cam(
		                pangolin::ProjectionMatrix(1024,768,ViewpointF,ViewpointF,512,389,0.1,1000),
		                pangolin::ModelViewLookAt(
		                		ViewpointX,ViewpointY,ViewpointZ,
		                		//0,0,0,
		                		0,0,0,
								0.0,-1.0, 0.0)
		                );

		// Add named OpenGL viewport to window and provide 3D Handler
		pangolin::View& d_cam = pangolin::CreateDisplay()
		.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
		.SetHandler(new pangolin::Handler3D(s_cam));

		pangolin::OpenGlMatrix Twc;
		Twc.SetIdentity();

		while(1)
		{
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			d_cam.Activate(s_cam);
			glClearColor(1.0f,1.0f,1.0f,1.0f);
			mapPtr->showPoints(
					menuShowLines,
					menuSubSample,
					menuPointSize,
					menuShowDiff,
					menuMaxError,
					menuShowNormals,
					menuShowSmoothness
					);
			pangolin::FinishFrame();

			if(menuReset)
			{
				break;
			}
			//cv::waitKey(33);
		}

	}
};

int main(int argc, char** argv)
{

	const char* firstCloud  = argv[1];
	const char* secondCloud = argv[2];
	const char* settingsFile = argv[3];

	cv::FileStorage fsSettings(settingsFile, cv::FileStorage::READ);
	if(!fsSettings.isOpened())
	{
		std::cerr << "Failed to open settings file at: " << settingsFile << std::endl;
		exit(-1);
	}

	float fx = fsSettings["Camera.fx"];
	float fy = fsSettings["Camera.fy"];
	float cx = fsSettings["Camera.cx"];
	float cy = fsSettings["Camera.cy"];

	fsSettings.release();

	cv::FileStorage id1(firstCloud, cv::FileStorage::READ);
	cv::Mat im1;
	id1["id"]>>im1;
	id1.release();

	cv::FileStorage id2(secondCloud, cv::FileStorage::READ);
	cv::Mat im2;
	id2["id"]>>im2;
	id2.release();

	Map m;

	std::vector<cv::Point3f> cloud, cloud2;

	for(float i= 0 ; i< im1.rows ; i++ )
	{
		for(float j= 0 ; j < im1.cols ; j++ )
		{
			cv::Mat x(3,1,CV_32F);
			float depth = 1./im1.at<float>(i,j);
			x.at<float>(0) = depth * (j-cx)/fx;
			x.at<float>(1) = depth * (i-cy)/fy;
			x.at<float>(2) = depth ;

			cloud.push_back(cv::Point3f(x.at<float>(0),x.at<float>(1),x.at<float>(2)));

			float depth2 = 1./im2.at<float>(i,j);
			x.at<float>(0) = depth2 * (j-cx)/fx;
			x.at<float>(1) = depth2 * (i-cy)/fy;
			x.at<float>(2) = depth2 ;

			cloud2.push_back(cv::Point3f(x.at<float>(0),x.at<float>(1),x.at<float>(2)));

		}
	}

	m.addPointCloud(cloud, cv::Point3f(0.1,0.1,0.9)); // Blue -> First Point Cloud
	m.addPointCloud(cloud2, cv::Point3f(0.1,0.9,0.1)); // Green -> Second point Cloud

	Viewer w(&m);
	w.Run();
	return 0;
}
