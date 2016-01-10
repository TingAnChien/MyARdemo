#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
# define PI           3.14159265358979323846

class Model	{
	public:
		double size;
		Mat modelPts;
		Mat floor;
		Mat head;
		Mat body;
		Mat legLeft;
		Mat legRight;
		Mat handLeft;
		Mat handRight;
		int hand_zdir;
		int leg_ydir;
		float speed;
		float hand_up;
		float hand_down;
		float leg_front;
		float leg_end;
		float leg_range;
		float hand_range;

		Model(double para = 50);

		~Model(){};
		
		//void moveHand();

		void moveLeg(int c, int s);

		void moveHand(int c, int s);

		void move(int c,int s);

		void turn(double angRad);

		void rotate(Mat part, Point2f src_center, double angRad);

		Point2f rotate2d(const cv::Point2f& inPoint, const double& angRad);

		Point2f rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad);

		int getOrientation();

		void build();

		void updateModel();

	};

