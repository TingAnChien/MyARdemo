#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
# define PI           3.14159265358979323846

class Model	{
	public:
		float size;
		Mat modelPts;
		Mat face;
		Mat head;
		Mat body;
		Mat legLeft;
		Mat legRight;
		Mat handLeft;
		Mat handRight;
		Mat handLeftWalk;
		Mat handRightWalk;
		int hand_zdir;
		int leg_ydir;
		float speed;
		float hand_up;
		float hand_down;
		float leg_front;
		float leg_end;
		float leg_range;
		float hand_range;
		bool ChangeHand;
		bool Build;

		Model(double para = 50);

		~Model(){};
		
		void goBack();
		void goFoward();
		void goLeft();
		void goRight();
		void goUp();
		void goDown();
		void moveLeg(int c, int s);
		void moveHand(int c, int s);
		void move(int c,int s);
		void turn(double angRad);
		void rotate(Mat part, Point2f src_center, double angRad);
		Point2f rotate2d(const cv::Point2f& inPoint, const double& angRad);
		Point2f rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad);
		int getOrientation();
		Point3f getCenter();
		void build();
		void updateModel();
		void goTo(Point3f target, int orientation);

	};

