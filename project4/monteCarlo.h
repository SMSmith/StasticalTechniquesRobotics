#include <iostream>
#include <fstream>
#include <string>
#include <Eigen>
#include <random>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
// #include <opencv2/core/eigen.hpp>	

using namespace std;
using namespace Eigen;
using namespace cv;

#define SENSORLENGTH 187
#define ODOMLENGTH 4

class MonteCarlo {
	public:
					MonteCarlo();
		bool		init(string mapFile, int width, int height, int resolution);
		bool		localize(string logFile, int particleCount);
		Mat 		mapImg;
		void		display();

	private:
		int 		cmPerCell;
		int 		mapW;
		int 		mapH;
		double		eps;
		MatrixXd 	map;
		MatrixXd	particles;
		VectorXd	probabilities;
		MatrixXd	odom = MatrixXd::Zero(1,ODOMLENGTH);
		MatrixXd	sensor = MatrixXd::Zero(1,SENSORLENGTH);
		vector<Vector2i> ranges;
		vector<MatrixXd> lookUpTable;

		bool		processMap(string file, int w, int h);
		bool		processLog(string file);
		VectorXd	split(string s, char del, int w);
		float		getProbabilityOfObstacle(const Vector3d &state, float angle, float range);
		void		sensorUpdate(int si, int pc);
		void		sensorUpdateRefined(int si, int pc);
		void		motionUpdate(const Vector3d &delta);
		void		resample();
		void		constructLUT();
		void		loadLUT(string file);
		double		getRange(int y, int x, double t);
		vector<vector<int> > bresenhamLine(int x0, int y0, int x1, int y1);
};

