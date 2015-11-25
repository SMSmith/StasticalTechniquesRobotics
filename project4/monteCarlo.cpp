#include "monteCarlo.h"

MonteCarlo::MonteCarlo() {}

bool MonteCarlo::init(string mapFile, int width, int height, int resolution) {
	cmPerCell = resolution;
	mapW = width;
	mapH = height;
	map.resize(mapH,mapW);

	processMap(mapFile,mapW,mapH);

	mapImg = Mat::zeros(mapH,mapW,CV_8UC3);
	for(int y=0; y<mapH; y++) {
		for(int x=0; x<mapW; x++) {
			Vec3b &color = mapImg.at<Vec3b>(y,x);
			// for(int i=0; i<3; i++) {
				if(map(y,x)>0)
					color[1]=int(50*map(y,x));
			// }
		}
	}

	// cout << map.block(300,0,1,800) << endl;

	// constructLUT();
	loadLUT("lut");

	// vector<vector<int> > k = bresenhamLine(0,0,0,100);
	// for(int i=0; i<k.size(); i++) {
	// 	cout << k[i][0] << "," << k[i][1] << endl;
	// }

	// for(int i=0; i<360; i++) {
	// 	cout << lookUpTable[i](0,0) << endl;
	// }

	// namedWindow("Test Map", WINDOW_AUTOSIZE);
	// imshow("Test Map", mapImg);
	// Mat inv, gray;
	// bitwise_not(mapImg*4,inv);
	// cvtColor(inv, gray, CV_BGR2GRAY);
	// imwrite("data/output/map.png",gray);

	// waitKey(500);

	return true;
}

bool MonteCarlo::localize(string logFile, int particleCount) {
	// Puts odometry entries in odom and sensor entries in sensor
	// These are matrices
	processLog(logFile);
	eps = 1./(particleCount);

	// cout << odom.rows() << " " << odom.cols() << " | " << sensor.rows() << " " << sensor.cols() << endl;

	// Do Localization
	// Initialize a bunch of random particles
	particles = MatrixXd::Random(particleCount,3);

	// cout << particles << endl;
	cout << "random " << endl;
	Matrix3d stateParticle;
	stateParticle << mapW/4*cmPerCell, 0, 0, 0, mapH/2*cmPerCell, 0, 0, 0, M_PI;
	// stateParticle << 400, 0, 0, 0, 200, 0, 0, 0, M_PI/4;
	particles*=stateParticle;
	// cout << particles << endl;
	cout << "split" << endl;
	for(int i=0; i<particles.rows(); i++) {
		particles(i,0) += mapW*cmPerCell/2;
		particles(i,1) += mapH*cmPerCell/2;
		// particles(i,2) -= M_PI;//-(M_PI/4);
	}
	// for(int i=0; i<particles.rows(); i++) {
	// 	particles(i,0) += 4000;
	// 	particles(i,1) += 4000;
	// 	particles(i,2) += M_PI;
	// }
	// cout << particles << endl;
	probabilities = VectorXd::Ones(particleCount)*eps;
	Vector3d baseState = odom.block(0,0,1,3).transpose();
	// cout << probabilities.block(0,0,10,1) << endl;

	double maxTime = max(odom(odom.rows()-1,ODOMLENGTH-1),sensor(sensor.rows()-1,SENSORLENGTH-1));
	cout << maxTime << endl;
	// iterate through time:
	int odomIterator=0;
	int sensorIterator=0;
	int iii=0;
	while(odomIterator<odom.rows() || sensorIterator<sensor.rows() ) {
		// Iterators are not close to end
		if(odomIterator<odom.rows() && sensorIterator<sensor.rows()) {
			// look at each sensor update and parse out probability of where the robot is (somehow)
			if(odom(odomIterator,ODOMLENGTH-1) > sensor(sensorIterator,SENSORLENGTH-1)) {
				// Resample when probabilities start to become unbalanced
				if (probabilities.maxCoeff()/probabilities.minCoeff()>pow(particleCount,5)) {//&&sensorIterator>40) {
					resample();
				}

				cout << "sensor: " << sensor(sensorIterator,SENSORLENGTH-1) << endl;
				sensorUpdateRefined(sensorIterator,particleCount);

				VectorXd::Index maxIndex, minIndex;
				probabilities.maxCoeff(&maxIndex);
				probabilities.minCoeff(&minIndex);
				cout << particles.block(maxIndex,0,1,3) << " " << probabilities(maxIndex) << endl;
				cout << particles.block(minIndex,0,1,3) << " " << probabilities(minIndex) << endl;

				sensorIterator++;
			}
			// look at each motion update and move the proababilities with motion
			else {
				// MOVE ALL OF THE PARTICLES
				Vector3d delta = odom.block(odomIterator,0,1,3).transpose() - baseState;
				if(delta(2)>M_PI) {
					delta(2) = odom(odomIterator,2)-(baseState(2)+2*M_PI);
				}
				if(delta(2)<-M_PI) {
					delta(2) = (odom(odomIterator,2)+2*M_PI)-baseState(2);
				}
				cout << "Motion: " << delta.transpose() << endl;
				cout << "base: " << baseState.transpose() << endl;
				motionUpdate(delta);
				baseState+=delta;

				odomIterator++;
			}
		}
		// only odom left
		else if (odomIterator<odom.rows()) {
			cout << "Last odom: " << odom(odomIterator,ODOMLENGTH-1) << endl;

			Vector3d delta = -(odom.block(odomIterator,0,1,3).transpose() - baseState);
			delta(2) = fmod(delta(2),2*M_PI);
			motionUpdate(delta);
			baseState+=delta;

			odomIterator++;
		}
		// only sensor left
		else {
			cout << "Last sensor: " << sensor(sensorIterator,SENSORLENGTH-1) << endl;

			sensorUpdateRefined(sensorIterator,particleCount);

			sensorIterator++;
		}
		if(iii%1==0)
			display();
		++iii;
	}

	return true;
}

void MonteCarlo::sensorUpdate(int si, int pc) {
	// range (for display)
	ranges.clear();
	// For each particle, find the "probability" that the robot is located there
	VectorXd tempScores = VectorXd::Zero(pc);
	Vector3d sensorOffset = (sensor.block(si,3,1,3)-sensor.block(si,0,1,3)).transpose();
	for(int i=0; i<pc; i++) {
		Vector3d sensorState = sensorOffset+particles.block(i,0,1,3).transpose();
		for(int j=6; j<186; j++) {
			tempScores(i)+=getProbabilityOfObstacle(sensorState,j-6.,sensor(si,j));
		}
	}
	// tempScores /= tempScores.sum();
	probabilities = probabilities.cwiseProduct(tempScores);
	probabilities /= probabilities.sum();
}

void MonteCarlo::sensorUpdateRefined(int si, int pc) {
	// range (for display)
	ranges.clear();
	// For each particle, find the "probability" that the robot is located there
	VectorXd tempScores = VectorXd::Zero(pc);
	Vector3d sensorOffset = (sensor.block(si,3,1,3)-sensor.block(si,0,1,3)).transpose();
	for(int i=0; i<pc; i++) {
		int x = int(round(particles(i,0)/cmPerCell));
		int y = int(round(particles(i,1)/cmPerCell));
		if(x>mapH-10||x<10||y<10||y>mapW-10) {
			tempScores(i) = eps;
			continue;
		}
		// The "I'm on a wall" heuristic
		if(map(y,x) < 0.1) {
			tempScores(i) = .1;
			continue;
		}
		if(map(y,x) < 0) {
			tempScores(i) = .1;
			continue;
		}
		Vector3d sensorState = sensorOffset+particles.block(i,0,1,3).transpose();
		for(int j=6; j<186; j+=1) {
			if(j>85 && j<107) continue; // the person ignorer
			drawRange(sensorState,j-6.,sensor(si,j));
			int angle = (int(sensorState(2)*180/M_PI-90+j-6)+1800)%360;
			// cout << angle << " " << lookUpTable.size() << endl;
			// double angle = sensorState(2)+(j-96)*M_PI/180.;
			double range = abs(lookUpTable[angle](y,x)-sensor(si,j)) + rand()/RAND_MAX*10;
			// double range = 10;
			// double range = getRange(int(round(particles(i,1)/cmPerCell)),int(round(particles(i,0)/cmPerCell)),angle)-sensor(si,j);
			// cout << range << " ";
			if(range<1) range = 1;
			tempScores(i) += 1/range;
		}
		tempScores(i)*=map(y,x);
		// cout << tempScores(i) << endl;
		// if(i%50==0) cout << i << endl;
	}
	// tempScores /= tempScores.sum();
	probabilities = probabilities.cwiseProduct(tempScores);
	// cout << probabilities << endl;
	probabilities /= probabilities.sum();
}

void MonteCarlo::motionUpdate(const Vector3d &delta) {
	for(int i=0; i<particles.rows(); i++) {
		Vector3d noise = Vector3d::Random();
		noise.head<2>()*=2;
		// noise.head<2>()-=(Vector2d::Ones());
		noise(2)*=M_PI/20.;
		// noise(2)-=M_PI/360.;
		particles(i,0) += (delta(0)*cos(particles(i,2))+delta(1)*sin(particles(i,2)));
		particles(i,1) += (-delta(0)*sin(particles(i,2))+delta(1)*cos(particles(i,2)));
		particles(i,2) += delta(2);
		particles.row(i)+=noise.transpose();
		particles(i,2) = fmod(particles(i,2),2*M_PI);
		if(particles(i,2)>M_PI) particles(i,2) -= 2*M_PI;
	}
}

void MonteCarlo::drawRange(const Vector3d &state, float angle, float range) {
	float actualAngle = state(2)+((angle+90)*M_PI/180.);
	int x = round((state(0)+range*cos(actualAngle))/cmPerCell);
	int y = round((state(1)+range*sin(actualAngle))/cmPerCell);

	if(y<0||y>=mapH||x<0||x>=mapW) return;
	Vector2i r(x,y);
	ranges.push_back(r);
}

float MonteCarlo::getProbabilityOfObstacle(const Vector3d &state, float angle, float range) {
	float actualAngle = state(2)+((angle-90)*M_PI/180.);
	int x = round((state(0)+range*cos(actualAngle))/cmPerCell);
	int y = round((state(1)+range*sin(actualAngle))/cmPerCell);

	if(y<0||y>=mapH||x<0||x>=mapW) return eps;
	float val = map(y,x);
	Vector2i r(x,y);
	ranges.push_back(r);
	if(val<0) val=eps;
	else return val;
}

void MonteCarlo::resample() {
	cout << "Resampling..." << endl;
	//Bin the particles
	// double binX[cmPerCell*mapW]; for(int i=0; i<cmPerCell*mapW; i++) binX[i] = 0.0;
	// double binY[cmPerCell*mapH]; for(int i=0; i<cmPerCell*mapH; i++) binY[i] = 0.0;
	// double binT[3600] = {0};
	int totalParticles = particles.rows();
	// for(int i=0; i<totalParticles; i++) {
	// 	if(particles(i,0)/cmPerCell<mapW && particles(i,0) >= 0)
	// 		binX[int(round(particles(i,0)))]+=probabilities(i);
	// 	if(particles(i,1)/cmPerCell<mapH && particles(i,1) >= 0)
	// 		binY[int(round(particles(i,1)))]+=probabilities(i);
	// 	binT[(int(particles(i,2)*1800/M_PI)+3600)%3600]+=probabilities(i);
	// }

	// Create weighted distribution
	default_random_engine generator;

	discrete_distribution<int> globalDist (probabilities.data(),probabilities.data()+probabilities.size());

	// discrete_distribution<int> distX (&binX[0],&binX[mapW*cmPerCell-1]);
	// discrete_distribution<int> distY (&binY[0],&binY[mapH*cmPerCell-1]);
	// discrete_distribution<int> distT (&binT[0],&binT[3599]);

	// Sample
	for(int i=0; i<totalParticles; i++) {
		Vector3d noise = Vector3d::Random();
		noise.head<2>()*=20;
		// noise.head<2>()-=(25*Vector2d::Ones());
		noise(2)*=M_PI/30.;
		// noise(2)-=M_PI/10.;
		particles.row(i) = particles.row(globalDist(generator))+noise.transpose();
		// particles(i,0) = (double(distX(generator))+rand()/double(RAND_MAX));
		// particles(i,1) = (double(distY(generator))+rand()/double(RAND_MAX));
		// particles(i,2) = (double(distT(generator))+rand()/double(RAND_MAX))*M_PI/1800.;
		probabilities(i) = 1./totalParticles;
	}
}

void MonteCarlo::constructLUT() {
	for(int i=0; i<360; i++) {
		// lookUpTable.push_back(MatrixXd(mapH,mapW));
		for(int y=0; y<mapH; y++) {
			for(int x=0; x<mapW; x++) {
				// if(map(y,x)<0) cout << 1000000 << ",";
				// lookUpTable[i-42](y,x) = getRange(x,y,i*M_PI/180);
				cout << getRange(y,x,i*M_PI/180) << ",";
			}
			cout << endl;
		}

		cout << ";" << endl;
	}

}

void MonteCarlo::loadLUT(string file) {
	string line;
	ifstream lutFile(file.c_str());

	// for(int i=0; i<360; i++) {
	lookUpTable.push_back(MatrixXd(mapH,mapW));
	// }

	int y=0;
	int t=0;
	if(lutFile.is_open()) {
		while(getline(lutFile,line)) {
			if(line[0]!=';') {
				VectorXd row = split(line,',',mapW);
				lookUpTable[t].block(y,0,1,mapW) = row.transpose();
				y++;
			}
			else {
				lookUpTable.push_back(MatrixXd(mapH,mapW));
				t++;
				y=0;
				// cout << t << " ";
			}
		}
		// cout << endl;
	}
}

double MonteCarlo::getRange(int y, int x, double t) {
	// int boundaryLength = int(max(mapH,mapW)*1.5);
	// int x2,y2;
	// double range = boundaryLength;
	// x2 = x + boundaryLength*cos(t);
	// y2 = y + boundaryLength*sin(t);
	// vector<vector<int> > line(boundaryLength+1);
	// // cout << x << ", " << y << ", " << x2 << ", " << y2 << endl;
	// int lineLength = bresenhamLine(x,y,x2,y2,line);

	// for(int i=0; i<lineLength; i++) {
	// 	if(line[i][1]>=mapH||line[i][1]<0||line[i][0]<0||line[i][0]>=mapW) {
	// 		break;
	// 	}
	// 	double mapValue = map(line[i][1],line[i][0]);
	// 	if(mapValue<0.01) {
	// 		range = sqrt(pow((line[i][0]-x),2)+pow((line[i][1]-y),2));
	// 		break;
	// 	}
	// }

	// return range*cmPerCell;

	double range = 1500;

	int x_ = x;
	int y_ = y;
	double c = cos(t);
	double s = sin(t);
	while(y_<mapH && y_>=0 && x_>=0 && x_<mapW) {
		// if(t*180/M_PI>28)
		// 	cout << y_ << "," << x_ << endl;
		double mapValue = map(y_,x_);
		if(mapValue<0.01) {
			range = sqrt(pow(x_-x,2)+pow(y_-y,2))*cmPerCell;
			break;
		}
		x_ = int(round(x_ + c));
		y_ = int(round(y_ + s));
	}

	return range;
}

// CHECK THIS
int MonteCarlo::bresenhamLine(int x1,int y1,int x2,int y2, vector<vector<int> > &line) {
	// vector<vector<int> > line;

	int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i,j;
	j=0;
	dx=x2-x1;
	dy=y2-y1;
	// line.resize(dy+dx);
	dx1=fabs(dx);
	dy1=fabs(dy);
	px=2*dy1-dx1;
	py=2*dx1-dy1;
	if(dy1<=dx1) {
		if(dx>=0) {
			x=x1;
			y=y1;
			xe=x2;
		}
		else {
			x=x2;
			y=y2;
			xe=x1;
		}
		vector<int> p {x,y};
		line[j] = p;
		j++;
		for(i=0;x<xe;i++) {
			x=x+1;
			if(px<0) {
				px=px+2*dy1;
			}
			else {
				if((dx<0 && dy<0) || (dx>0 && dy>0)) {
					y=y+1;
				}
				else {
					y=y-1;
				}
				px=px+2*(dy1-dx1);
			}
			vector<int> p {x,y};
			line[j] = p;
			j++;
		}
	}
	else {
		if(dy>=0) {
			x=x1;
			y=y1;
			ye=y2;
		}
		else {
			x=x2;
			y=y2;
			ye=y1;
		}
		vector<int> p{x,y};
		line[j] = p;
		j++;
		for(i=0;y<ye;i++) {
			y=y+1;
			if(py<=0) {
				py=py+2*dx1;
			}
			else {
				if((dx<0 && dy<0) || (dx>0 && dy>0)) {
					x=x+1;
				}
				else {
					x=x-1;
				}
				py=py+2*(dx1-dy1);
			}
			vector<int> p {x,y};
			line[j] = p;
			j++;
		}
	}

	return j-1;
	// return line;
}

void MonteCarlo::display() {
	Mat img = Mat::zeros(mapH,mapW,CV_8UC3);
	for(int i=0; i<particles.rows(); i++) {
		if(particles(i,0)>=31 && particles(i,0)<mapW*cmPerCell-31 && particles(i,1)>=31 && particles(i,1)<mapH*cmPerCell-31) {
			int y = int(round(particles(i,1)/cmPerCell));
			int x = int(round(particles(i,0)/cmPerCell));
			Vec3b & color0 = img.at<Vec3b>(y,x);
			Vec3b & colorN = img.at<Vec3b>(y+1,x);
			Vec3b & colorS = img.at<Vec3b>(y-1,x);
			Vec3b & colorE = img.at<Vec3b>(y,x+1);
			Vec3b & colorW = img.at<Vec3b>(y,x-1);
			if(color0[2]<255) color0[2] += 128;
			if(colorN[2]<255) colorN[2] += 128;
			if(colorS[2]<255) colorS[2] += 128;
			if(colorE[2]<255) colorE[2] += 128;
			if(colorW[2]<255) colorW[2] += 128;

		}
	}
	for(int i=0; i<ranges.size(); i++) {
		Vec3b & color = img.at<Vec3b>(ranges[i](1),ranges[i](0));
		if(color[0]<255) color[0] +=128;
	}

	namedWindow("particles", WINDOW_AUTOSIZE);
	imshow("particles", img+mapImg);

	waitKey(25);

	char outFile[30];
	sprintf(outFile, "data/output/out%06d.png",imageCount);
	cout << outFile << endl;

	// outFile << "data/output/out" << setfill('0') << setw(6) << imageCount;

	imwrite(outFile,img+mapImg);
	imageCount++;
}

/********************************************************************************
* Reads the log file as measurements											*
********************************************************************************/
bool MonteCarlo::processLog(string file) {
	string line;
	ifstream logFile(file.c_str());

	int oi=0;
	int si=0;
	if(logFile.is_open()) {
		while(getline(logFile,line)) {
			if(line[0]=='O') {
				// Only resize log2(n) times
				if(oi>odom.rows()-1) {
					odom.conservativeResize(2*odom.rows(),ODOMLENGTH);
				}
				VectorXd odomMeasurement = split(line.substr(2),' ',ODOMLENGTH);
				odom.block(oi,0,1,ODOMLENGTH) = odomMeasurement.transpose();
				oi++;
			}
			else if (line[0]=='L') {
				if(si>sensor.rows()-1) {
					sensor.conservativeResize(2*sensor.rows(),SENSORLENGTH);
				}
				VectorXd sensorMeasurement = split(line.substr(2),' ',SENSORLENGTH);
				sensor.block(si,0,1,SENSORLENGTH) = sensorMeasurement.transpose();
				si++;
			}
		}
		logFile.close();
	}
	else {
		throw "Log file unreadable";
	}

	// Take the matrices back down to their acutal size
	odom.conservativeResize(oi,ODOMLENGTH);
	sensor.conservativeResize(si,SENSORLENGTH);
	return true;
}

/********************************************************************************
* Converts the map to matrix													*
********************************************************************************/
bool MonteCarlo::processMap(string file, int w, int h) {
	string line;
	ifstream mapFile(file.c_str());

	int i=0;
	if(mapFile.is_open()) {
		while(getline(mapFile,line)) {
			if(i>6) { // ignore map settings, input manually
				VectorXd row = split(line,' ',w);
				map.block(i-7,0,1,w) = row.transpose();
			}
			i++;
		}
		mapFile.close();
	}
	else {
		throw "Map file unreadable";
	}

	return true;
}

/********************************************************************************
* Splits string by delimeter (and converts to float) with output length of w	*
********************************************************************************/
VectorXd MonteCarlo::split(string s, char del, int w) {
	VectorXd row(w);

	string::size_type lastPos = s.find_first_not_of(del,0);
	string::size_type pos = s.find_first_of(del,lastPos);

	int i=0;
	while(string::npos!=pos || string::npos!=lastPos) {
		row(i) = stof(s.substr(lastPos,pos-lastPos));

		lastPos = s.find_first_not_of(del,pos);
		pos = s.find_first_of(del,lastPos);

		i++;
	}

	return row;
}

int main(int argc, char **argv) {
	int particleCount;
	if(argc<2) {
		cout << "Using 10k for particle count" << endl;
		particleCount = 10000;
	}
	else particleCount = stoi(argv[1]);
	MonteCarlo mc = MonteCarlo();
	mc.init("data/map/wean.dat",800,800,10);
	mc.localize("data/log/ascii-robotdata2.log",particleCount);	

	return 0;
}