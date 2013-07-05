#include "PoissonDiskSampling.h"

PoissonDiskSampling::PoissonDiskSampling(int width, int height, double pitch):width_(width), height_(height), pitch_(pitch)
{
	nodes.resize(width_*height_);

	for(int j=0; j<height_; j++){
		for(int i=0; i<width_; i++){		
			nodes[i+j*width] = Node(i, j, 1.0);
		}
	}
	srand(time(0));	
	f = Converter();
}

PoissonDiskSampling::~PoissonDiskSampling(){

}
/*
void PoissonDiskSampling::sample(std::vector<Eigen::Vector3d> &points){
	points.resize(0);
	int count = 0;
	std::vector<Node>::iterator n_it;	
	std::vector<int> neighbors;	
	int max = 100000;
	std::vector<Node> active_list;

	while(count < max){
		Eigen::Vector3d point = genRandomPoint();
		int pid_x = point[0]/pitch_;
		int pid_y = point[1]/pitch_;
		int n_id = pid_x+pid_y*width_;
		n_it = nodes.begin()+ n_id;
		neighbors = calcNeighborIndex(*n_it, f(n_it->val));

		std::vector<int>::iterator id_it;
		bool free = true;
		for(id_it = neighbors.begin(); id_it != neighbors.end(); id_it++){
			if((nodes.begin()+*id_it)->visited == true){
				free = false;
			}
		}
		if(free){
			points.push_back(point);			
			nodes.at(n_id).visited = true;
		}
		count ++;	
	}
}
*/

void PoissonDiskSampling::sample(std::vector<Eigen::Vector3d> &points){
	points.resize(0);
	int count = 0;
	std::vector<Node>::iterator n_it;	
	std::vector<int> neighbors;	
	//int max = 100000;
	int K = 10;
	std::deque<Eigen::Vector3d> active_list;
	Eigen::Vector3d init_point = Eigen::Vector3d(width_-2, height_-2, 0);//genRandomPoint();
	active_list.push_back(init_point);
	insertPoint(nodes, points, init_point);

	while(active_list.size() != 0){

		Eigen::Vector3d point = active_list[0];
		bool is_valid = false;
		int pid_x = point[0]/pitch_;
		int pid_y = point[1]/pitch_;
		int n_id = pid_x+pid_y*width_;
		n_it = nodes.begin()+n_id;
		double r = f(n_it->val);		

		// generate K points in r~2r region		
		for(int i=0; i<K; i++){			
			Eigen::Vector3d p = genRandomPoint(point, r, r*2);
			double R = f((nodes.begin()+ ((int)(p[0]/pitch_)+(int)(p[1]/pitch_)*width_))->val);
			if(!existNeighbors(p, R, points)){
				active_list.push_back(p);
				insertPoint(nodes, points, p);
				is_valid = true;
			}
		}		

		if(!is_valid){
			//active_list.erase(active_list.begin());
			active_list.pop_front();
		}
		//int pid_x = point[0]/pitch_;
		//int pid_y = point[1]/pitch_;
		//int n_id = pid_x+pid_y*width_;
		//n_it = nodes.begin()+ n_id;
		//neighbors = calcNeighborIndex(*n_it, f(n_it->val));

/*
		std::vector<int>::iterator id_it;
		bool free = true;
		for(id_it = neighbors.begin(); id_it != neighbors.end(); id_it++){
			if((nodes.begin()+*id_it)->visited == true){
				free = false;
			}
		}
		if(free){
			points.push_back(point);			
			nodes.at(n_id).visited = true;

		}
		count ++;	
		*/
	}
}

void PoissonDiskSampling::insertPoint(std::vector<Node> &nodes, std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &point){
	int pid_x = point[0]/pitch_;
	int pid_y = point[1]/pitch_;
	int n_id = pid_x+pid_y*width_;
	nodes.at(n_id).visited = true;
	points.push_back(point);
	nodes.at(n_id).p_id.push_back(points.size()-1);
}

void PoissonDiskSampling::setDensityFunc(const std::vector<double> &dfunc){
	assert((int)dfunc.size() == width_*height_);
	std::vector<double>::const_iterator d_it = dfunc.begin();
	std::cout << nodes.size() << " "<< dfunc.size() << std::endl;
	for(std::vector<Node>::iterator it = nodes.begin(); it != nodes.end(); ++it, ++d_it){
		it->val = *d_it;
	}
}

void PoissonDiskSampling::setConverter(boost::function<double(double)> _f){	
	f = _f;
}

void PoissonDiskSampling::test(){
	std::cout << "test" << std::endl;
	Eigen::Vector3d p;
	p[0] = width_/2; p[1] = height_/2, p[2] = 0;
	double r = 5;
	double r2 = 10;
	for(int i=0; i<100; i++){
		Eigen::Vector3d point = genRandomPoint(p, r, r2);
		std::cout << (point-p).eval().norm() << std::endl;
	}
}

std::vector<int> PoissonDiskSampling::calcNeighborIndex(Node &node, double r){
	std::vector<int> neighbors;
	double dx = static_cast<double>(node.x_id);
	double dy = static_cast<double>(node.y_id);

	int min_x = (dx*pitch_-r)/pitch_;
	int max_x = (dx*pitch_+r)/pitch_;
	int min_y = (dy*pitch_-r)/pitch_;
	int max_y = (dy*pitch_+r)/pitch_;

	for(int j=min_y; j<=max_y; j++){
		for(int i=min_x; i<=max_x; i++){
			if(i<0 || i>=width_ || j<0 || j>=height_){
				continue;
			}
			neighbors.push_back(i+j*width_);
		}
	}
	return neighbors;
}

Eigen::Vector3d PoissonDiskSampling::genRandomPoint(){
	double x = ((double)rand()+1.0)/((double)RAND_MAX+2.0)*(double)width_;
	double y = ((double)rand()+1.0)/((double)RAND_MAX+2.0)*(double)height_;	
	return Eigen::Vector3d(x, y, 0);
}

Eigen::Vector3d PoissonDiskSampling::genRandomPoint(const Eigen::Vector3d &point, double lower_r, double upper_r){

	Eigen::Vector3d p;
	
	double sq_lr = lower_r*lower_r;
	double sq_ur = upper_r*upper_r;	

	while(1){
		double x = ((double)rand()+1.0)/((double)RAND_MAX+2.0)*2*upper_r - upper_r;
		double y = ((double)rand()+1.0)/((double)RAND_MAX+2.0)*2*upper_r - upper_r;
		//double z = point[2] + ((double)rand()+1.0)/((double)RAND_MAX+2.0)*2*upper_r - upper_r;		

		double sq_r = x*x+y*y;
		if(sq_r < sq_lr || sq_r > sq_ur){
			continue;
		}
		x += point[0]; y+= point[1];
		if(x < 0 || x > width_){
			continue;
		}
		if(y < 0 || y > height_){
			continue;
		}		
		p = Eigen::Vector3d(x, y, 0);
		break;
	}

	return p;
}

bool PoissonDiskSampling::existNeighbors(Eigen::Vector3d &point, double r, std::vector<Eigen::Vector3d> &points){
	bool exist = false;

	double r2 = r*r;

	std::vector<int> neighbors;
	double dx = point[0];
	double dy = point[1];

	int min_x = (dx-r)/pitch_;
	int max_x = (dx+r)/pitch_;
	int min_y = (dy-r)/pitch_;
	int max_y = (dy+r)/pitch_;

	for(int j=min_y; j<=max_y; j++){
		for(int i=min_x; i<=max_x; i++){
			if(i<0 || i>=width_ || j<0 || j>=height_){
				continue;
			}
			neighbors.push_back(i+j*width_);
		}
	}
	std::vector<int>::iterator id_it = neighbors.begin();
	for(; id_it!= neighbors.end(); ++id_it){
		Node n = *(nodes.begin()+ *id_it);
		if(n.visited == true){
		for(int i=0; i<n.p_id.size(); i++){
			//if(n.visited == true && n.p_id > 0){
				double x = points[n.p_id[i]][0]-point[0];
				double y = points[n.p_id[i]][1]-point[1];			
				double dist2 = x*x+y*y;
				if(dist2 < r2){
					exist = true;
					break;				
				}
			}
		}		
	}
	return exist;
}

