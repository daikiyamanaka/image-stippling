#ifndef POISSONDISKSAMPLING_H_
#define POISSONDISKSAMPLING_H_

#include <iostream>
#include <vector>
#include <deque>
#include <cassert>
#include <ctime>
#include <algorithm>
#include <Eigen/Sparse>
#include <boost/function.hpp>

class PoissonDiskSampling
{
public:
	class Node{
		public:
		Node(){};
		Node(int x, int y, double df):x_id(x), y_id(y), val(df){ visited = false;};
		~Node(){};
		Node(const Node& rn){
			x_id = rn.x_id;
			y_id = rn.y_id;
			val = rn.val;
			visited = rn.visited;
			p_id = rn.p_id;
		};
		int x_id, y_id;
		std::vector<int> p_id;
		double val;
		bool visited;
	};

	class Converter{
		public:
			Converter(){};
			~Converter(){};		
			double operator()(double val){
				return val;
			};
		};

	PoissonDiskSampling(int width, int height, double pitch);
	~PoissonDiskSampling();

	void sample(std::vector<Eigen::Vector3d> &points);
	void setDensityFunc(const std::vector<double> &density_func);
	void setConverter(boost::function<double(double)> _f);
	void test();

private:
	std::vector<int> calcNeighborIndex(Node &node, double r);
	Eigen::Vector3d genRandomPoint();		
	Eigen::Vector3d genRandomPoint(const Eigen::Vector3d &point, double lower_r, double upper_r);
	bool existNeighbors(Eigen::Vector3d &point, double r, std::vector<Eigen::Vector3d> &points);
 	void insertPoint(std::vector<Node> &nodes, std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &point);

	boost::function<double(double)> f;
	std::vector<Node> nodes;
	int width_, height_;
	double pitch_;

};
#endif
