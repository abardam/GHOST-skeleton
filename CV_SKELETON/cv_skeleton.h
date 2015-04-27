#pragma once
#include <opencv2\opencv.hpp>


//pointerless version of skeleton node for saving/loading
struct SkeletonNodeHard{
	std::vector<SkeletonNodeHard> mChildren;
	cv::Mat mTransformation;
	cv::Mat mTempTransformation; //temporary GLOBAL transformation
	std::string mName;
	std::string mParentName;

	std::string toString(std::string prefix) const;

	SkeletonNodeHard(){};
	SkeletonNodeHard(const SkeletonNodeHard& c){
		mChildren = c.mChildren;
		mTransformation = c.mTransformation.clone();
		mTempTransformation = c.mTempTransformation.clone();
		mName = c.mName;
		mParentName = c.mParentName;
	};
};
typedef std::map<const std::string, SkeletonNodeHard*> SkeletonNodeHardMap;
typedef std::pair<const std::string, SkeletonNodeHard*> SkeletonNodeHardEntry;


struct BodyPartDefinition{
	std::string mNode1Name;
	std::string mNode2Name;

	float mNode1Offset[3];
	float mNode2Offset[3];

	std::string mBodyPartName;
	float mColor[3];

	BodyPartDefinition(std::string name, std::string n1, std::string n2, float * color, float * n1o = 0, float * n2o = 0) :
		mBodyPartName(name),
		mNode1Name(n1),
		mNode2Name(n2){
		mColor[0] = color[0];
		mColor[1] = color[1];
		mColor[2] = color[2];

		if (n1o == 0){
			mNode1Offset[0] = 0;
			mNode1Offset[1] = 0;
			mNode1Offset[2] = 0;
		}
		else{
			mNode1Offset[0] = n1o[0];
			mNode1Offset[1] = n1o[1];
			mNode1Offset[2] = n1o[2];
		}

		if (n2o == 0){
			mNode2Offset[0] = 0;
			mNode2Offset[1] = 0;
			mNode2Offset[2] = 0;
		}
		else{
			mNode2Offset[0] = n2o[0];
			mNode2Offset[1] = n2o[1];
			mNode2Offset[2] = n2o[2];
		}
	}

	BodyPartDefinition(std::string name, std::string n1, std::string n2, float r, float g, float b, float * n1o = 0, float * n2o = 0) :
		mBodyPartName(name),
		mNode1Name(n1),
		mNode2Name(n2){
		mColor[0] = r;
		mColor[1] = g;
		mColor[2] = b;

		if (n1o == 0){
			mNode1Offset[0] = 0;
			mNode1Offset[1] = 0;
			mNode1Offset[2] = 0;
		}
		else{
			mNode1Offset[0] = n1o[0];
			mNode1Offset[1] = n1o[1];
			mNode1Offset[2] = n1o[2];
		}

		if (n2o == 0){
			mNode2Offset[0] = 0;
			mNode2Offset[1] = 0;
			mNode2Offset[2] = 0;
		}
		else{
			mNode2Offset[0] = n2o[0];
			mNode2Offset[1] = n2o[1];
			mNode2Offset[2] = n2o[2];
		}
	}

	BodyPartDefinition(std::string name, std::string n1, std::string n2) :
		mBodyPartName(name),
		mNode1Name(n1),
		mNode2Name(n2){
		mColor[0] = 1;
		mColor[1] = 1;
		mColor[2] = 1;

		mNode1Offset[0] = 0;
		mNode1Offset[1] = 0;
		mNode1Offset[2] = 0;
		mNode2Offset[0] = 0;
		mNode2Offset[1] = 0;
		mNode2Offset[2] = 0;
	}

	BodyPartDefinition(){
		mColor[0] = 1;
		mColor[1] = 1;
		mColor[2] = 1;

		mNode1Offset[0] = 0;
		mNode1Offset[1] = 0;
		mNode1Offset[2] = 0;
		mNode2Offset[0] = 0;
		mNode2Offset[1] = 0;
		mNode2Offset[2] = 0;
	}
};

typedef std::vector<BodyPartDefinition> BodyPartDefinitionVector;


std::ostream& operator<<(std::ostream& os, const SkeletonNodeHard& snh);

//opencv serialization for skeleton node hard
void write(cv::FileStorage& fs, const std::string&, const SkeletonNodeHard& n);
void read(const cv::FileNode& node, SkeletonNodeHard& n, const SkeletonNodeHard& default_value = SkeletonNodeHard());

//opencv serialization for body parts
void write(cv::FileStorage& fs, const std::string&, const BodyPartDefinition& n);
void read(const cv::FileNode& node, BodyPartDefinition& n, const BodyPartDefinition& default_value = BodyPartDefinition());

//retrieves the global transform for the specified body part. you need to run cv_draw_and_build_skeleton first in order to set the snh transformations. also returns the length of the body part, if you want.
cv::Mat get_bodypart_transform(const BodyPartDefinition& bpd, const SkeletonNodeHardMap& snhMap, const cv::Mat * external_parameters = 0, float * length = 0);

void cv_draw_and_build_skeleton(SkeletonNodeHard * node, const cv::Mat& parent_transform, const cv::Mat& camera_matrix, SkeletonNodeHardMap * snhMap = NULL, cv::Mat& image = cv::Mat());

cv::Point project2D(cv::Vec4f pt, const cv::Mat& camera_matrix);
cv::Vec4f get_origin(const cv::Mat& pt_m);

//function for saving input frames
bool save_input_frame(
	const std::string& filename,
	const double& time,
	const cv::Mat& camera_pose,
	const float& win_width,
	const float& win_height,
	const float& fovy,
	const SkeletonNodeHard& snh,
	const cv::Mat& color,
	const cv::Mat& depth);

//function for loading input frames
bool load_input_frame(
	const std::string& filename,
	double& time,
	cv::Mat& camera_pose,
	cv::Mat& camera_matrix,
	SkeletonNodeHard& snh,
	cv::Mat& color,
	cv::Mat& depth);