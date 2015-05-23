#include "cv_skeleton.h"

//from assimp
#define AI_DEG_TO_RAD(x) ((x)*0.0174532925f)

//opencv serialization
void write(cv::FileStorage& fs, const std::string& s, const SkeletonNodeHard& n){
	fs << "{" << "name" << n.mName
		<< "parentname" << n.mParentName
		<< "transformation" << n.mTransformation
		<< "confidence" << n.confidence
		<< "children" << "[";
	for (auto it = n.mChildren.begin(); it != n.mChildren.end(); ++it){
		write(fs, s, *it);
	}
	fs << "]" << "}";
}
void read(const cv::FileNode& node, SkeletonNodeHard& n, const SkeletonNodeHard& default_value){
	if (node.empty()){
		n = default_value;
	}
	else{
		n.mName = (std::string)node["name"];
		n.mParentName = (std::string)node["parentname"];
		read(node["transformation"], n.mTransformation);
		if (node["confidence"].empty()){
			n.confidence = 1;
		}
		else{
			n.confidence = (float)node["confidence"];
		}
		cv::FileNode children = node["children"];
		if (children.type() != cv::FileNode::SEQ) return;
		n.mChildren.clear();
		for (auto it = children.begin(); it != children.end(); ++it){
			SkeletonNodeHard child_snh;
			read(*it, child_snh, default_value);
			n.mChildren.push_back(child_snh);
		}
	}
}

void write(cv::FileStorage& fs, const std::string&, const BodyPartDefinition& n){
	fs << "{" << "bodypartname" << n.mBodyPartName
		<< "node1name" << n.mNode1Name
		<< "node2name" << n.mNode2Name
		<< "color_r" << n.mColor[0] << "color_g" << n.mColor[1] << "color_b" << n.mColor[2]
		<< "node1offset" << n.mNode1Offset
		<< "node2offset" << n.mNode2Offset
		<< "}";
}


cv::Mat _create_translation_mat(float x, float y, float z){
	cv::Mat ret = cv::Mat::eye(4, 4, CV_32F);
	ret.ptr<float>(0)[3] = x;
	ret.ptr<float>(1)[3] = y;
	ret.ptr<float>(2)[3] = z;
	return ret;
}

void read(const cv::FileNode& node, BodyPartDefinition& n, const BodyPartDefinition& default_value){
	if (node.empty()){
		n = default_value;
	}
	else{
		n.mBodyPartName = (std::string)node["bodypartname"];
		n.mNode1Name = (std::string)node["node1name"];
		n.mNode2Name = (std::string)node["node2name"];
		n.mColor[0] = (float)node["color_r"];
		n.mColor[1] = (float)node["color_g"];
		n.mColor[2] = (float)node["color_b"];

		if (node["node1offset"].empty()){
			//old style
			float mNode1Offsetx = (float)node["node1offset_x"];
			float mNode1Offsety = (float)node["node1offset_y"];
			float mNode1Offsetz = (float)node["node1offset_z"];
			float mNode2Offsetx = (float)node["node2offset_x"];
			float mNode2Offsety = (float)node["node2offset_y"];
			float mNode2Offsetz = (float)node["node2offset_z"];

			n.mNode1Offset = _create_translation_mat(mNode1Offsetx, mNode1Offsety, mNode1Offsetz);
			n.mNode2Offset = _create_translation_mat(mNode2Offsetx, mNode2Offsety, mNode2Offsetz);
		}
		else{
			node["node1offset"] >> n.mNode1Offset;
			node["node2offset"] >> n.mNode2Offset;
		}
	}
}


//snh serialization
std::ostream& operator<<(std::ostream& os, const SkeletonNodeHard& snh){
	os << snh.toString("");
	return os;
}

std::string SkeletonNodeHard::toString(std::string prefix) const{
	std::stringstream text;
	text << prefix << mName << std::endl << mTransformation << std::endl;
	for (auto it = mChildren.begin(); it != mChildren.end(); ++it){
		text << it->toString(prefix + "---");
	}
	return text.str();
}


cv::Vec4f get_origin(const cv::Mat& pt_m){
	cv::Vec4f vert;

	vert(0) = pt_m.ptr<float>(0)[3];
	vert(1) = pt_m.ptr<float>(1)[3];
	vert(2) = pt_m.ptr<float>(2)[3];
	vert(3) = 1;

	return vert;
}

cv::Point project2D(cv::Vec4f pt, const cv::Mat& camera_matrix){
	cv::Mat pt_m = camera_matrix * cv::Mat(pt);

	int x = pt_m.ptr<float>(0)[0] / pt_m.ptr<float>(2)[0];
	int y = pt_m.ptr<float>(1)[0] / pt_m.ptr<float>(2)[0];

	return cv::Point(x, y);
}


SkeletonNodeHard * get_skeleton_node(const BodyPartDefinition& bpd, const SkeletonNodeHardMap& snhMap){
	auto parentEntry = snhMap.find(bpd.mNode1Name);
	return parentEntry->second;
}

SkeletonNodeHard * get_skeleton_node(const BodyPartDefinition& bpd, SkeletonNodeAbsoluteVector& snav){
	for (int i = 0; i < snav.size(); ++i){
		if (bpd.mNode1Name == snav[i].mName){
			return &snav[i];
		}
	}
}

cv::Mat get_bodypart_transform(const BodyPartDefinition& bpd, const SkeletonNodeHardMap& snhMap, const cv::Mat& camera_pose, float * length){
	auto parentEntry = snhMap.find(bpd.mNode1Name);
	if (parentEntry == snhMap.end()) return cv::Mat();

	//cv::Mat parent_offset = cv::Mat::eye(4, 4, CV_32F);
	//parent_offset.ptr<float>(0)[3] = bpd.mNode1Offset[0];
	//parent_offset.ptr<float>(1)[3] = bpd.mNode1Offset[1];
	//parent_offset.ptr<float>(2)[3] = bpd.mNode1Offset[2];

	cv::Mat parent_offset = bpd.mNode1Offset;

	cv::Mat parent_transform = parentEntry->second->mTempTransformation * parent_offset;
	//cv::Mat childs_parent_transform = snhMap.find(childEntry->second->mParentName)->second->mTempTransformation;

	//volume

	cv::Mat volume_transform = parent_transform;

	//cv::Mat volume_transform = childs_parent_transform.clone(); // volume transform has the rotation matrix of node2's direct parent but the translation of the node1 EDIT: no, i guess not
	//volume_transform.ptr<float>(0)[3] = parent_pt(0);
	//volume_transform.ptr<float>(1)[3] = parent_pt(1);
	//volume_transform.ptr<float>(2)[3] = parent_pt(2);

	if (length != 0){

		auto childEntry = snhMap.find(bpd.mNode2Name);
		if (childEntry == snhMap.end()){
			*length = 0;
			
		}
		else{
			cv::Vec4f parent_pt = get_origin(parent_transform);

			//cv::Mat child_offset = cv::Mat::eye(4, 4, CV_32F);
			//child_offset.ptr<float>(0)[3] = bpd.mNode2Offset[0];
			//child_offset.ptr<float>(1)[3] = bpd.mNode2Offset[1];
			//child_offset.ptr<float>(2)[3] = bpd.mNode2Offset[2];

			cv::Mat child_offset = bpd.mNode2Offset;

			cv::Mat child_transform = childEntry->second->mTempTransformation * child_offset;
			*length = cv::norm(get_origin(camera_pose.inv()*child_transform) - get_origin(camera_pose.inv()*parent_transform)); //length value is affected by the root transform (which may include scaling)
		}
	}

	return camera_pose * volume_transform;
}


cv::Mat get_bodypart_transform(const BodyPartDefinition& bpd, const SkeletonNodeAbsoluteVector& snav, const cv::Mat& camera_pose, float * length){
	int snh_i = -1;
	for (int i = 0; i < snav.size(); ++i){
		if (snav[i].mName == bpd.mNode1Name){
			snh_i = i;
			break;
		}
	}

	if (snh_i == -1) return cv::Mat();

	cv::Mat parent_offset = bpd.mNode1Offset;

	cv::Mat parent_transform = snav[snh_i].mTransformation * parent_offset;

	//volume

	cv::Mat volume_transform = parent_transform;

	if (length != 0){

		int snh_child_i = -1;
		for (int i = 0; i < snav.size(); ++i){
			if (snav[i].mName == bpd.mNode2Name){
				snh_child_i = i;
				break;
			}
		}

		if (snh_child_i == -1){
			*length = 0;

		}
		else{
			cv::Vec4f parent_pt = get_origin(parent_transform);
			cv::Mat child_offset = bpd.mNode2Offset;

			cv::Mat child_transform = snav[snh_child_i].mTransformation * child_offset;
			*length = cv::norm(get_origin(camera_pose.inv()*child_transform) - get_origin(camera_pose.inv()*parent_transform)); //length value is affected by the root transform (which may include scaling)
		}
	}

	return camera_pose * volume_transform;
}

void absolutize_snh(const SkeletonNodeHard& rel, std::vector<SkeletonNodeHard>& abs, const cv::Mat& parent_transform){

	SkeletonNodeHard snh;
	snh.mTransformation = rel.mTransformation * parent_transform;
	snh.mName = rel.mName;
	snh.mParentName = rel.mParentName;
	snh.confidence = rel.confidence;
	abs.push_back(snh);
	for (int i = 0; i < rel.mChildren.size(); ++i){
		absolutize_snh(rel.mChildren[i], abs, snh.mTransformation);
	}

}

void relativize_snh(const std::vector<SkeletonNodeHard>& abs, SkeletonNodeHard& rel){
	SkeletonNodeHardMap snhMap;
	for (int i = 0; i < abs.size(); ++i){
		SkeletonNodeHard snh = abs[i];
		snh.mTempTransformation = snh.mTransformation.clone();

		
		if (snhMap.find(snh.mParentName) != snhMap.end()){
			SkeletonNodeHard * parent = snhMap.find(snh.mParentName)->second;
			snh.mTransformation = snh.mTempTransformation * parent->mTempTransformation.inv();
			parent->mChildren.push_back(snh);
			snhMap.insert(SkeletonNodeHardEntry(snh.mName, &parent->mChildren.back()));

		}
		else{
			rel = snh;
			snhMap.insert(SkeletonNodeHardEntry(snh.mName, &rel));
		}
	}
}

void cv_draw_and_build_skeleton(SkeletonNodeHard * node, const cv::Mat& parent_transform, const cv::Mat& camera_matrix, const cv::Mat& camera_pose, SkeletonNodeHardMap * snhMap, cv::Mat& image, const cv::Scalar& color){

	if (snhMap != NULL){
		snhMap->insert(SkeletonNodeHardEntry(node->mName, node));
	}

	cv::Mat child_transform = node->mTransformation * parent_transform;

	node->mTempTransformation = child_transform;


	if (!image.empty()){
		cv::Vec4f parent_pt = get_origin(camera_pose * parent_transform);
		cv::Vec4f child_pt = get_origin(camera_pose * child_transform);

		cv::line(image, project2D(parent_pt, camera_matrix), project2D(child_pt, camera_matrix), color);
	}

	for (auto it = node->mChildren.begin(); it != node->mChildren.end(); ++it){
		cv_draw_and_build_skeleton(&*it, child_transform, camera_matrix, camera_pose, snhMap, image, color);
	}
}

void cv_draw_skeleton(cv::Mat& image, const SkeletonNodeAbsoluteVector& snav, const cv::Mat& camera_matrix, const cv::Mat& camera_pose, const cv::Scalar& color){
	for (int i = 0; i < snav.size(); ++i){
		int parent_id = -1;

		for (int j = 0; j < snav.size(); ++j){
			if (snav[j].mName == snav[i].mParentName){
				parent_id = j;
				break;
			}
		}

		if (parent_id == -1) continue;

		cv::Mat zero_pt(cv::Vec4f(0, 0, 0, 1));
		cv::Mat parent_proj_pt = camera_matrix * camera_pose * snav[parent_id].mTransformation * zero_pt;
		cv::Mat child_proj_pt = camera_matrix * camera_pose * snav[i].mTransformation * zero_pt;

		cv::line(image, cv::Point(parent_proj_pt.ptr<float>(0)[0] / parent_proj_pt.ptr<float>(2)[0],
			parent_proj_pt.ptr<float>(1)[0] / parent_proj_pt.ptr<float>(2)[0]),
			cv::Point(child_proj_pt.ptr<float>(0)[0] / parent_proj_pt.ptr<float>(2)[0],
			child_proj_pt.ptr<float>(1)[0] / parent_proj_pt.ptr<float>(2)[0]), color);
	}
}

bool save_input_frame(
	const std::string& filename,
	const double& time,
	const cv::Mat& camera_pose,
	const float& win_width,
	const float& win_height,
	const float& fovy,
	const SkeletonNodeHard& snh,
	const cv::Mat& color,
	const cv::Mat& depth,
	const int& facing){

	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	if (!fs.isOpened()) return false;

	fs << "time" << time;
	fs << "camera_extrinsic" << camera_pose;
	//fs << "camera_intrinsic" << camera_intrinsic;
	fs << "camera_intrinsic" << "{"
		<< "width" << win_width << "height" << win_height << "fovy" << fovy
		<< "}";
	fs << "skeleton" << snh;
	fs << "color" << color;
	fs << "depth" << depth;
	fs << "facing" << facing;
	fs.release();

	return true;
}

bool save_input_frame(
	const std::string& filename,
	const double& time,
	const cv::Mat& camera_pose,
	const cv::Mat& camera_matrix,
	const SkeletonNodeHard& snh,
	const cv::Mat& color,
	const cv::Mat& depth,
	const int& facing){

	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	if (!fs.isOpened()) return false;

	fs << "time" << time;
	fs << "camera_extrinsic" << camera_pose;
	//fs << "camera_intrinsic" << camera_intrinsic;
	fs << "camera_intrinsic_mat" << camera_matrix;
	fs << "skeleton" << snh;
	fs << "color" << color;
	fs << "depth" << depth;
	fs << "facing" << facing;
	fs.release();

	return true;
}

bool load_input_frame(
	const std::string& filename,
	double& time,
	cv::Mat& camera_pose,
	cv::Mat& camera_matrix,
	SkeletonNodeHard& snh,
	cv::Mat& color,
	cv::Mat& depth,
	int& facing){

	float win_width;
	float win_height;
	float fovy;

	cv::FileStorage fs;
	
	fs.open(filename, cv::FileStorage::READ);

	if (!fs.isOpened()) return false;

	fs["time"] >> time;
	fs["camera_extrinsic"] >> camera_pose;

	if (fs["camera_intrinsic"].empty()){
		fs["camera_intrinsic_mat"] >> camera_matrix;
	}
	else{
		fs["camera_intrinsic"]["width"] >> win_width;
		fs["camera_intrinsic"]["height"] >> win_height;
		fs["camera_intrinsic"]["fovy"] >> fovy;

		camera_matrix = cv::Mat::eye(4, 4, CV_32F);
		camera_matrix.ptr<float>(0)[0] = -win_width / (2 * tan(AI_DEG_TO_RAD((fovy * (win_width / win_height) / 2.)))); //for some strange reason this is inaccurate for non-square aspect ratios
		camera_matrix.ptr<float>(1)[1] = win_height / (2 * tan(AI_DEG_TO_RAD(fovy / 2.)));
		camera_matrix.ptr<float>(0)[2] = win_width / 2 + 0.5;
		camera_matrix.ptr<float>(1)[2] = win_height / 2 + 0.5;
		//camera_intrinsic.ptr<float>(2)[2] = -1;
	}

	fs["skeleton"] >> snh;
	fs["color"] >> color;
	fs["depth"] >> depth;
	fs["facing"] >> facing;

	fs.release();

	return true;
}
