#include "cv_skeleton.h"


//opencv serialization
void write(cv::FileStorage& fs, const std::string& s, const SkeletonNodeHard& n){
	fs << "{" << "name" << n.mName
		<< "parentname" << n.mParentName
		<< "transformation" << n.mTransformation
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
		cv::FileNode children = node["children"];
		if (children.type() != cv::FileNode::SEQ) return;
		n.mChildren.clear();
		for (auto it = children.begin(); it != children.end(); ++it){
			n.mChildren.push_back(SkeletonNodeHard());
			read(*it, n.mChildren.back(), default_value);
		}
	}
}

void write(cv::FileStorage& fs, const std::string&, const BodyPartDefinition& n){
	fs << "{" << "bodypartname" << n.mBodyPartName
		<< "node1name" << n.mNode1Name
		<< "node2name" << n.mNode2Name
		<< "color_r" << n.mColor[0] << "color_g" << n.mColor[1] << "color_b" << n.mColor[2]
		<< "node1offset_x" << n.mNode1Offset[0]
		<< "node1offset_y" << n.mNode1Offset[1]
		<< "node1offset_z" << n.mNode1Offset[2]
		<< "node2offset_x" << n.mNode2Offset[0]
		<< "node2offset_y" << n.mNode2Offset[1]
		<< "node2offset_z" << n.mNode2Offset[2]
		<< "}";
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
		n.mNode1Offset[0] = (float)node["node1offset_x"];
		n.mNode1Offset[1] = (float)node["node1offset_y"];
		n.mNode1Offset[2] = (float)node["node1offset_z"];
		n.mNode2Offset[0] = (float)node["node2offset_x"];
		n.mNode2Offset[1] = (float)node["node2offset_y"];
		n.mNode2Offset[2] = (float)node["node2offset_z"];
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
	pt[0] /= pt[2];
	pt[1] /= pt[2];
	pt[2] = 1;
	pt[3] = 1;
	cv::Mat pt_m = camera_matrix * cv::Mat(pt);
	int x = pt_m.ptr<float>(0)[0];
	int y = pt_m.ptr<float>(1)[0];

	return cv::Point(x, y);
}

cv::Mat get_bodypart_transform(const BodyPartDefinition& bpd, const SkeletonNodeHardMap& snhMap, const cv::Mat * external_parameters, float * length){
	auto parentEntry = snhMap.find(bpd.mNode1Name);
	if (parentEntry == snhMap.end()) return cv::Mat();

	auto childEntry = snhMap.find(bpd.mNode2Name);
	if (childEntry == snhMap.end()) return cv::Mat();

	cv::Mat parent_offset = cv::Mat::eye(4, 4, CV_32F);
	parent_offset.ptr<float>(0)[3] = bpd.mNode1Offset[0];
	parent_offset.ptr<float>(1)[3] = bpd.mNode1Offset[1];
	parent_offset.ptr<float>(2)[3] = bpd.mNode1Offset[2];

	cv::Mat parent_transform = parentEntry->second->mTempTransformation * parent_offset;
	cv::Vec4f parent_pt = get_origin(parent_transform);

	cv::Mat child_offset = cv::Mat::eye(4, 4, CV_32F);
	child_offset.ptr<float>(0)[3] = bpd.mNode2Offset[0];
	child_offset.ptr<float>(1)[3] = bpd.mNode2Offset[1];
	child_offset.ptr<float>(2)[3] = bpd.mNode2Offset[2];

	cv::Mat child_transform = childEntry->second->mTempTransformation * child_offset;
	//cv::Mat childs_parent_transform = snhMap.find(childEntry->second->mParentName)->second->mTempTransformation;

	//volume

	cv::Mat volume_transform = parent_transform;

	//cv::Mat volume_transform = childs_parent_transform.clone(); // volume transform has the rotation matrix of node2's direct parent but the translation of the node1 EDIT: no, i guess not
	//volume_transform.ptr<float>(0)[3] = parent_pt(0);
	//volume_transform.ptr<float>(1)[3] = parent_pt(1);
	//volume_transform.ptr<float>(2)[3] = parent_pt(2);

	if (length != 0 && external_parameters != 0){
		*length = cv::norm(get_origin(external_parameters->inv()*child_transform) - get_origin(external_parameters->inv()*parent_transform)); //length value is affected by the root transform (which may include scaling)
	}

	return volume_transform;
}

void cv_draw_and_build_skeleton(SkeletonNodeHard * node, const cv::Mat& parent_transform, const cv::Mat& camera_matrix, SkeletonNodeHardMap * snhMap, cv::Mat& image){

	if (snhMap != NULL){
		snhMap->insert(SkeletonNodeHardEntry(node->mName, node));
	}

	cv::Vec4f parent_pt = get_origin(parent_transform);
	cv::Mat child_transform = parent_transform * node->mTransformation;

	node->mTempTransformation = child_transform;

	cv::Vec4f child_pt = get_origin(child_transform);

	if (!image.empty()) cv::line(image, project2D(parent_pt, camera_matrix), project2D(child_pt, camera_matrix), cv::Scalar(255, 0, 0));

	for (auto it = node->mChildren.begin(); it != node->mChildren.end(); ++it){
		cv_draw_and_build_skeleton(&*it, child_transform, camera_matrix, snhMap, image);
	}
}
