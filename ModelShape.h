#pragma once

#include <osg/Group>

#include <osgManipulator/Selection>
#include <osgManipulator/CommandManager>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/TranslateAxisDragger>


class ModelShape : public osg::Group
{
public:
	ModelShape(osg::Node* shape);
	~ModelShape(void);

	void EnableDragger(void);
	void DisableDragger(void);

	osg::Matrix GetDraggerMatrix();

private:
	osg::ref_ptr<osg::Node> mShape;
	osg::ref_ptr<osgManipulator::Dragger> mDragger;
	osg::ref_ptr<osgManipulator::Selection> mSelection;
};


