#include "ModelShape.h"

ModelShape::ModelShape(osg::Node* shape)
	: mShape(shape)
	, mDragger(new osgManipulator::TranslateAxisDragger())
	, mSelection(new osgManipulator::Selection())
{
	float scale = shape->getBound().radius() * 1.6;
	mDragger->setMatrix(osg::Matrix::scale(scale, scale, scale) *
		osg::Matrix::translate(shape->getBound().center()));

	mDragger->setupDefaultGeometry();

	mSelection->addChild(shape);
	addChild(mSelection);
}


ModelShape::~ModelShape(void)
{
}


void ModelShape::EnableDragger()
{
	addChild(mDragger);

	mDragger->addTransformUpdating(mSelection);
	mDragger->setHandleEvents(true);
}


void ModelShape::DisableDragger()
{
	removeChild(mDragger);

	mDragger->removeTransformUpdating(mSelection);
	mDragger->setHandleEvents(false);
}
osg::Matrix ModelShape::GetDraggerMatrix()
{
	return mDragger->getMatrix();
}