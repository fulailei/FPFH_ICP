#include "PickHandler.h"
#include "ModelShape.h"

#include <osgViewer/Viewer>


PickHandler::PickHandler(void)
	: mX(0.0f)
	, mY(0.0f)
	, mEnableDragger(true)
{

}


PickHandler::~PickHandler(void)
{

}


bool PickHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	osgViewer::View* view = dynamic_cast<osgViewer::View*> (&aa);
	if (NULL == view)
	{
		return false;
	}

	switch (ea.getEventType())
	{
	case osgGA::GUIEventAdapter::PUSH:
	{
		mX = ea.getX();
		mY = ea.getY();
	}
	break;

	case osgGA::GUIEventAdapter::RELEASE:
	{
		if (ea.getX() == mX && ea.getY() == mY)
		{
			//pick(ea, aa);
		}
	}
	break;

	case osgGA::GUIEventAdapter::KEYDOWN:
	{
		if (ea.getKey() == 'd')
		{
			mEnableDragger = !mEnableDragger;
		}
	}
	break;

	default:
		break;
	}

	return false;
}


void PickHandler::pick(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	osgViewer::View* view = dynamic_cast<osgViewer::View*> (&aa);
	osgUtil::LineSegmentIntersector::Intersections hits;
	float x = ea.getX();
	float y = ea.getY();
	if (view->computeIntersections(x, y, hits))
	{
	
		osgUtil::LineSegmentIntersector::Intersection intersection = *hits.begin();
		osg::NodePath& nodePath = intersection.nodePath;
		int nNodeSize = static_cast<int> (nodePath.size());
		
		for (int i = 0; i < nNodeSize; i++) 
		{
			osg::Node* node = nodePath[i];
			//osg::Node* grandParent = node->getParent(0)->getParent(0);

			// This method maybe not right?
			
			osg::MatrixTransform *tran= dynamic_cast<osg::MatrixTransform*> (node);
			if (tran)
			{
				ModelShape* shape = dynamic_cast<ModelShape*> (tran->getChild(0)); //grandParent
				if (shape)
				{
					mEnableDragger ? shape->EnableDragger() : shape->DisableDragger();
					break;
				}
			}

		}
	}

}
