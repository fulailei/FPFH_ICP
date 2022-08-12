

#pragma once

#include <osgGA/GUIEventHandler>


class PickHandler : public osgGA::GUIEventHandler
{
public:
	PickHandler(void);
	~PickHandler(void);

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

protected:
	void pick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

private:
	float mX;
	float mY;
	bool mEnableDragger;
};



