#include "testosgQt.h"
#include <Qdir>
#include <ctime>
#include <time.h>
#include <Qtimer>
#include "EigenMath.h"
#include <random>


osg::ref_ptr<osg::Group> createLight(osg::ref_ptr<osg::Node> node)
{

	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();

	//当光照计算结果过于明亮或者暗淡时，允许法线的重缩放
	//stateset->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);

	//场景中的缩放变换非均匀时，允许法线归一化，以保证法线为单位长度
	//stateset->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

	//为获得光照效果允许光照，需要允许光照并至少允许一个光源
	//开启光照 将ON改为OFF则关闭光照
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	//允许一个光源GL_LIGHT0
	stateset->setMode(GL_LIGHT0, osg::StateAttribute::ON);

	//计算包围盒
	osg::BoundingSphere bs;
	node->computeBound();
	bs = node->getBound();

	//创建一个Light对象，定义光源参数
	osg::ref_ptr<osg::Light> light = new osg::Light();
	light->setLightNum(0);
	//设置方向
	light->setDirection(osg::Vec3(0.0f, 0.0f, -1.0f));
	//设置位置
	light->setPosition(osg::Vec4(bs.center().x(), bs.center().y(), bs.center().z() + bs.radius(), 1.0f));
	//设置环境光的颜色
	light->setAmbient(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	//设置散射光的颜色
	light->setDiffuse(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));

	//设置恒衰减指数
	light->setConstantAttenuation(1.0f);
	//设置线性衰减指数
	light->setLinearAttenuation(0.0f);
	//设置二次方衰减指数
	light->setQuadraticAttenuation(0.0f);

	//创建光源
	osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource();
	lightSource->setLight(light.get());

	osg::ref_ptr<osg::Group> lightRoot = new osg::Group();
	stateset = lightRoot->getOrCreateStateSet();
	lightRoot->addChild(node);
	lightRoot->addChild(lightSource.get());

	return lightRoot.get();
}

void setupProperties(osgText::Text& textObject, osgText::Font* font, float size, const osg::Vec3& pos, osg::Vec4 color)

{

	textObject.setFont(font);//

	textObject.setCharacterSize(size);//字体大小
	textObject.setPosition(pos);
	textObject.setColor(color);
	textObject.setAlignment(osgText::Text::CENTER_BOTTOM);//文字显示方向
	//textObject.setAxisAlignment(osgText::Text::SCREEN);//获取文字对称成方式正对屏幕方向
	//textObject.setCharacterSizeMode(osgText::Text::SCREEN_COORDS);//跟随视角不断变化，离物体越远，文字越大
	textObject.setAutoRotateToScreen(true);//跟随视角不断变化，但离物体越远，文字越小，和现实当中像类似
	//textObject.setBackdropType(osgText::Text::OUTLINE);//对文字进行描边
	//textObject.setBackdropColor(osg::Vec4(1.0, 0.0, 0.0, 1.0));//描边颜色
	//textObject.setDrawMode(osgText::Text::TEXT | osgText::Text::BOUNDINGBOX);//添加文字边框

	textObject.setAxisAlignment(osgText::Text::XZ_PLANE);//获取文字对称成方式



}
void createContent(osgText::Text& textObject, const char* string)

{

	int requiredSize = mbstowcs(NULL, string, 0);//如果mbstowcs第一参数为NULL那么返回字符串的数目

	wchar_t* wText = new wchar_t[requiredSize + 1];

	mbstowcs(wText, string, requiredSize + 1);//由char转换成wchar类型

	textObject.setText(wText);

	delete wText;

}
void createRope(float z,float radius,float step, osg::Geode *geode)
{
	//
	if (!geode)
	{
		geode = new osg::Geode();
	}
	geode->removeDrawables(0, geode->getNumDrawables());
	//半径
	//float radius = 10;
	//高度
	float height = 1.6f;
	//精细度
	osg::TessellationHints* hints1 = new osg::TessellationHints();
	//设置精细度
	hints1->setDetailRatio(2.0f);
	float step_d = step;
	if (step < radius)
	{
		step_d = radius;
	}
	//创建球体
	for (float i = 0; i < abs(z); i += step_d)
	{
		osg::Sphere *sphere = new osg::Sphere(osg::Vec3(0.0f, 0.0f, -i), radius);
		osg::ShapeDrawable *draw1 = new osg::ShapeDrawable(sphere, hints1);
		draw1->setColor(osg::Vec4(1, 0, 0, 0));
		geode->addDrawable(draw1);
	}
	
}
osg::Geode*	createGripper(float size)
{
	//
	osg::Geode *geode = new osg::Geode();
	osg::TessellationHints* hints1 = new osg::TessellationHints();
	//设置精细度
	hints1->setDetailRatio(2.0f);
	osg::Sphere *sphere = new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0), 400*size);
	osg::Sphere *sphere1 = new osg::Sphere(osg::Vec3(800*size, 0.0f, 0), 200*size);
	osg::Sphere *sphere2 = new osg::Sphere(osg::Vec3(-800 * size, 0.0f, 0), 200 * size);
	osg::Sphere *sphere3 = new osg::Sphere(osg::Vec3(0.0f, 800 * size, 0), 200 * size);
	osg::Sphere *sphere4 = new osg::Sphere(osg::Vec3(0.0f, -800 * size, 0), 200 * size);
	osg::ShapeDrawable *draw1 = new osg::ShapeDrawable(sphere, hints1);
	osg::ShapeDrawable *draw2 = new osg::ShapeDrawable(sphere1, hints1);
	osg::ShapeDrawable *draw3 = new osg::ShapeDrawable(sphere2, hints1);
	osg::ShapeDrawable *draw4 = new osg::ShapeDrawable(sphere3, hints1);
	osg::ShapeDrawable *draw5 = new osg::ShapeDrawable(sphere4, hints1);
	osg::ShapeDrawable *sd0 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(550 * size, 0, 0), 500 * size, 200 * size, 200 * size));
	osg::ShapeDrawable *sd1 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(-550 * size, 0, 0), 500 * size, 200 * size, 200 * size));
	osg::ShapeDrawable *sd2 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 550 * size, 0), 200 * size, 500 * size, 200 * size));
	osg::ShapeDrawable *sd3 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, -500 * size, 0), 200 * size, 500 * size, 200 * size));
	geode->addDrawable(draw1);
	geode->addDrawable(draw2);
	geode->addDrawable(draw3);
	geode->addDrawable(draw4);
	geode->addDrawable(draw5);
	geode->addDrawable(sd0);
	geode->addDrawable(sd1);
	geode->addDrawable(sd2);
	geode->addDrawable(sd3);
	osg::ShapeDrawable *sd4 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(800 * size, 0, -400 * size), 200 * size, 200 * size, 500 * size));
	osg::ShapeDrawable *sd5 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(-800 * size, 0, -400 * size), 200 * size, 200 * size, 500 * size));
	osg::ShapeDrawable *sd6 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 800 * size, -400 * size), 200 * size, 200 * size, 500 * size));
	osg::ShapeDrawable *sd7 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, -800 * size, -400 * size), 200 * size, 200 * size, 500 * size));
	geode->addDrawable(sd4);
	geode->addDrawable(sd5);
	geode->addDrawable(sd6);
	geode->addDrawable(sd7);
	osg::Sphere *sphere5 = new osg::Sphere(osg::Vec3(800 * size, 0, -800 * size), 200 * size);
	osg::Sphere *sphere6 = new osg::Sphere(osg::Vec3(-800 * size, 0, -800 * size), 200 * size);
	osg::Sphere *sphere7 = new osg::Sphere(osg::Vec3(0, 800 * size, -800 * size), 200 * size);
	osg::Sphere *sphere8 = new osg::Sphere(osg::Vec3(0, -800 * size, -800 * size), 200 * size);
	osg::ShapeDrawable *draw6 = new osg::ShapeDrawable(sphere5, hints1);
	osg::ShapeDrawable *draw7 = new osg::ShapeDrawable(sphere6, hints1);
	osg::ShapeDrawable *draw8 = new osg::ShapeDrawable(sphere7, hints1);
	osg::ShapeDrawable *draw9 = new osg::ShapeDrawable(sphere8, hints1);
	geode->addDrawable(draw6);
	geode->addDrawable(draw7);
	geode->addDrawable(draw8);
	geode->addDrawable(draw9);
	//抓手片
	osg::ref_ptr<osg::Vec3Array> Points= new osg::Vec3Array();;
	Points->push_back(osg::Vec3(400*sqrt(6)*size, 0, 0));
	Points->push_back(osg::Vec3(0, 400*sqrt(2)*size, 0));
	Points->push_back(osg::Vec3(0, -400*sqrt(2)*size,0));
	osg::ref_ptr<osg::Vec4Array>color = new osg::Vec4Array();
	color->push_back(osg::Vec4(1, 0.5, 0.5, 1.0f));
	
	osg::ref_ptr<osg::Geode> trangle0 = CreateModule_DelaunayTriangulator(Points, color, 0);
    //trangle0->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	//trangle0->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	float num_r_y= M_PI / 3.5;

	//osg::Geode *dygeo = new osg::Geode();
	//dygeo->addDrawable(sd4);
	osg::ref_ptr<osg::MatrixTransform> tr0 = new osg::MatrixTransform();
	tr0->addChild(trangle0);
	Eigen::Matrix4d mat0;
	mat0 = Rot_z(-M_PI/4)*Rot_y(M_PI - num_r_y);
	mat0(2, 3) = -800 * size;
	mat0(0, 3) = 300*sqrt(2)*size;
	mat0(1, 3) = -300*sqrt(2)*size;
	tr0->setMatrix(eigen2osg(mat0));
	//
	osg::ref_ptr<osg::MatrixTransform> tr1 = new osg::MatrixTransform();
	tr1->addChild(trangle0);
	Eigen::Matrix4d mat1;
	mat1 = Rot_z(-M_PI / 4)*Rot_y(num_r_y);
	mat1(2, 3) = -800 * size;
	mat1(0, 3) = -300 * sqrt(2)*size;
	mat1(1, 3) = 300 * sqrt(2)*size;
	tr1->setMatrix(eigen2osg(mat1));
	//
	osg::ref_ptr<osg::MatrixTransform> tr2 = new osg::MatrixTransform();
	tr2->addChild(trangle0);
	Eigen::Matrix4d mat2;
	mat2 = Rot_z(M_PI / 4)*Rot_y(M_PI - num_r_y);
	mat2(2, 3) = -800 * size;
	mat2(0, 3) = 300 * sqrt(2)*size;
	mat2(1, 3) = 300 * sqrt(2)*size;
	tr2->setMatrix(eigen2osg(mat2));
	//
	osg::ref_ptr<osg::MatrixTransform> tr3 = new osg::MatrixTransform();
	tr3->addChild(trangle0);
	Eigen::Matrix4d mat3;
	mat3 = Rot_z(M_PI / 4)*Rot_y(num_r_y);
	mat3(2, 3) = -800 * size;
	mat3(0, 3) = -300 * sqrt(2)*size;
	mat3(1, 3) = -300 * sqrt(2)*size;
	tr3->setMatrix(eigen2osg(mat3));
	//
	geode->addChild(tr0);
	geode->addChild(tr1);
	geode->addChild(tr2);
	geode->addChild(tr3);
	return geode;
}
osg::Geode* createBox(float x,float y,float z)
{
	osg::Geode *geode = new osg::Geode();
	osg::ShapeDrawable *sd = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), x,y,z));
	geode->addDrawable(sd);
	
	sd->setColor(osg::Vec4(0.5, 0.5, 0.5, 0.05f));
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	return geode;
}
osg::Geode* createSphere(float r)
{
	//
	osg::Geode *geode = new osg::Geode();
	//半径
	float radius = r;
	//高度
	float height = 1.6f;

	//精细度
	osg::TessellationHints* hints1 = new osg::TessellationHints();
	//设置精细度
	hints1->setDetailRatio(2.0f);

	//创建球体
	osg::Sphere *sphere = new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), radius);
	osg::ShapeDrawable *draw1 = new osg::ShapeDrawable(sphere, hints1);

	geode->addDrawable(draw1);
	
	draw1->setColor(osg::Vec4(1,0, 0, 0));
	//geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	//geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	
	return geode;
}
osg::Geode* createCrane(float size)
{
	osg::Geode *geode = new osg::Geode();
	osg::ShapeDrawable *sd0 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), 1000, 50000, 1000));
	geode->addDrawable(sd0);
	osg::Geode *geode1 = new osg::Geode();
	osg::ShapeDrawable *sd1 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, -1000), 1000, 2000, 2000));
	geode1->addDrawable(sd1);
	osg::ref_ptr<osg::MatrixTransform> tr0 = new osg::MatrixTransform();
	osg::ref_ptr<osg::MatrixTransform> tr1 = new osg::MatrixTransform();
	tr0->addChild(geode);
	tr1->addChild(geode1);
	tr0->addChild(tr1);
	osg::ref_ptr <osg::Geode> Crane = new osg::Geode();
	Crane->addChild(tr0);
	return Crane;
}
osg::Matrixd eigen2osg(Eigen::Matrix4d mat)
{
	osg::Matrixd osg_mat(mat(0, 0), mat(1, 0), mat(2, 0), mat(3, 0),
	                	 mat(0, 1), mat(1, 1), mat(2, 1), mat(3, 1),
		                 mat(0, 2), mat(1, 2), mat(2, 2), mat(3, 2),
		                 mat(0, 3), mat(1, 3), mat(2, 3), mat(3, 3)
	                    );
	return osg_mat;
}
std::string WORDToString(WORD w)
{
	char tmpbuff[16];
	sprintf(tmpbuff, "%d", w);
	std::string res = tmpbuff;
	return res;
}
osg::ref_ptr<osgText::Text> createText(float size, osg::Vec3 pos, std::string name, osg::Vec4 color)
{
	setlocale(LC_ALL, ".936");//　配置地域化信息

	const char* titleString = name.c_str();

	osgText::Font* fontHei = osgText::readFontFile("Fonts/simhei.ttf");

	osg::ref_ptr<osgText::Text> title = new osgText::Text;

	setupProperties(*title, fontHei, size, pos, color);

	createContent(*title, titleString);

	//osg::ref_ptr<osg::Geode> geode = new osg::Geode;

	//geode->addDrawable(title.get());

	return title;

}
bool  TaskComp(const SchedulingTask &a, const SchedulingTask &b) {
	if (a.current_Value > b.current_Value) {
		return true;
	}
	else {
		return false;
	}
}
bool  GroundComp(const BoxGround *a, const BoxGround *b) {
	if (a->level > b->level) {
		return true;
	}
	else {
		return false;
	}
}



testosgQt::testosgQt(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	
	resize(800, 600);

	QHBoxLayout *pLayout = new QHBoxLayout(this->centralWidget());
	pLayout->setMargin(0);

	pOsgW = new osgQOpenGLWidget();
	pOsgW->setFixedSize(1500, 600);
	pLayout->addWidget(pOsgW);
	connect(pOsgW, SIGNAL(initialized()), this, SLOT(initOsg()));
	connect(ui.pushButton1, SIGNAL(clicked()), this, SLOT(changeOpen()));
	connect(ui.pushButton2, SIGNAL(clicked()), this, SLOT(changeClose())); 
	connect(ui.CurrentImg, SIGNAL(clicked()), this, SLOT(GetCurrentPointCloud()));
	connect(ui.showPT, SIGNAL(clicked()), this, SLOT(GetPointCloud()));
	connect(ui.imgnum, SIGNAL(clicked()), this, SLOT(GetPointCloud2()));
	connect(ui.mapbuild, SIGNAL(clicked()), this, SLOT(calCalib()));
	connect(ui.Offline, SIGNAL(clicked()), this, SLOT(getOffLineMap()));
	timer = new QTimer(this);
	Open_timer = new QTimer(this);
	Close_timer = new QTimer(this);
	Motion_timer = new QTimer(this);
	Sim_timer = new QTimer(this);
	
	connect(ui.Opentimer, SIGNAL(clicked()), this, SLOT(OpenGripper()));
	connect(ui.Closetimer, SIGNAL(clicked()), this, SLOT(CloseGripper()));
	connect(Open_timer, SIGNAL(timeout()), this, SLOT(ExcuteGripperOpen()));
	connect(Close_timer, SIGNAL(timeout()), this, SLOT(ExcuteGripperClose()));
	connect(Motion_timer, SIGNAL(timeout()), this, SLOT(SimulationMotion()));
	connect(Sim_timer, SIGNAL(timeout()), this, SLOT(ExcuteTask1()));
	connect(ui.movetopose, SIGNAL(clicked()), this, SLOT(startMotion()));
	connect(ui.addBoxOK, SIGNAL(clicked()), this, SLOT(saveGroundBox()));
	connect(ui.SIM, SIGNAL(clicked()), this, SLOT(startSim()));
	connect(ui.offlineMap, SIGNAL(clicked()), this, SLOT(openMap()));
	//connect(ui.CreatePath, SIGNAL(clicked()), this, SLOT(CreatePath()));
	//timer->start(1000);
	SYSTEMTIME now;
	GetLocalTime(&now);
	//auto pix = ui.label->pixmap();
	QImage img;
	QDir *folder = new QDir;
	
	bool exist = folder->exists(QString(QApplication::applicationDirPath() + "\\img"));
	if (exist)
	{
	}
	else //如果不存在，创建文件夹
	{
		//创建文件夹
		bool ok = folder->mkpath(QString(QApplication::applicationDirPath()+"\\img"));
	}
	
	if(img.load(QString("F:\\testdata\\image.jpg")))
	{
		QPixmap pix;
		pix.fromImage(img);
		std::string str;
		str = WORDToString(now.wYear);
		str+= WORDToString(now.wMonth);
		str+= WORDToString(now.wDay);
		str += "-";
		str+= WORDToString(now.wHour);
		str += WORDToString(now.wMinute);
		str += WORDToString(now.wSecond);
		QString qStr = QString::fromStdString(QApplication::applicationDirPath().toStdString() +"\\img\\" +str+".jpg");
		img.save(qStr, "JPG", 100);
		//pix.save(QString("F:\\testdata\\aaa.jpg"),"JPG",100);
	}
	mapcloud = new open3d::geometry::PointCloud();
	//connect(, SIGNAL(clecked()), this, SLOT(initOsg()));
	//connect(ui.centralWidget., SIGNAL(clecked()), this, SLOT(initOsg()));

}
void testosgQt::changeOpen()
{
	//osgViewer::Viewer *pViewer = ((osgQOpenGLWidget *)sender())->getOsgViewer();
	osg::Vec3f vPosEye;
	osg::Vec3f vCenter;
	osg::Vec3f vUp;
	Viewer->getCamera()->getViewMatrixAsLookAt(vPosEye, vCenter, vUp);
	
	//contrel = nullptr;
    contrel = new osgGA::TrackballManipulator();
	//Viewer->setCameraManipulator(contrel);
	auto mat2 = Viewer->getCamera()->getViewMatrix();
	contrel->setByInverseMatrix(mat2);
	auto mat=contrel->getMatrix();
	//contrel->setTransformation(m_vPosEye, location, vUp);
	if (!Viewer->getCameraManipulator())
	{
		Viewer->setCameraManipulator(contrel);
		contrel->setByInverseMatrix(mat2);
	}
	//View
}
void testosgQt::changeClose()
{
	//osgViewer::Viewer *pViewer = ((osgQOpenGLWidget *)sender())->getOsgViewer();
	//Viewer->setCameraManipulator(nullptr);
	//return;
	
	Viewer->setCameraManipulator(nullptr);
	osg::Vec3d eye, center, up;
	eye = osg::Vec3d(10000, 0, 0.0);
	center = osg::Vec3d(0, 0.0, 0.0);
	up = osg::Vec3d(0, 0, 1);
	Viewer->getCamera()->setViewMatrixAsLookAt(eye, center, up);
	
}
void testosgQt::CreatePath()
{
	getMapPath(8, 1, 5000, 0, 5000, 0);

	osg::Geode *geode = new osg::Geode();
	//半径
	float radius = 1000;
	//高度
	float height = 1.6f;

	//精细度
	osg::TessellationHints* hints1 = new osg::TessellationHints();
	//设置精细度
	hints1->setDetailRatio(2.0f);

	//创建球体
	for (int i = 0; i < path2.size(); i++)
	{
		float x = path2[i].x();
		float y = path2[i].y();
		osg::Sphere *sphere = new osg::Sphere(osg::Vec3(x, y, 0.0f), radius);
		osg::ShapeDrawable *draw1 = new osg::ShapeDrawable(sphere, hints1);
		draw1->setColor(osg::Vec4(1, 0, 0, 0));
		geode->addDrawable(draw1);
	}
	root->addChild(geode);

}
void testosgQt::startSim()
{
	Sim_timer->start(10);
}
void testosgQt::startMotion()
{
	Motion_timer->start(10);
}
void testosgQt::SimulationMotion()
{
	double x = ui.poseX->value();
	double y = ui.poseY->value();
	double z = ui.poseZ->value();
	double current_x = current_cranepose1.x();
	double current_y = current_cranepose1.y();
	double current_z = current_cranepose1.z();
	Eigen::Vector3d length = Eigen::Vector3d(x-current_x,y-current_y,z-current_z);
	if (length.norm() > 150)
	{
		length.normalize();
		int i_x=0, i_y=0, i_z=0;
		if (length.x() < 0)
		{
			i_x = -1;
		}
		if (length.y() < 0)
		{
			i_y = -1;
		}
		if (length.z() < 0)
		{
			i_z = -1;
		}
		if (length.x() > 0)
		{
			i_x = 1;
		}
		if (length.y() > 0)
		{
			i_y = 1;
		}
		if (length.z() > 0)
		{
			i_z = 1;
		}
		updateCrane1(current_x + 100*i_x, current_y + 100*i_y , current_z + 100*i_z );
	}
	else
	{
		updateCrane1(x, y, z);
		Motion_timer->stop();
	}
	//
}
void testosgQt::SimMotion(double x,double y,double z)
{
	double current_x = current_cranepose1.x();
	double current_y = current_cranepose1.y();
	double current_z = current_cranepose1.z();
	Eigen::Vector3d length = Eigen::Vector3d(x - current_x, y - current_y, z - current_z);
	if (length.norm() > 150)
	{
		crane1_state = true;
		length.normalize();
		float i_x = 0, i_y = 0, i_z = 0;
		if (length.x() < 0)
		{
			i_x = -1;
		}
		if (length.y() < 0)
		{
			i_y = -1;
		}
		if (length.z() < 0)
		{
			i_z = -1;
		}
		if (length.x() > 0)
		{
			i_x = 1;
		}
		if (length.y() > 0)
		{
			i_y = 1;
		}
		if (length.z() > 0)
		{
			i_z = 1;
		}
		if (abs(x - current_x) < 100)
		{
			i_x  = (x - current_x) / 100;
		}
		if (abs(y - current_y) < 100)
		{
			i_y = (y - current_y) / 100;
		}
		if (abs(z - current_z) < 50)
		{
			i_z = (z - current_z) / 50;
		}
		
		updateCrane1(current_x + 100 * i_x, current_y + 100 * i_y, current_z + 50 * i_z);
	}
	else
	{
		updateCrane1(x, y, z);
		crane1_state = false;
	}
	//
}
void testosgQt::adjustGripper(osg::Geode *geode,float angle)
{
	float num_r_y = angle;
	int count = 0;
	Eigen::Matrix4d mat0;
	for (int i=0; i < geode->getNumChildren(); i++)
	{
		auto node=geode->getChild(i);
		if (node)
		{
			osg::MatrixTransform *tran = dynamic_cast<osg::MatrixTransform*> (node);
			if (tran)
			{
				switch (count)
				{
				case(0):
					mat0 = Rot_z(-M_PI / 4)*Rot_y(M_PI - num_r_y);
					mat0(2, 3) = tran->getMatrix().getTrans().z();
					mat0(0, 3) = tran->getMatrix().getTrans().x();
					mat0(1, 3) = tran->getMatrix().getTrans().y();
					tran->setMatrix(eigen2osg(mat0));
					count++;
					break;
					//
				case(1):
					mat0 = Rot_z(-M_PI / 4)*Rot_y(num_r_y);
					mat0(2, 3) = tran->getMatrix().getTrans().z();
					mat0(0, 3) = tran->getMatrix().getTrans().x();
					mat0(1, 3) = tran->getMatrix().getTrans().y();
					tran->setMatrix(eigen2osg(mat0));
					count++;
					break;
					//
				case(2):
					mat0 = Rot_z(M_PI / 4)*Rot_y(M_PI - num_r_y);
					mat0(2, 3) = tran->getMatrix().getTrans().z();
					mat0(0, 3) = tran->getMatrix().getTrans().x();
					mat0(1, 3) = tran->getMatrix().getTrans().y();
					tran->setMatrix(eigen2osg(mat0));
					count++;
					break;
					//
				case(3):
					mat0 = Rot_z(M_PI / 4)*Rot_y(num_r_y);
					mat0(2, 3) = tran->getMatrix().getTrans().z();
					mat0(0, 3) = tran->getMatrix().getTrans().x();
					mat0(1, 3) = tran->getMatrix().getTrans().y();
					tran->setMatrix(eigen2osg(mat0));
					count++;
					break;
				default:
					break;
				}
					
			}
		}
		else
		{
			break;
		}
	}




}
void testosgQt::OpenGripper()
{
	Open_timer->start(100);
	timecount = 0;
}
void testosgQt::CloseGripper()
{
	Close_timer->start(100);
	timecount = 0;
}
void testosgQt::ExcuteGripperOpen()
{
	if (!Gripper1_state)
	{
		float angle = float(timecount) / 10 * (M_PI*(0.2143)) + M_PI * 1 / 3.5;
		adjustGripper(Gripper1, angle);
		timecount++;
		if (timecount > 10)
		{
			Open_timer->stop();
			timecount = 0;
			Gripper1_state = 1;
		}
	}
	else
	{
		Open_timer->stop();
	}

}
void testosgQt::ExcuteGripperClose()
{
	if (Gripper1_state)
	{
		float angle = float(timecount) / 10 * (M_PI*(-0.2143)) + M_PI * 1 / 2;
		adjustGripper(Gripper1, angle);
		timecount++;
		if (timecount > 10)
		{
			Close_timer->stop();
			timecount = 0;
			Gripper1_state = 0;
		}
	}
	else
	{
		Close_timer->stop();
	}
}
void testosgQt::initCrane1(float size)
{
	osg::Geode *geode = new osg::Geode();
	osg::ShapeDrawable *sd0 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), 1000*size, 36000*size, 1000*size));
	geode->addDrawable(sd0);
	sd0->setColor(osg::Vec4(0.6, 0.5, 0.5, 1.0f));
	osg::Geode *geode1 = new osg::Geode();
	osg::ShapeDrawable *sd1 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), 1000*size, 2000*size, 2000*size));
	geode1->addDrawable(sd1);
	sd1->setColor(osg::Vec4(1, 1, 0, 1.0f));
	tr_x1 = new osg::MatrixTransform();
	tr_y1 = new osg::MatrixTransform();
	tr_x1->addChild(geode);
	tr_y1->addChild(geode1);
	tr_x1->addChild(tr_y1);
    crane1 = new osg::Geode();
    tr_z1 = new osg::MatrixTransform();
	Gripper1=createGripper(size);
	tr_z1->addChild(Gripper1);
	tr_y1->addChild(tr_z1);
	rope1 = new osg::Geode();
	createRope(40000, 100*size, 200*size, rope1);
	tr_y1->addChild(rope1);
    crane1->addChild(tr_x1);
	crane_size1 = size;
	updateCrane1(0, 0, -500);
	current_cranepose1 = Eigen::Vector3d(0, 0, -500);
}
void testosgQt::initCrane2(float size)
{
	osg::Geode *geode = new osg::Geode();
	osg::ShapeDrawable *sd0 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), 1000 * size, 36000 * size, 1000 * size));
	geode->addDrawable(sd0);
	sd0->setColor(osg::Vec4(0.6, 0.5, 0.5, 1.0f));
	osg::Geode *geode1 = new osg::Geode();
	osg::ShapeDrawable *sd1 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), 1000 * size, 2000 * size, 2000 * size));
	geode1->addDrawable(sd1);
	sd1->setColor(osg::Vec4(1, 1, 0, 1.0f));
	tr_x2 = new osg::MatrixTransform();
	tr_y2 = new osg::MatrixTransform();
	tr_x2->addChild(geode);
	tr_y2->addChild(geode1);
	tr_x2->addChild(tr_y2);
	crane2 = new osg::Geode();
	tr_z2 = new osg::MatrixTransform();
	Gripper2 = createGripper(size);
	tr_z2->addChild(Gripper2);
	tr_y2->addChild(tr_z2);
	rope2 = new osg::Geode();
	createRope(40000, 100 * size, 200 * size, rope2);
	tr_y2->addChild(rope2);
	crane2->addChild(tr_x2);
	crane_size2 = size;
	updateCrane2(50000, 0, -500);
	current_cranepose2 = Eigen::Vector3d(50000, 0, -500);
}
void testosgQt::updateCrane1(float x,float y,float z)
{
	Eigen::Matrix4d mat_x;
	mat_x << 1, 0, 0, x,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	tr_x1->setMatrix(eigen2osg(mat_x));
	Eigen::Matrix4d mat_y;
	mat_y << 1, 0, 0, 0,
		0, 1, 0, y,
		0, 0, 1, 0,
		0, 0, 0, 1;
	tr_y1->setMatrix(eigen2osg(mat_y));
	Eigen::Matrix4d mat_z;
	mat_z << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, z,
		0, 0, 0, 1;
	tr_z1->setMatrix(eigen2osg(mat_z));
	//createRope(abs(z), 100 * crane_size1, 200 * crane_size1, rope1);
	for (int i = 0; i<rope1->getNumDrawables(); i++)
	{
		auto draw=rope1->getDrawable(i);
		if (i < abs(z) / (200 * crane_size1))
		{
			draw->setNodeMask(1);
		}
		else
		{
			draw->setNodeMask(0);
		}
	}
	current_cranepose1 = Eigen::Vector3d(x, y, z);
}
void testosgQt::updateCrane2(float x, float y, float z)
{
	Eigen::Matrix4d mat_x;
	mat_x << 1, 0, 0, x,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	tr_x2->setMatrix(eigen2osg(mat_x));
	Eigen::Matrix4d mat_y;
	mat_y << 1, 0, 0, 0,
		0, 1, 0, y,
		0, 0, 1, 0,
		0, 0, 0, 1;
	tr_y2->setMatrix(eigen2osg(mat_y));
	Eigen::Matrix4d mat_z;
	mat_z << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, z,
		0, 0, 0, 1;
	tr_z2->setMatrix(eigen2osg(mat_z));
	//createRope(abs(z), 100 * crane_size1, 200 * crane_size1, rope1);
	for (int i = 0; i < rope2->getNumDrawables(); i++)
	{
		auto draw = rope2->getDrawable(i);
		if (i < abs(z) / (200 * crane_size2))
		{
			draw->setNodeMask(1);
		}
		else
		{
			draw->setNodeMask(0);
		}
	}
	current_cranepose2 = Eigen::Vector3d(x, y, z);
}
//x y z font_size
osg::Geode* testosgQt::makeCoordinate(float a_x, float a_y, float a_z, float font_size)
{
	osg::ref_ptr<osg::Sphere> pSphereShape = new osg::Sphere(osg::Vec3(0, 0, 0), 0.1f);
	osg::ref_ptr<osg::ShapeDrawable> pShapeDrawable = new osg::ShapeDrawable(pSphereShape.get());
	pShapeDrawable->setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));

	//创建保存几何信息的对象
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();

	//创建四个顶点
	osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array();
	v->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	v->push_back(osg::Vec3(a_x, 0.0f, 0.0f));

	v->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	v->push_back(osg::Vec3(0.0f, a_y, 0.0f));
	v->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));

	v->push_back(osg::Vec3(0.0f, 0.0f, a_z));
	geom->setVertexArray(v.get());


	osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array();
	c->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	c->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));

	c->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
	c->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
	c->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));

	c->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
	geom->setColorArray(c.get());
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);


	//xyz
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 2, 2));
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 4, 2));


	osg::ref_ptr<osgText::Text> pTextXAuxis1 = new osgText::Text;
	pTextXAuxis1->setText(L"X");
	pTextXAuxis1->setFont("Fonts/simhei.ttf");

	pTextXAuxis1->setAxisAlignment(osgText::Text::SCREEN);
	pTextXAuxis1->setCharacterSize(font_size);
	pTextXAuxis1->setPosition(osg::Vec3(a_x, 0.0f, 0.0f));

	osg::ref_ptr<osgText::Text> pTextYAuxis1 = new osgText::Text;
	pTextYAuxis1->setText(L"Y");
	pTextYAuxis1->setFont("Fonts/simhei.ttf");

	pTextYAuxis1->setAxisAlignment(osgText::Text::SCREEN);
	pTextYAuxis1->setCharacterSize(font_size);
	pTextYAuxis1->setPosition(osg::Vec3(0.0f, a_y, 0.0f));

	osg::ref_ptr<osgText::Text> pTextZAuxis1 = new osgText::Text;
	pTextZAuxis1->setText(L"Z");
	pTextZAuxis1->setFont("Fonts/simhei.ttf");

	pTextZAuxis1->setAxisAlignment(osgText::Text::SCREEN);
	pTextZAuxis1->setCharacterSize(font_size);
	pTextZAuxis1->setPosition(osg::Vec3(0.0f, 0.0f, a_z));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setAttribute(new osg::LineWidth(3.0), osg::StateAttribute::ON);

	geode->addDrawable(pShapeDrawable.get());
	geode->addDrawable(geom.get());
	geode->addDrawable(pTextXAuxis1.get());

	geode->addDrawable(pTextYAuxis1.get());
	geode->addDrawable(pTextZAuxis1.get());

	return geode.release();
}
void testosgQt::initOsg()
{
	Viewer= ((osgQOpenGLWidget *)sender())->getOsgViewer();
	osgViewer::Viewer *pViewer = ((osgQOpenGLWidget *)sender())->getOsgViewer();
	contrel = new osgGA::TrackballManipulator();
	root = new osg::Group;
    osg::Geode* geo=makeCoordinate(1, 1, 1,1);
    root->addChild(geo);
	auto sphere = createSphere(10);
	auto Box = createBox(50000,36000,40000);
	//
	//initCrane1(1);
	//root->addChild(crane1);
	//initCrane2(1);
   // root->addChild(crane2);
	//crane1->setNodeMask(0);
	//crane2->setNodeMask(0);
	//adjustGripper(Gripper1, M_PI / 3.5);
	
	osg::ref_ptr<osg::MatrixTransform> tr_Box = new osg::MatrixTransform();
	Eigen::Matrix4d mat2;
	mat2 << 1, 0, 0, 25000,
		0, 1, 0, 0,
		0, 0, 1, -20000,
		0, 0, 0, 1;
	tr_Box->setMatrix(eigen2osg(mat2));

	tr_Box->addChild(Box);
	root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	root->getOrCreateStateSet()->setMode(GL_LIGHT0, osg::StateAttribute::ON);
	root->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);
	root->getOrCreateStateSet()->setMode(GL_LIGHT2, osg::StateAttribute::ON);
	root->getOrCreateStateSet()->setMode(GL_LIGHT3, osg::StateAttribute::ON);
	root->getOrCreateStateSet()->setMode(GL_LIGHT4, osg::StateAttribute::ON);
	root->getOrCreateStateSet()->setMode(GL_LIGHT5, osg::StateAttribute::ON);
	root->getOrCreateStateSet()->setMode(GL_LIGHT6, osg::StateAttribute::ON);
	root->getOrCreateStateSet()->setMode(GL_LIGHT7, osg::StateAttribute::ON);
	//root->addChild(tr_Box);
	//addGroundBox(10000, 10000, 10000,"A",osg::Vec4(1,0,0,0.1),osg::Vec4(0,1,0,0.5));
	//testTriangulator();
    pViewer->setSceneData(root.get());
	pViewer->addEventHandler(new osgViewer::StatsHandler());
	pViewer->addEventHandler(new osgViewer::WindowSizeHandler());
	pViewer->addEventHandler(new osgGA::StateSetManipulator(pViewer->getCamera()->getOrCreateStateSet()));
	pViewer->addEventHandler(new PickHandler());
	//SimSetGround();
	//addTask();
	//init_sdk();
}

void testosgQt::saveGroundBox()
{
	auto center=current_ground->GetDraggerMatrix().getTrans();
	int i = 0;
	switch (i)
	{
	default:
		break;
	}


}
void testosgQt::CalTaskGround1()
{
	//std::vector<SchedulingTask> Task_group1;
	for (int i=0;i< Task_group1.size();i++)
	{
		Task_group1[i].current_Value = Task_group1[i].current_Value + Task_group1[i].Frequency;
	}
	std::sort(Task_group1.begin(), Task_group1.end(), TaskComp);
	if (Task_group1[0].current_Value >= Task_group1[0].overflow_Value)
	{
		Task_group1[0].current_Value=Task_group1[0].current_Value - Task_group1[0].overflow_Value;
	}
	std::sort(Task_group1[0].Grab_Ground.begin(), Task_group1[0].Grab_Ground.end(), GroundComp);
	std::sort(Task_group1[0].Place_Ground.begin(), Task_group1[0].Place_Ground.end(), GroundComp);
	current_Grab_Ground1 = Task_group1[0].Grab_Ground[0];
	current_Place_Ground1 = Task_group1[0].Place_Ground[0];
	CalTaskPath();
}
void testosgQt::CalTaskPath()
{
	movePath1.clear();
	Eigen::Vector3d point1=current_cranepose1;
	if (current_cranepose1.z() < safe_height)
	{
		point1.z() = safe_height;
	}
	Eigen::Vector3d point2(current_Grab_Ground1->points[0][0], current_Grab_Ground1->points[0][1], point1.z());
	Eigen::Vector3d point3(current_Grab_Ground1->points[0][0], current_Grab_Ground1->points[0][1], current_Grab_Ground1->points[0][2]);
	Eigen::Vector3d point4(current_Grab_Ground1->points[0][0], current_Grab_Ground1->points[0][1], current_Place_Ground1->points[0][2]);
	if (current_Place_Ground1->points[0][2] < safe_height)
	{
		point4.z() = safe_height;
	}
	Eigen::Vector3d point5(current_Place_Ground1->points[0][0], current_Place_Ground1->points[0][1], point4.z());
	Eigen::Vector3d point6(current_Place_Ground1->points[0][0], current_Place_Ground1->points[0][1], current_Place_Ground1->points[0][2]);
	movePath1.push_back(point1);
	movePath1.push_back(point2);
	movePath1.push_back(point3);
	movePath1.push_back(point4);
	movePath1.push_back(point5);
	movePath1.push_back(point6);
}
void testosgQt::CalTaskGround2()
{
	//std::vector<SchedulingTask> Task_group2;
	for (auto it : Task_group2)
	{
		it.current_Value = it.current_Value + it.Frequency;
	}
	std::sort(Task_group2.begin(), Task_group2.end(), TaskComp);
	if (Task_group2[0].current_Value > Task_group2[0].overflow_Value)
	{
		Task_group2[0].current_Value = Task_group2[0].current_Value - Task_group2[0].overflow_Value;
	}
	std::sort(Task_group2[0].Grab_Ground.begin(), Task_group2[0].Grab_Ground.end(), GroundComp);
	std::sort(Task_group2[0].Place_Ground.begin(), Task_group2[0].Place_Ground.end(), GroundComp);
	current_Grab_Ground2 = Task_group2[0].Grab_Ground[0];
	current_Place_Ground2 = Task_group2[0].Place_Ground[0];
}
void testosgQt::ExcuteTask1()
{
	
	    double x1 = movePath1[current_path_point].x();
		double y1 = movePath1[current_path_point].y();
		double z1 = movePath1[current_path_point].z();
		SimMotion(x1, y1, z1);
		if (crane1_state)
		{
			return;
		}
		else
		{
			current_path_point++;
			if (current_path_point > 5)
			{	
				CalTaskGround1();
	            updateAllBoxGround();
				current_path_point = 0;
			}
			return;
		}

}
void testosgQt::SimSetGround()
{
	osg::Vec4 color1;
	osg::Vec4 color2;
	for (int i = 0; i < 5; i++)
	{
       BoxGround *ground=new BoxGround;
	   ground->length_x = 5000;
	   ground->length_y = 5000;
	   ground->length_z = 5000;
	   ground->x = (i + 1) * 8000;
	   ground->y = -17500;
	   ground->z = -27500;
	   ground->Type = GarbageSlot;
	   std::string str = "A";
	   str+= std::to_string(i);
	   ground->name = str;
	   CalCatchPoints(ground);
	   updateBoxGround(ground);
	   Customer_Ground.push_back(ground);
	 
	   addGroundBox(ground->length_x, ground->length_y, ground->length_z, ground->name, osg::Vec4(1, 0, 0, 0.1), osg::Vec4(1, 0, 0, 1), osg::Vec3(ground->x, ground->y, ground->z));
	}
	for (int i = 0; i < 5; i++)
	{
		BoxGround *ground = new BoxGround;
		ground->length_x = 5000;
		ground->length_y = 5000;
		ground->length_z = 5000;
		ground->x = (i + 1) * 8000;
		ground->y =  0;
		ground->z = -27500;
		ground->Type = FermentationArea;
		std::string str = "B";
		str += std::to_string(i);
		ground->name = str;
		CalCatchPoints(ground);
		updateBoxGround(ground);
		Customer_Ground.push_back(ground);
		
		addGroundBox(ground->length_x, ground->length_y, ground->length_z, ground->name, osg::Vec4(1, 1, 0, 0.1), osg::Vec4(1, 1, 0, 1), osg::Vec3(ground->x, ground->y, ground->z));
	}
	for (int i = 0; i < 2; i++)
	{
		BoxGround *ground = new BoxGround;
		ground->length_x = 5000;
		ground->length_y = 5000;
		ground->length_z = 5000;
		ground->x = (i + 1) * 20000;
		ground->y = 17500;
		ground->z = -8000;
		ground->Type = BurnerPort;
		std::string str = "C";
		str += std::to_string(i);
		ground->name = str;
		CalCatchPoints(ground);
		updateBoxGround(ground);
		Customer_Ground.push_back(ground);
		addGroundBox(ground->length_x, ground->length_y, ground->length_z, ground->name, osg::Vec4(0, 1, 0, 0.1), osg::Vec4(0, 1, 0, 1),osg::Vec3(ground->x, ground->y, ground->z));
	}
}
void testosgQt::CalCatchPoints(BoxGround *box)
{
	/*
	for (int i = 0; i < 5; i++)
	{
		int x = rand() % 4000 + box->x - 2000;
		int y = rand() % 4000 + box->y - 2000;
		int z = rand() % 4000 + box->z - 2000;
		box->points[i][0] = x;
		box->points[i][1] = y;
		box->points[i][2] = z;
	}*/
	open3d::geometry::PointCloud* cloud=new open3d::geometry::PointCloud();
	osg::ref_ptr<osg::Vec3Array> PointGroup = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array>color= new osg::Vec4Array();;
	for (int i = 0; i < 3000; i++)
	{
		int x = rand() % 5000 + box->x - 2500;
		int y = rand() % 5000 + box->y - 2500;
		int z = rand() % 300 + box->z ;
		if (x > box->x)
		{
			z +=500;
		}
		if (y > box->y)
		{
			z += 500;
		}
		Eigen::Vector3d point(x,y,z);
		cloud->points_.push_back(point);
		PointGroup->push_back(osg::Vec3(x, y, z));
		color->push_back(osg::Vec4(1, 0, 0, 0.5));
	}
	Eigen::Vector3d grid(1000, 1000, 1000);
	getCatchingPoints(box, cloud, grid, true);
	
	auto geo = CreateModule_DelaunayTriangulator(PointGroup, color);
	root->addChild(geo);
}
void testosgQt::updateBoxGround(BoxGround *box)
{
	box->level = rand() % 5;
}
void testosgQt::updateAllBoxGround()
{
	std::default_random_engine e;
	std::uniform_int_distribution<int> u(0, 5); // 左闭右闭区间
	e.seed(time(0));
	for (int i=0;i<Customer_Ground.size();i++)
	{
		Customer_Ground[i]->level = u(e);
	}
}
void testosgQt::addTask()
{
	SchedulingTask task1;
	SchedulingTask task2;
	for (int i = 0; i < Customer_Ground.size(); i++)
	{
		switch (Customer_Ground[i]->Type)
		{
		case(GarbageSlot):
			task1.Grab_Ground.push_back(Customer_Ground[i]);

		break;
		case(FermentationArea):
			task2.Grab_Ground.push_back(Customer_Ground[i]);
			task1.Place_Ground.push_back(Customer_Ground[i]);
		break;
		case(BurnerPort):
			task2.Place_Ground.push_back(Customer_Ground[i]);
		break;
		default:
			break;
		}
	}
	task1.Frequency = 1;
	task1.current_Value = 1;
	task2.Frequency = 1;
	task1.overflow_Value = task1.Frequency * 2;
	task2.overflow_Value = task2.Frequency * 2;
	Task_group1.push_back(task1);
	Task_group1.push_back(task2);
	
	CalTaskGround1();
	updateAllBoxGround();
}
void testosgQt::addGroundBox(float x,float y,float z, std::string name, osg::Vec4 color1, osg::Vec4 color2,osg::Vec3 pose)
{
	osg::Geode *Box = new osg::Geode();
	osg::ShapeDrawable *sd = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), x, y, z));
	Box->addDrawable(sd);
	sd->setColor(color1);
	Box->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	Box->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	float size=10;
	if (x > y)
	{
		size = y/2;
	}
	else
	{
		size = x/2;
	}
	auto text = createText(size,osg::Vec3(x/2,y/2,z/2),name,color2);
	osg::ref_ptr<osg::MatrixTransform> tr_Box = new osg::MatrixTransform();
	auto model = new ModelShape(Box);
	current_ground = model;
	//model->EnableDragger();
	osg::Matrix mat;
	mat.setTrans(pose);
	tr_Box->setMatrix(mat);
	tr_Box->addChild(model);
	root->addChild(tr_Box);
	Box->addDrawable(text.get());
}
bool testosgQt::addmodel(std::string IDname, osg::ref_ptr<osg::Node> node)
{
	std::map<std::string, osg::ref_ptr<osg::Node>>::const_iterator model_map;
	model_map = this->label_map_model.find(IDname);
	osg::ref_ptr<osg::Node> model;
	if (model_map != this->label_map_model.end())
	{
		model = model_map->second;
		return false;
	}
	else
	{
		std::pair<std::string, osg::ref_ptr<osg::Node>> map_labeltoID;
		map_labeltoID.first = IDname;
		map_labeltoID.second = node;
		label_map_model.insert(map_labeltoID);
		return true;
	}
}
void testosgQt::GetCurrentPointCloud()
{
	timecount++;
	if (timecount > 10)
	{
		timecount = 0;
		timer->stop();
	}
	open3d::geometry::PointCloud *cloud=new open3d::geometry::PointCloud();
	open3d::geometry::PointCloud *cloud2 = new open3d::geometry::PointCloud();
	unsigned start, end;
	std::string str;
	start = clock();
	//
	getPointCloud(cloud, cloud2,1);
	//
	end = clock();
	float time = float(end - start);
	str += std::to_string(time);
	str += "ms";
	//ui.output->setText(QString::fromStdString(str));
	
	if (last_cloud)
	{
		unsigned start, end;
		current_cloud =cloud;
		start = clock();
		mixImg(current_cloud, last_cloud);
		std::string str;
		end = clock();
		float time = float(end - start);
		str += std::to_string(time);
		str += "ms";
		ui.imgtime->setText(QString::fromStdString(str));
		addMechToRoot(current_cloud, 0.03);
		last_cloud = current_cloud;
	}
	else
	{
		last_cloud = cloud;
		//addModelToRoot(cloud);
	    unsigned start, end;
		start = clock();
		std::string str;
		addMechToRoot(last_cloud, 0.03);
		//addModelToRoot(cloud);
		end = clock();
		float time = float(end - start) ;
		str += std::to_string(time);
		str += "ms";
		ui.imgtime->setText(QString::fromStdString(str));
		

	}
	
	
}
void testosgQt::GetPointCloud()
{
	for (auto it : matgroup)
	{
		root->removeChild(it);
	}
	open3d::geometry::PointCloud *cloud1 = new open3d::geometry::PointCloud();
	open3d::geometry::PointCloud *cloud2 = new open3d::geometry::PointCloud();
	auto livox = new livoxSDK();
	livox->getBothPointCloud();
	o3dRegistration* reg = new o3dRegistration();
	std::vector<Eigen::Vector3d> points1;
	std::vector<Eigen::Vector3d> points2;
	/*for (auto it1 : livox->cloud1->points_)
	{
		if (it1.y() > 1.3&&it1.x()<1)
		{
			Eigen::Vector3d p1;
			p1 = it1;
			
			p1(1) = it1(1)-2.2;
		    // points1.push_back(p1);
			points1.push_back(it1);
		}
	}
	for (auto it2 : livox->cloud2->points_)
	{
		if (it2.y() < -1.3&&it2.x()>-1)
		{
			points2.push_back(it2);
		}
	}*/
	//cloud1->points_ = points1;
	//cloud2->points_ = points2;
	//Eigen::Matrix4d R_x=Rot_x(3.1415926/5);
	//Eigen::Matrix4d R_x2 = Rot_x(-3.1415926 / 5);
	//cloud1->Transform(R_x);
	//cloud2->Transform(R_x2);
	//reg->prepare_dataset(*cloud1, *cloud2, 0.05);
	//Eigen::Matrix4d A2B = reg->refine_registration(0.05);
	
	//cloud1->Transform(A2B);
	//p1*r_x*A2B = p2 * r_x2;
	Eigen::Matrix4d n_A2B;// =// R_x * A2B*R_x2.inverse();
	n_A2B << 0.9999, -0.016,0.010128,0.00468,
		- 0.0081, 0.51765, - 0.85582, - 0.11394579,
		- 0.0014,0.85557,0.5171752, - 0.058913,
		0,0,0,1;
	livox->cloud1->Transform(n_A2B);
    addMechToRoot(livox->cloud1, 0.03);
	addMechToRoot(livox->cloud2, 0.03);
	
}
void testosgQt::getOffLineMap()
{
	for (int i = 1; i < 5; i++)
	{
		std::string str = "F:/data/2022623-";
		std::string num = std::to_string(i);
		str +=num ;
		str += "-1.pcd";
		open3d::geometry::PointCloud *cloud = new open3d::geometry::PointCloud();
	    bool is_ok= open3d::io::ReadPointCloudFromPCD(str,*cloud);
	    if(is_ok)
		pathimg1.push_back(cloud);
	}
	for (int i = 1; i < 5; i++)
	{
		std::string str = "F:/data/2022623-";
		std::string num = std::to_string(i);
		str += num;
		str += "-2.pcd";
		open3d::geometry::PointCloud *cloud = new open3d::geometry::PointCloud();
		bool is_ok = open3d::io::ReadPointCloudFromPCD(str, *cloud);
		if (is_ok)
		pathimg2.push_back(cloud);
	}
	
	ui.left->setValue(pathimg1.size());
	ui.right->setValue(pathimg2.size());


}
void testosgQt::GetPointCloud2()
{
	//open3d::geometry::PointCloud *cloud1 = new open3d::geometry::PointCloud();
	auto livox = new livoxSDK();
	livox->getBothPointCloud();
	//addMechToRoot(livox->cloud1, 0.03);
	//addMechToRoot(livox->cloud2, 0.03);
	if (livox->cloud1->points_.size() > 10)
	{
		pathimg1.push_back(livox->cloud1);
	}
	if (livox->cloud2->points_.size() > 10)
	{
		pathimg2.push_back(livox->cloud2);
	}
	SYSTEMTIME now;
	GetLocalTime(&now);
	std::string str1;
	str1 = "F:/data/";
	str1 += WORDToString(now.wYear);
	str1 += WORDToString(now.wMonth);
	str1 += WORDToString(now.wDay);
	str1 += "-";
	str1 += WORDToString(now.wHour);
	str1 += WORDToString(now.wMinute);
	str1 += WORDToString(now.wSecond);
	str1 += "1.pcd";
	std::string str2;
	str2 = "F:/data/";
	str2 += WORDToString(now.wYear);
	str2 += WORDToString(now.wMonth);
	str2 += WORDToString(now.wDay);
	str2 += "-";
	str2 += WORDToString(now.wHour);
	str2 += WORDToString(now.wMinute);
	str2 += WORDToString(now.wSecond);
	str2 += "-2.pcd";
	open3d::io::WritePointCloud(str1, *(livox->cloud1));
	open3d::io::WritePointCloud(str2, *(livox->cloud2));
	ui.left->setValue(pathimg1.size());
	ui.right->setValue(pathimg2.size());
}
void testosgQt::calCalib()
{
	for (auto it : matgroup)
	{
		root->removeChild(it);
	}
	o3dRegistration* reg = new o3dRegistration();
	auto cloud1 = pathimg1[0];
	auto cloud2 = pathimg2[0];
	auto cloud3 = pathimg2[1];
	
	reg->prepare_dataset(*cloud2, *cloud3, 0.05);
	Eigen::Matrix4d A2A = reg->refine_registration(0.05);
	Eigen::Matrix4d A2B;// = //;A*p1*A2B=B*p2
	A2B << 0.9999, -0.016, 0.010128, 0.00468,
		-0.0081, 0.51765, -0.85582, -0.11394579,
		-0.0014, 0.85557, 0.5171752, -0.058913,
		0, 0, 0, 1;

	Eigen::Vector3d axis_y(A2B(0, 3), A2B(1, 3), A2B(2, 3));
	Eigen::Vector3d axis_x(A2A(0, 3), A2A(1, 3), A2A(2, 3));
	Eigen::Vector3d axis_z = axis_x.cross(axis_y);
	axis_z.normalize();
	axis_x.normalize();
	Eigen::Vector3d axis_yn = axis_z.cross(axis_x);
	Eigen::Matrix4d B2C;
	double angle = -28.0 / 180.0 * 3.1415926;
	B2C = Rot_x(angle)*Rot_y(-1.2/180.0*3.1415926);
	//Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	//mat(2,3) = -3;
	//Eigen::Matrix4d mat2 = Rot_x(1 / 180 * 3.1415926);
	std::vector<open3d::geometry::PointCloud*>cloud_group1;
	for (auto it1 : pathimg1)
	{
		std::vector<Eigen::Vector3d> points;
		it1->Transform(A2B);
		points.insert(points.begin(), it1->points_.begin(), it1->points_.end());
		open3d::geometry::PointCloud* pointcloud=new open3d::geometry::PointCloud();
		pointcloud->points_ = points;
		cloud_group1.push_back(pointcloud);
		//it1->Transform(B2C);
		//addMechToRoot(it1, 0.06);
	}
	for (int i=0;i<pathimg2.size();i++)
	{
		cloud_group1[i]->points_.insert(cloud_group1[i]->points_.begin(), pathimg2[i]->points_.begin(), pathimg2[i]->points_.end());
		cloud_group1[i]->Transform(B2C);
		//cloud_group1[i]->Translate(Eigen::Vector3d(0, 0, -3));
		//cloud_group1[i]->Rotate(B2C.topLeftCorner(3,3),Eigen::Vector3d(0,0,0));
		//leftTranfrom(cloud_group1[i],B2C);
		//addMechToRoot(it2, 0.06);
	}
	//addMechToRoot(cloud_group1[0], 0.06);
	//addPointToRoot(cloud_group1[0]);
	//Eigen::Matrix4d n_A2A;

	mapcloud->points_.insert(mapcloud->points_.begin(), cloud_group1[0]->points_.begin(), cloud_group1[0]->points_.end());
	for (int i = 0; i < cloud_group1.size() - 1; i++)
	{
		reg->prepare_dataset(*cloud_group1[i+1], *cloud_group1[i], 0.05);
		Eigen::Matrix4d A2A = reg->refine_registration(0.05);
		cloud_group1[i + 1]->Transform(A2A);
		mapcloud->points_.insert(mapcloud->points_.end(), cloud_group1[i + 1]->points_.begin(), cloud_group1[i + 1]->points_.end());
		if (i < cloud_group1.size() - 2)
		{
			cloud_group1[i + 2]->Transform(A2A);
		}
		//addMechToRoot(cloud_group1[i+1], 0.06);
		//addPointToRoot(cloud_group1[i + 1]);
	}
	//auto downpts=mapcloud->VoxelDownSample(0.01);
	auto removemap= mapcloud->RemoveStatisticalOutliers(10,0.05);
	open3d::geometry::PointCloud* pts = new open3d::geometry::PointCloud();
	auto pts1= std::get<0>(removemap);
	pts->points_ = (*pts1).points_;
	//addMechToRoot(pts, 0.03);
	auto downpts2 = pts->VoxelDownSample(0.02);
	pts->points_ = (*downpts2).points_;
	addPointToRoot(pts);
	//addMechToRoot(mapcloud, 0.06);
	//open3d::io::WritePointCloud("F:/data/map.pcd", *(pts));
}
void testosgQt::makeplane(open3d::geometry::PointCloud* cloud,double x,double y,double z)
{
	std::vector<Eigen::Vector3d> pts;
	for (auto it : cloud->points_)
	{
		if (it.z() <z)
		{
			pts.push_back(it);
		}
	}
	Eigen::Vector3d point;
	for (double i = -x; i < x; i += x / 100)
	{
		for (double j = -y; j < y; j += y/ 100)
		{
			point(0) = i;
			point(1) = j;
			point(2) = z;
			pts.push_back(point);
		}
	}
	//cloud->points_.clear();
	cloud->points_ = pts;
}





osg::ref_ptr<osg::Node> testosgQt::getmodel(std::string IDname)
{
	std::map<std::string, osg::ref_ptr<osg::Node>>::const_iterator model_map;
	model_map = this->label_map_model.find(IDname);
	osg::ref_ptr<osg::Node> model;
	if (model_map != this->label_map_model.end())
	{
		model = model_map->second;
		return model;
	}
	return model;
}
void testosgQt::leftTranfrom(open3d::geometry::PointCloud* cloud, Eigen::Matrix4d mat)
{
	std::vector<Eigen::Vector3d> points;
	Eigen::Matrix4d p_tran=Eigen::Matrix4d::Identity();
	for (auto it : cloud->points_)
	{
		Eigen::Vector3d point;
		p_tran(0, 3) = it(0);
		p_tran(1, 3) = it(1);
		p_tran(2, 3) = it(2);
		point(0) = (mat*p_tran)(0, 3);
		point(1) = (mat*p_tran)(1, 3);
		point(2) = (mat*p_tran)(2, 3);
		points.push_back(point);
	}
	cloud->points_ = points;


}
void testosgQt::addPointToRoot(open3d::geometry::PointCloud* cloud, Eigen::Matrix4d mat)
{
	osg::ref_ptr<osg::Vec3Array> Points = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
	
	auto cloud_down = cloud->VoxelDownSample(0.01);
	auto cloud_out = cloud_down->RemoveStatisticalOutliers(10, 0.5);
	
	auto points= std::get<0>(cloud_out)->points_;
	//auto points = cloud->points_;
	if (!global_cloud)
	{
		global_cloud = new open3d::geometry::PointCloud();
	}
	else
	{
	   
	}
	//global_cloud->points_.insert(global_cloud->points_.begin(), points.begin(), points.end());
	for (auto it:points)
	{
		if (it.z() > -1&&it.y() > -3 && it.y() < 2)
		{
			osg::Vec3d point;
			point.x() = it.x();
			point.y() = it.y();
			point.z() = it.z();
			Points->push_back(point);
			osg::Vec4 color((it.z()-1.8)*0.5, (4-it.z())*0.5, 0, 0.5);
			colors->push_back(color);
			//it.y() = point.y();
			//it.z() = point.z();
			global_cloud->points_.push_back(it);
		}
	}
	Eigen::Matrix4d mat_global;
	Eigen::Matrix4d mat_global2;
	mat_global << 0.999848, -0.000913, -0.017428, -1.677475,
		0.000000, -0.998629, 0.052340, -2.779365,
		-0.017452, -0.052332, -0.998477 ,3.024254,
		0.000000, 0.000000, 0.000000, 1.000000 ;
	mat_global2 << 0.999506, -0.031428, 0.000000, -0.000048,
		0.031423, 0.999354, - 0.017452, 0.062607,
		0.000548, 0.017444, 0.999848, 0.001093,
		0.000000, 0.000000, 0.000000, 1.000000;
//	global_cloud->Transform(mat_global.inverse());
	//leftTranfrom(global_cloud, mat_global2*mat_global);
	open3d::io::WritePointCloudToPCD("F:/data/map.pcd", *global_cloud);
	osg::Matrixd osgmat(mat(0,0),mat(1,0), mat(2, 0), mat(3, 0),
		mat(0, 1), mat(1, 1), mat(2, 1), mat(3, 1), 
		mat(0, 2), mat(1, 2), mat(2, 2), mat(3, 2), 
		mat(0, 3), mat(1, 3), mat(2, 3), mat(3, 3));
	/*
	osg::Matrixd osgmat(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);*/
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	osg::ref_ptr<osg::Geode> geo2= new osg::Geode;
	//auto geo = CreateModule_DelaunayTriangulator(Points, colors, 500);
	geom->setUseDisplayList(true);
	geom->setUseVertexBufferObjects(true);//这两行可以不写
	geom->setVertexArray(Points.get());//设置点集
	geom->setColorArray(colors.get());//设置颜色

	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);//OVERALL表示所有的点都一个颜色，BIND_PER_VERTEX表示每个点各有一种颜色
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, Points->size()));
	geo2->addDrawable(geom.get());
	osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform;
	osg::Matrixd mat_t = osg::Matrix::translate(0, 0, 0);
	
	trans->setMatrix(mat_t);
	//将滑翔机加入到此变换矩阵中
	trans->addChild(geo2.get());
	root->addChild(trans);

	root->addChild(createLight(geo2.get()));

	auto ss = geo2->getOrCreateStateSet();//->setMode(GL_LIGHTING,osg::StateAttribute::ON);

	
	//root->addChild(geo);
	
}
void resize_PointCloud(open3d::geometry::PointCloud* cloud,double voxel_size[2])
{





}
void testosgQt::openMap()
{
	
	auto cloud = new open3d::geometry::PointCloud();
	bool is_ok = open3d::io::ReadPointCloudFromPCD("F:/data/map2.pcd", *cloud);

	osg::ref_ptr<osg::Vec3Array> Points = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
	osg::ref_ptr<osg::Vec3Array> Points2 = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array> colors2 = new osg::Vec4Array();
    

	


	auto cloud_down = cloud->VoxelDownSample(0.02);
	auto cloud_out = cloud_down->RemoveStatisticalOutliers(10, 0.5);

	auto points = std::get<0>(cloud_out)->points_;
	//auto points = cloud->points_;
	if (!global_cloud)
	{
		global_cloud = new open3d::geometry::PointCloud();
	}
	else
	{

	}
	unsigned start, end;
	start = clock();
	//global_cloud->points_.insert(global_cloud->points_.begin(), points.begin(), points.end());
	for (auto it : points)
	{
		if (it.z() > 2 && it.y() > -3 && it.y() < 2&&it.x()<5.3&&it.x()>-1)
		{
			osg::Vec3d point;
			point.x() = it.x();
			point.y() = -it.y();
			point.z() = -it.z();
			Points->push_back(point);
			osg::Vec4 color((it.z() - 1.8)*0.5, (4 - it.z())*0.5, 0, 0.5);
			colors->push_back(color);
			global_cloud->points_.push_back(it);
			if (it.y() > 1)
			{
				Points2->push_back(point);
				osg::Vec4 color((it.z() - 1.8)*0.5, (4 - it.z())*0.5, 0, 0.5);
				colors2->push_back(color);
			}


		}
	}
	Eigen::Matrix4d mat_global;
	Eigen::Matrix4d mat_global2;
	mat_global << 0.999848, -0.000913, -0.017428, -1.677475,
		0.000000, -0.998629, 0.052340, -2.779365,
		-0.017452, -0.052332, -0.998477, 3.024254,
		0.000000, 0.000000, 0.000000, 1.000000;
	mat_global2 << 0.999506, -0.031428, 0.000000, -0.000048,
		0.031423, 0.999354, -0.017452, 0.062607,
		0.000548, 0.017444, 0.999848, 0.001093,
		0.000000, 0.000000, 0.000000, 1.000000;
	//	global_cloud->Transform(mat_global.inverse());
	//leftTranfrom(global_cloud, mat_global2*mat_global);
	open3d::io::WritePointCloudToPCD("F:/data/map3.pcd", *global_cloud);
	/*
	osg::Matrixd osgmat(mat(0, 0), mat(1, 0), mat(2, 0), mat(3, 0),
		mat(0, 1), mat(1, 1), mat(2, 1), mat(3, 1),
		mat(0, 2), mat(1, 2), mat(2, 2), mat(3, 2),
		mat(0, 3), mat(1, 3), mat(2, 3), mat(3, 3));
	
	osg::Matrixd osgmat(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);*/
	
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	osg::ref_ptr<osg::Geometry> geom1 = new osg::Geometry;
	osg::ref_ptr<osg::Geode> geo2 = new osg::Geode;
	//auto geo = CreateModule_DelaunayTriangulator(Points, colors, 500);
	geom->setUseDisplayList(true);
	geom->setUseVertexBufferObjects(true);//这两行可以不写
	geom->setVertexArray(Points.get());//设置点集
	geom->setColorArray(colors.get());//设置颜色

	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);//OVERALL表示所有的点都一个颜色，BIND_PER_VERTEX表示每个点各有一种颜色
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, Points->size()));
	

	geom1->setUseDisplayList(true);
	geom1->setUseVertexBufferObjects(true);//这两行可以不写
	geom1->setVertexArray(Points2.get());//设置点集
	geom1->setColorArray(colors2.get());//设置颜色

	geom1->setColorBinding(osg::Geometry::BIND_PER_VERTEX);//OVERALL表示所有的点都一个颜色，BIND_PER_VERTEX表示每个点各有一种颜色
	geom1->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, Points2->size()));




	geo2->addDrawable(geom.get());


	
	
	geo2->addDrawable(geom1.get());
	
	osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform;
	osg::Matrixd mat_t = osg::Matrix::translate(0, 0, 0);

	trans->setMatrix(mat_t);
	//将滑翔机加入到此变换矩阵中
	trans->addChild(geo2.get());
	
	root->addChild(trans);
	geo2->removeDrawables(0);
	end = clock();
	float time3 = float(end - start);
	//root->addChild(geo);

}
void testosgQt::addMechToRoot(open3d::geometry::PointCloud* cloud, double voxel)
{
	auto cloud_down = cloud->VoxelDownSample(voxel);
	cloud_down->EstimateNormals();
	std::vector<double> radii;
	radii.push_back(voxel);
	radii.push_back(0.01);
	radii.push_back(0.02);
	radii.push_back(0.04);

	
	
	//auto draw = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*cloud_down, radii);
	//auto draw = open3d::geometry::TriangleMesh::CreateFromPointCloudAlphaShape(*cloud_down, 0.01);
	osg::ref_ptr<osg::Vec3Array> Points2 = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array> colors2 = new osg::Vec4Array();
	/*
	for (auto p : draw->triangles_)
	{
		for (int i = 0; i < 3; i++)
		{
			auto p_i = cloud->points_[p[i]];
			if (cloud->points_[p[0]].z() > 2 && cloud->points_[p[1]].z()>2&& cloud->points_[p[2]].z() > 2)
			{
				osg::Vec3d point;
				point.x() = -p_i.x();
				point.y() = -p_i.y();
				point.z() = -p_i.z();
				Points2->push_back(point);
				osg::Vec4 color(1, 1, 1, 0.5);
				colors2->push_back(color);
			}
		}

	}*/
	for (auto p : cloud_down->points_)
	{
			if (p.z() > 1.8&&p.y()>-3&&p.y()<1 )
			{
				osg::Vec3d point;
				point.x() = p.x();
				point.y() = -p.y();
				point.z() = -p.z();
				Points2->push_back(point);
				osg::Vec4 color(1, 1, 1, 0.5);
				colors2->push_back(color);
			}
	}
	osg::ref_ptr<osg::Geode> geo2 = new osg::Geode();
	geo2 = CreateModule_DelaunayTriangulator(Points2, colors2, 1);
	osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform;
	osg::Matrixd mat(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);
	//osg::Matrixd mat_t = osg::Matrix::translate(0, 0, 2000);
	trans->setMatrix(mat);
	//将滑翔机加入到此变换矩阵中
	trans->addChild(geo2.get());
	matgroup.push_back(trans);
	root->addChild(trans);

}
void testosgQt::mixImg(open3d::geometry::PointCloud* current,open3d::geometry::PointCloud* last)
{
	o3dRegistration* reg = new o3dRegistration();
	//
	unsigned start, end;
	start = clock();
	std::string str;
	reg->prepare_dataset(*current,*last, 0.05);
	Eigen::Matrix4d mat_icp = reg->refine_registration(0.05);
	end = clock();
	float time = float(end - start);
	str += std::to_string(time);
	str += "ms";
	ui.output->setText(QString::fromStdString(str));
	current->Transform(mat_icp);
	//addModelToRoot(current);
}
void testosgQt::testTriangulator()
{
	osg::ref_ptr<osg::Vec3Array> Points2 = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array> colors2 = new osg::Vec4Array();
	
	open3d::geometry::PointCloud* cloud = new open3d::geometry::PointCloud();
	
	cloud = readcloudpoint("F://testdata//pointcloud.ply");
	auto cloud_down = cloud->VoxelDownSample(0.005);
	//auto cloud_out=cloud_down->RemoveRadiusOutliers(40, 0.03);
	
	std::vector<double> radii;
	radii.push_back(0.005);
	radii.push_back(0.01);
	radii.push_back(0.02);
	radii.push_back(0.04);
	//[0.005, 0.01, 0.02, 0.04];
    //auto cloud_down = cloud->VoxelDownSample(0.01);
	cloud_down->EstimateNormals();
	auto draw = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*cloud_down, radii);
	//std::vector<std::shared_ptr<const open3d::geometry::Geometry>> group;
	//group.push_back(draw);
	//open3d::visualization::DrawGeometries(group);
	for (auto p : draw->triangles_)
	{
		for (int i = 0; i < 3; i++)
		{   
			auto p_i=cloud_down->points_[p[i]];

			osg::Vec3d point;
			if (abs(cloud_down->points_[p[0]].z() < 0.7)&&
				abs(cloud_down->points_[p[1]].z() < 0.7)&&
				abs(cloud_down->points_[p[2]].z() < 0.7))
			{
				point.x() = -p_i.x() * 1000;
				point.y() = -p_i.y() * 1000;
				point.z() = -p_i.z() * 1000;
				Points2->push_back(point);
				osg::Vec4 color(1, 1, 1, 0.5);
				colors2->push_back(color);
			}
		}
	 
    }
	Eigen::MatrixXd mat1(Points2->size(),3);
	Eigen::MatrixXd mat2(Points2->size(),3);
	for (int i = 0; i < Points2->size(); i++)
	{
		osg::Vec3d point=Points2[0][i];
		mat1(i, 0) = point.x();
		mat1(i, 1) = point.y();
		mat1(i, 2) = point.z();
		mat2(i, 0) = point.x() + 100;
		mat2(i, 1) = point.y() + 20;
		mat2(i, 2) = point.z() + 30;
	}
	/*ICP_OUT result = icp(mat2, mat1, 100, 0.0000001);

	double  x = result.trans(0, 3);
	double  y = result.trans(1, 3);
	double  z = result.trans(2, 3);
	std::string str=std::to_string(x);
	str += ",";
	str+= std::to_string(y);
	str += ",";
	str += std::to_string(z);
	ui.output->setText(QString::fromStdString(str));
	*/
	osg::ref_ptr<osg::Geode> geo2 = new osg::Geode();
	geo2 = CreateModule_DelaunayTriangulator(Points2, colors2,10);
	osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform;
	osg::Matrixd mat(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);
	//osg::Matrixd mat_t = osg::Matrix::translate(0, 0, 2000);
	trans->setMatrix(mat);
	//将滑翔机加入到此变换矩阵中
	trans->addChild(geo2.get());
    root->addChild(trans);
	//root->addChild(geo2);
}
osg::ref_ptr<osg::Geode> CreateModule_DelaunayTriangulator(osg::ref_ptr<osg::Vec3Array> Points, osg::ref_ptr<osg::Vec4Array>color,double limit_distance)
{
	//DelaunayTriangulator
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;

	osg::ref_ptr<osgUtil::DelaunayTriangulator> trig = new osgUtil::DelaunayTriangulator();
	trig->setInputPointArray(Points);
	
	/** NB you need to supply a vec3 array for the triangulator to calculate normals into */
	osg::ref_ptr<osg::Vec3Array> norms = new osg::Vec3Array;
	norms->push_back(osg::Vec3(0.0, 0.0, -1.0));
	trig->setOutputNormalArray(norms);
	trig->triangulate();//it will change the ordinary and maybe change the Size of point

	//Add color and Calculate the texture coordinates after triangulation as 
	//the points may get disordered by the triangulate function
	osg::ref_ptr<osg::Geometry> gm = new osg::Geometry;
	gm->setVertexArray(Points); // points may have been modified in order by triangulation.
	//add color
	gm->setColorArray(color);//set color
	gm->setColorBinding(osg::Geometry::BIND_OVERALL);
	auto de = trig->getTriangles();

	osg::DrawElementsUInt::iterator iter;
	unsigned int _size = de->size();
	int a(0), x(0);
	float th = limit_distance;
	
	for (iter = de->begin(); iter != de->end();)
	{
		osg::Vec3f pt1 = Points->at(*iter);
		osg::Vec3f pt2 = Points->at(*(iter + 1));
		osg::Vec3f pt3 = Points->at(*(iter + 2));

		double dis1 = (pt1 - pt2).length();//getDistance()  函数 是我自己限制函数
		double dis2 = (pt3 - pt2).length();
		double dis3 = (pt1 - pt3).length();

		//如果大于一定数值删除 
		if (dis3 > th || dis2 > th || dis1 > th)
		{
			iter = de->erase(iter, iter + 3); //
			continue;
		}
		iter += 3;
	}
	

	gm->addPrimitiveSet(de);
	gm->setNormalArray(trig->getOutputNormalArray());
	gm->setNormalBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
	
	// use smoothing visitor to set the average normals
	osgUtil::SmoothingVisitor sv;
	sv.apply(*gm);
	geode->addDrawable(gm.get());

	return geode;
}
open3d::geometry::PointCloud* testosgQt::readcloudpoint(std::string filename)
{
	const std::string option("pointcloud");
	auto cloud =  new open3d::geometry::PointCloud();
	auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
	
	if (option == "mesh") {
		if (!open3d::io::ReadTriangleMesh(filename, *mesh)) {
			open3d::utility::LogWarning("Failed to read {}", filename);
			
		}
		cloud->points_ = mesh->vertices_;
	}
	else if (option == "pointcloud") {
		if (!open3d::io::ReadPointCloud(filename, *cloud)) {
			open3d::utility::LogWarning("Failed to read {}\n\n", filename);
			
		}
	}
	else {
		open3d::utility::LogError("option {} not supported\n", option);
	}
	return cloud;
}
void testosgQt::getMapPath(int num_x,int num_y,float init_x,float init_y,float step_x,float step_y)
{
	path1.clear();
	path2.clear();
	for (int i=0; i < num_y; i++)
	{
		if (i % 2)
		{
			for (int j = num_x-1; j > -1; j--)
			{
				Eigen::Vector2d point;
				point(0) = init_x + j * step_x;
				point(1) = init_y + i * step_y;
				path2.push_back(point);
			}
		}
		else
		{
			for (int j = 0; j < num_x; j++)
			{
				Eigen::Vector2d point;
				point(0) = init_x + j * step_x;
				point(1) = init_y + i * step_y;

				path2.push_back(point);
			}
		}
	}
	if (num_x < 2)
	{
			Eigen::Vector2d point;
			point(0) = init_x- step_x;
			point(1) = init_y;
			path1.push_back(point);
			Eigen::Vector2d point1;
			point1(0) = init_x;
			point1(1) = init_y;
			path1.push_back(point1);
			Eigen::Vector2d point2;
			point2(0) = init_x + step_x;
			point2(1) = init_y;
			path1.push_back(point2);
	}
	else if (num_x > 1)
	{
		float step = (num_x - 1)*step_x / 3;
		Eigen::Vector2d point;
		point(0) = init_x - step;
		point(1) = init_y;
		path1.push_back(point);
		Eigen::Vector2d point1;
		point1(0) = init_x;
		point1(1) = init_y;
		path1.push_back(point1);
		Eigen::Vector2d point2;
		point2(0) = init_x + step;
		point2(1) = init_y;
		path1.push_back(point2);
	}

}
void testosgQt::getCatchingPoints(BoxGround *box, open3d::geometry::PointCloud* cloud,Eigen::Vector3d grid,bool grabORthorw)
{
	std::vector<Eigen::Vector3d> cut_point;
	for (int i = 0; i < cloud->points_.size(); i++)
	{
		Eigen::Vector3d point = cloud->points_[i];
		if (point.x() < box->x + box->length_x / 2 &&
			point.x() > box->x - box->length_x / 2 &&
			point.y() < box->y + box->length_y / 2 &&
			point.y() > box->y - box->length_y / 2 &&
			point.z() < box->z + box->length_z / 2 &&
			point.z() > box->z - box->length_z / 2
			)
		{
			cut_point.push_back(point);
		}
	}
	int num_x = (box->length_x/2) / grid.x()+ (box->length_x / 2) / grid.x();
	int num_y = (box->length_y/2) / grid.y()+ (box->length_y / 2) / grid.y();
	float *local_grid = new float[num_x*num_y];
	float *local_count = new float[num_x*num_y];
	for (int i = 0; i < num_x*num_y; i++)
	{
		local_grid[i] = 0;
			local_count[i] = 0;
	}
	for (int j = 0; j < cut_point.size(); j++)
	{
		Eigen::Vector3d point = cut_point[j];
		int scale_x = (point.x() - box->x + num_x * grid.x()/2) / grid.x();
		int scale_y = (point.y() - box->y + num_y * grid.y()/2) / grid.y();
		float z = point.z();
		float old_z = local_grid[scale_x + scale_y * num_x];
		int count = local_count[scale_x + scale_y * num_x];
		local_grid[scale_x + scale_y * num_x] = ((old_z*count)+z)/(count+1);
		local_count[scale_x + scale_y * num_x] += 1;
	}
	// 初始化索引向量
	std::vector<int> idx(num_x*num_y);
	//使用iota对向量赋0~？的连续值
	std::iota(idx.begin(), idx.end(), 0);
	sort(idx.begin(), idx.end(),
		[&local_grid](size_t i1, size_t i2) {return local_grid[i1] < local_grid[i2];});
	std::sort(local_grid, local_grid + num_x * num_y);
	if (grabORthorw)
	{
		for (int i = 0; i < box->numofcatch; i++)
		{
			box->points[i][0]= (idx[num_x*num_y - 1-i] % num_x - num_x / 2)*grid.x()+box->x ;
			box->points[i][1]= (idx[num_x*num_y - 1-i] / num_x - num_y / 2)*grid.y()+box->y ;
			box->points[i][2]= local_grid[num_x*num_y - 1-i];
		}
	}
	else //(point.x - box->x + num_x * grid.x) / grid.x
	{
		box->points[0][0] = (idx[0] % num_x - num_x / 2)*grid.x()+box->x ;
		box->points[0][1] = (idx[0] / num_x - num_y / 2)*grid.y()+box->y ;
		box->points[0][2] = local_grid[0];
	}
}