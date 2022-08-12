#pragma once

#include "open3d/Open3D.h"
#include <QtWidgets/QMainWindow>
#include <Qfile>
#include <QTextStream>
#include "ui_testosgQt.h"
#include <QWidget>
#include <QPushButton>
#include <osgQOpenGL/osgQOpenGLWidget>
#include <QBoxLayout>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgUtil/Optimizer>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osg/Drawable>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgUtil/DelaunayTriangulator>
#include <osgUtil/SmoothingVisitor>
#include <osg/Array>
#include <Eigen/Core>
#include "ModelShape.h"
#include "PickHandler.h"
#include "icp.h"
#include "o3dRegistration.h"
#include "livoxSDK.h"
#include <osg/Material>
enum WorkType { Auto, SemiAuto,Manual, BuildMap };
enum GroundType {GarbageSlot, FermentationArea,BurnerPort};

struct PointCloudGround
{
	std::string label;
	open3d::geometry::PointCloud * pointcloud;
	float x=0;
	float y=0;
	int index = 0;
};
struct BoxGround
{
	std::string name;
	float x=0;
	float y=0;
	float z=0;
	float length_x=1;
	float length_y=1;
	float length_z=1;
	int level = -1;
	int numofcatch = 1;
	float points[10][3] = {0};
	GroundType Type;
};
struct SchedulingTask
{
	std::vector<BoxGround*> Grab_Ground;//抓取点
	std::vector<BoxGround*> Place_Ground;//放置点
	float Frequency = 0;
	float current_Value = 0;
	float overflow_Value = 0;
	bool is_working = false;
};
class testosgQt : public QMainWindow
{
    Q_OBJECT

public:
    testosgQt(QWidget *parent = Q_NULLPTR);
	osgQOpenGLWidget *pOsgW;
	osg::Geode* makeCoordinate(float a_x, float a_y, float a_z, float font_size);
private:
    Ui::testosgQtClass ui;
	osg::ref_ptr<osgGA::CameraManipulator> contrel;
	osgViewer::Viewer *Viewer;
	osg::ref_ptr<osg::Group> root;
	void addGroundBox(float x, float y, float z, std::string name, osg::Vec4 color1, osg::Vec4 color2);
	bool addmodel(std::string IDname, osg::ref_ptr<osg::Node> node);
	osg::ref_ptr<osg::Node> getmodel(std::string IDname);
	void leftTranfrom(open3d::geometry::PointCloud * cloud, Eigen::Matrix4d mat);
	void addPointToRoot(open3d::geometry::PointCloud * cloud, Eigen::Matrix4d mat = Eigen::Matrix4d::Identity());
	
	//void addModelToRoot(open3d::geometry::PointCloud * cloud, Eigen::Matrix4d mat=Eigen::Matrix4d::Identity());
	void addMechToRoot(open3d::geometry::PointCloud * cloud, double voxel);
	void mixImg(open3d::geometry::PointCloud * cloud1, open3d::geometry::PointCloud * cloud2);
	void testTriangulator();
	open3d::geometry::PointCloud * readcloudpoint(std::string filename);
	void adjustGripper(osg::Geode * geode, float angle);
	void getMapPath(int num_x, int num_y, float init_x, float init_y, float step_x, float step_y);
	void getCatchingPoints(BoxGround * box, open3d::geometry::PointCloud * cloud, Eigen::Vector3d grid, bool grabORthorw=true);
	void initCrane1(float size);
	void initCrane2(float size);
	void updateCrane1(float x, float y, float z);
	void updateCrane2(float x, float y, float z);
	open3d::geometry::PointCloud* mapcloud;
	open3d::geometry::PointCloud* current_cloud=0;
	open3d::geometry::PointCloud* last_cloud=0;
	open3d::geometry::PointCloud* global_cloud=0;
	QTimer *timer;
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	osg::ref_ptr<osg::Geode> geo2 = new osg::Geode;
	int timecount = 0;
	std::vector<open3d::geometry::PointCloud*> pathimg1;
	std::vector<open3d::geometry::PointCloud*> pathimg2;
	std::vector<osg::ref_ptr<osg::MatrixTransform>> matgroup;
private slots:
	void openMap();
	void initOsg();
	void changeOpen();
	void changeClose();
	void CreatePath();
	void startSim();
	void startMotion();
	void SimulationMotion();
	void SimMotion(double x, double y, double z);
	void OpenGripper();
	void CloseGripper();
	void ExcuteGripperOpen();
	void ExcuteGripperClose();
	void GetCurrentPointCloud();
	void GetPointCloud();
	void getOffLineMap();
	void GetPointCloud2();
	void calCalib();
	void makeplane(open3d::geometry::PointCloud * cloud, double x, double y, double z);
	//void makeplane(open3d::geometry::PointCloud * cloud, double z=0);
	void saveGroundBox();
	void CalTaskGround1();
	void CalTaskPath();
	void CalTaskGround2();
	void ExcuteTask1();
	void SimSetGround();
	void CalCatchPoints(BoxGround * box);
	void updateBoxGround(BoxGround * box);
	void updateAllBoxGround();
	void addTask();
	void addGroundBox(float x, float y, float z, std::string name, osg::Vec4 color1, osg::Vec4 color2, osg::Vec3 pose);
private://安装行车和夹爪
	osg::ref_ptr<osg::Geode> Gripper1;
	osg::ref_ptr<osg::Geode> Gripper2;
	bool Gripper1_state = 0;
	bool Gripper2_state = 0;
	osg::ref_ptr<osg::Geode> rope1;
	osg::ref_ptr<osg::Geode> rope2;
	osg::ref_ptr<osg::Geode> crane1;
	osg::ref_ptr<osg::Geode> crane2;
	osg::ref_ptr<osg::MatrixTransform> tr_x1 ;
	osg::ref_ptr<osg::MatrixTransform> tr_y1 ;
	osg::ref_ptr<osg::MatrixTransform> tr_z1 ;
	osg::ref_ptr<osg::MatrixTransform> tr_x2;
	osg::ref_ptr<osg::MatrixTransform> tr_y2;
	osg::ref_ptr<osg::MatrixTransform> tr_z2;
	Eigen::Vector3d current_cranepose1;
	Eigen::Vector3d current_cranepose2;
	WorkType current_WorkType;
	std::vector<Eigen::Vector2d> path1;
	std::vector<Eigen::Vector2d> path2;
	float crane_size1 = 1;
	float crane_size2 = 1;
private://分区
	std::vector<PointCloudGround> Ground_group;
	std::vector<BoxGround*> Customer_Ground;
	std::map<std::string, osg::ref_ptr<osg::Node>> label_map_model;
private://通用
	ModelShape* current_ground;
    QTimer *Open_timer;
	QTimer *Close_timer;
	QTimer *Motion_timer;
	QTimer *Sim_timer;
private://任务调度
	std::vector<SchedulingTask> Task_group1;
	std::vector<SchedulingTask> Task_group2;
	BoxGround *current_Grab_Ground1;
	BoxGround *current_Place_Ground1;
	BoxGround *current_Grab_Ground2;
	BoxGround *current_Place_Ground2;
	bool crane1_state = false;
	bool crane2_state = false;
	bool crane1_process = false;
	bool crane2_process = false;
	float safe_height = -10000;
	std::vector <Eigen::Vector3d> movePath1;
	int current_path_point=0;
private:
	int station_Max_length_x=4800;
	int station_Max_length_y=2400;
};




void createRope(float z, float radius, float step, osg::Geode * geode);
osg::Geode * createGripper(float size=1);
osg::Geode * createBox(float x=10,float y=10,float z=10);
osg::Geode * createSphere(float r=10);
osg::Matrixd eigen2osg(Eigen::Matrix4d mat);
osg::ref_ptr<osg::Geode> CreateModule_DelaunayTriangulator(osg::ref_ptr<osg::Vec3Array> Points, osg::ref_ptr<osg::Vec4Array> color, double limit_distance = 0.5);
osg::ref_ptr<osgText::Text> createText(float size, osg::Vec3 pos, std::string name, osg::Vec4 color);