#include "livoxSDK.h"

#include <algorithm>
#include <string.h>
#include "lvx_file.h"
#include "cmdline.h"


DeviceItem devices[kMaxLidarCount];
LvxFileHandle lvx_file_handler;
std::list<LvxBasePackDetail> point_packet_list;
std::vector<std::string> broadcast_code_rev;
std::condition_variable lidar_arrive_condition;
std::condition_variable extrinsic_condition;
std::condition_variable point_pack_condition;
std::mutex mtx;
int lvx_file_save_time = 1;
bool is_finish_extrinsic_parameter = false;
bool is_read_extrinsic_from_xml = false;
uint8_t connected_lidar_count = 0;

#define FRAME_RATE 20

using namespace std::chrono;
/** Connect all the broadcast device in default and connect specific device when use program options or broadcast_code_list is not empty. */
std::vector<std::string> broadcast_code_list = {
	//"000000000000001"
	//"000000000000002",
	//"000000000000003",
	//"000000000000004"
};

/** Receiving error message from Livox Lidar. */
void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message) {
	static uint32_t error_message_count = 0;
	if (message != NULL) {
		++error_message_count;
		if (0 == (error_message_count % 100)) {
			printf("handle: %u\n", handle);
			printf("temp_status : %u\n", message->lidar_error_code.temp_status);
			printf("volt_status : %u\n", message->lidar_error_code.volt_status);
			printf("motor_status : %u\n", message->lidar_error_code.motor_status);
			printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
			printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
			printf("pps_status : %u\n", message->lidar_error_code.device_status);
			printf("fan_status : %u\n", message->lidar_error_code.fan_status);
			printf("self_heating : %u\n", message->lidar_error_code.self_heating);
			printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
			printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
			printf("system_status : %u\n", message->lidar_error_code.system_status);
		}
	}
}

/** Receiving point cloud data from Livox LiDAR. */
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
	if (data) {
		if (handle < connected_lidar_count && is_finish_extrinsic_parameter) {
			std::unique_lock<std::mutex> lock(mtx);
			LvxBasePackDetail packet;
			packet.device_index = handle;
			lvx_file_handler.BasePointsHandle(data, packet);
			point_packet_list.push_back(packet);
		}
	}
}

/** Callback function of starting sampling. */
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
	printf("OnSampleCallback statues %d handle %d response %d \n", status, handle, response);
	if (status == kStatusSuccess) {
		if (response != 0) {
			devices[handle].device_state = kDeviceStateConnect;
		}
	}
	else if (status == kStatusTimeout) {
		devices[handle].device_state = kDeviceStateConnect;
	}
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
}

/** Callback function of get LiDARs' extrinsic parameter. */
void OnGetLidarExtrinsicParameter(livox_status status, uint8_t handle, LidarGetExtrinsicParameterResponse *response, void *data) {
	if (status == kStatusSuccess) {
		if (response != 0) {
			printf("OnGetLidarExtrinsicParameter statue %d handle %d response %d \n", status, handle, response->ret_code);
			std::unique_lock<std::mutex> lock(mtx);
			LvxDeviceInfo lidar_info;
			strncpy((char *)lidar_info.lidar_broadcast_code, devices[handle].info.broadcast_code, kBroadcastCodeSize);
			memset(lidar_info.hub_broadcast_code, 0, kBroadcastCodeSize);
			lidar_info.device_index = handle;
			lidar_info.device_type = devices[handle].info.type;
			lidar_info.extrinsic_enable = true;
			lidar_info.pitch = response->pitch;
			lidar_info.roll = response->roll;
			lidar_info.yaw = response->yaw;
			lidar_info.x = static_cast<float>(response->x / 1000.0);
			lidar_info.y = static_cast<float>(response->y / 1000.0);
			lidar_info.z = static_cast<float>(response->z / 1000.0);
			lvx_file_handler.AddDeviceInfo(lidar_info);
			if (lvx_file_handler.GetDeviceInfoListSize() == connected_lidar_count) {
				is_finish_extrinsic_parameter = true;
				extrinsic_condition.notify_one();
			}
		}
	}
	else if (status == kStatusTimeout) {
		printf("GetLidarExtrinsicParameter timeout! \n");
	}
}

/** Get LiDARs' extrinsic parameter from file named "extrinsic.xml". */
void LidarGetExtrinsicFromXml(uint8_t handle) {
	LvxDeviceInfo lidar_info;
	ParseExtrinsicXml(devices[handle], lidar_info);
	lvx_file_handler.AddDeviceInfo(lidar_info);
	lidar_info.extrinsic_enable = true;
	if (lvx_file_handler.GetDeviceInfoListSize() == broadcast_code_list.size()) {
		is_finish_extrinsic_parameter = true;
		extrinsic_condition.notify_one();
	}
}

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *data) {
	if (status != kStatusSuccess) {
		printf("Device Query Informations Failed %d\n", status);
	}
	if (ack) {
		printf("firm ver: %d.%d.%d.%d\n",
			ack->firmware_version[0],
			ack->firmware_version[1],
			ack->firmware_version[2],
			ack->firmware_version[3]);
	}
}

void LidarConnect(const DeviceInfo *info) {
	uint8_t handle = info->handle;
	QueryDeviceInformation(handle, OnDeviceInformation, NULL);
	if (devices[handle].device_state == kDeviceStateDisconnect) {
		devices[handle].device_state = kDeviceStateConnect;
		devices[handle].info = *info;
	}
}

void LidarDisConnect(const DeviceInfo *info) {
	uint8_t handle = info->handle;
	devices[handle].device_state = kDeviceStateDisconnect;
}

void LidarStateChange(const DeviceInfo *info) {
	uint8_t handle = info->handle;
	devices[handle].info = *info;
}

/** Callback function of changing of device state. */
void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type) {
	if (info == nullptr) {
		return;
	}
	printf("OnDeviceChange broadcast code %s update type %d\n", info->broadcast_code, type);
	uint8_t handle = info->handle;
	if (handle >= kMaxLidarCount) {
		return;
	}

	if (type == kEventConnect) {
		LidarConnect(info);
		printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
	}
	else if (type == kEventDisconnect) {
		LidarDisConnect(info);
		printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
	}
	else if (type == kEventStateChange) {
		LidarStateChange(info);
		printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
	}

	if (devices[handle].device_state == kDeviceStateConnect) {
		printf("Device Working State %d\n", devices[handle].info.state);
		if (devices[handle].info.state == kLidarStateInit) {
			printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
		}
		else {
			printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
		}
		printf("Device feature %d\n", devices[handle].info.feature);
		SetErrorMessageCallback(handle, OnLidarErrorStatusCallback);
		if (devices[handle].info.state == kLidarStateNormal) {
			if (!is_read_extrinsic_from_xml) {
				LidarGetExtrinsicParameter(handle, OnGetLidarExtrinsicParameter, nullptr);
			}
			else {
				LidarGetExtrinsicFromXml(handle);
			}
			LidarStartSampling(handle, OnSampleCallback, nullptr);
			devices[handle].device_state = kDeviceStateSampling;
		}
	}
}

/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
	if (info == nullptr || info->dev_type == kDeviceTypeHub) {
		return;
	}

	printf("Receive Broadcast Code %s\n", info->broadcast_code);
	if ((broadcast_code_rev.size() == 0) ||
		(std::find(broadcast_code_rev.begin(), broadcast_code_rev.end(), info->broadcast_code) == broadcast_code_rev.end())) {
		broadcast_code_rev.push_back(info->broadcast_code);
		lidar_arrive_condition.notify_one();
	}
}

/** Wait until no new device arriving in 2 second. */
void WaitForDevicesReady() {
	bool device_ready = false;
	seconds wait_time = seconds(2);
	steady_clock::time_point last_time = steady_clock::now();
	while (!device_ready) {
		std::unique_lock<std::mutex> lock(mtx);
		lidar_arrive_condition.wait_for(lock, wait_time);
		if ((steady_clock::now() - last_time + milliseconds(50)) >= wait_time) {
			device_ready = true;
		}
		else {
			last_time = steady_clock::now();
		}
	}
}

void WaitForExtrinsicParameter() {
	std::unique_lock<std::mutex> lock(mtx);
	extrinsic_condition.wait(lock);
}

void AddDevicesToConnect() {
	if (broadcast_code_rev.size() == 0)
		return;

	for (int i = 0; i < broadcast_code_rev.size(); ++i) {
		if ((broadcast_code_list.size() != 0) &&
			(std::find(broadcast_code_list.begin(), broadcast_code_list.end(), broadcast_code_rev[i]) == broadcast_code_list.end())) {
			continue;
		}
		uint8_t handle = 0;
		broadcast_code_rev[0] = "3GGDJ7S00100211";
		broadcast_code_rev[1] = "3GGDJ7U00100431";
		livox_status result = AddLidarToConnect(broadcast_code_rev[i].c_str(), &handle);
		if (result == kStatusSuccess) {
			/** Set the point cloud data for a specific Livox LiDAR. */
			SetDataCallback(handle, GetLidarData, nullptr);
			devices[handle].handle = handle;
			devices[handle].device_state = kDeviceStateDisconnect;
			connected_lidar_count++;
		}
	}
}

/** Set the program options.
* You can input the registered device broadcast code and decide whether to save the log file.
*/
void SetProgramOption(int argc, const char *argv[]) {
	cmdline::parser cmd;
	cmd.add<std::string>("code", 'c', "Register device broadcast code", false);
	cmd.add("log", 'l', "Save the log file");
	cmd.add<int>("time", 't', "Time to save point cloud to the lvx file", false);
	cmd.add("param", 'p', "Get the extrinsic parameter from extrinsic.xml file");
	cmd.add("help", 'h', "Show help");
	cmd.parse_check(argc, const_cast<char **>(argv));
	if (cmd.exist("code")) {
		std::string sn_list = cmd.get<std::string>("code");
		printf("Register broadcast code: %s\n", sn_list.c_str());
		size_t pos = 0;
		broadcast_code_list.clear();
		while ((pos = sn_list.find("&")) != std::string::npos) {
			broadcast_code_list.push_back(sn_list.substr(0, pos));
			sn_list.erase(0, pos + 1);
		}
		broadcast_code_list.push_back(sn_list);
	}
	if (cmd.exist("log")) {
		printf("Save the log file.\n");
		SaveLoggerFile();
	}
	if (cmd.exist("time")) {
		printf("Time to save point cloud to the lvx file:%d.\n", cmd.get<int>("time"));
		lvx_file_save_time = cmd.get<int>("time");
	}
	if (cmd.exist("param")) {
		printf("Get the extrinsic parameter from extrinsic.xml file.\n");
		is_read_extrinsic_from_xml = true;
	}
	return;
}
int DecToBin(uint8_t dec1)
{
	int result = 0, temp = dec1, j = 1;
	while (temp)
	{
		result = result + j * (temp % 2);
		temp = temp / 2;
		j = j * 10;
	}
	return result;
}

int Dec4ToBin(uint8_t Dec1, uint8_t Dec2, uint8_t Dec3, uint8_t Dec4)
{
	int result1 = DecToBin(Dec1);
	int result2 = DecToBin(Dec2);
	int result3 = DecToBin(Dec3);
	int result4 = DecToBin(Dec4);

	// std::cout<<result1<<","<<result2<<","<<result3<<","<<result4<<std::endl;

	char *resultChar = (char *)malloc(32);
	sprintf(resultChar, "%.8d%.8d%.8d%.8d", result4, result3, result2, result1);//²¹Î»µ½8
  // std::cout<<resultChar<<std::endl;
	int i = 0, n = 0, m = 1, result = 0;
	while (resultChar[i++])
		n++;
	for (i = 0; i < n; i++)
	{
		result = result + (resultChar[n - i - 1] - '0')*m;
		m *= 2;
	}
	free(resultChar);

	return  result;
}

int getPointCloud(open3d::geometry::PointCloud *cloud1, open3d::geometry::PointCloud *cloud2,float time) {
	/** Set the program options. */
	//SetProgramOption(argc, argv);

	//printf("Livox SDK initializing.\n");
	/** Initialize Livox-SDK. */
	
		if (!Init()) {
			return -1;
		}
		//printf("Livox SDK has been initialized.\n");

		LivoxSdkVersion _sdkversion;
		GetLivoxSdkVersion(&_sdkversion);
		//printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

		memset(devices, 0, sizeof(devices));

		/** Set the callback function receiving broadcast message from Livox LiDAR. */
		SetBroadcastCallback(OnDeviceBroadcast);

		/** Set the callback function called when device state change,
		 * which means connection/disconnection and changing of LiDAR state.
		 */
		SetDeviceStateUpdateCallback(OnDeviceInfoChange);

		/** Start the device discovering routine. */
		if (!Start()) {
			Uninit();
			return -1;
		}
		//printf("Start discovering device.\n");

		WaitForDevicesReady();
	
	AddDevicesToConnect();

	if (connected_lidar_count == 0) {
		//printf("No device will be connected.\n");
		Uninit();
		return -1;
	}

	WaitForExtrinsicParameter();

	//printf("Start initialize lvx file.\n");
	//if (!lvx_file_handler.InitLvxFile()) {
		//Uninit();
		//return -1;
	//}

	//lvx_file_handler.InitLvxFileHeader();

	int i = 0;
	steady_clock::time_point last_time = steady_clock::now();
	std::vector<Eigen::Vector3d> points1;
	std::vector<Eigen::Vector3d> points2;
	std::vector<std::list<LvxBasePackDetail>> point_group;
	
	for (i = 0; i < /*lvx_file_save_time*/ time* FRAME_RATE; ++i) {
		std::list<LvxBasePackDetail> point_packet_list_temp;
		{
			std::unique_lock<std::mutex> lock(mtx);
			point_pack_condition.wait_for(lock, milliseconds(kDefaultFrameDurationTime) - (steady_clock::now() - last_time));
			last_time = steady_clock::now();
			point_packet_list_temp.swap(point_packet_list);
		}
		if (point_packet_list_temp.empty()) {
			//printf("Point cloud packet is empty.\n");
			break;
		}
		else
		{
			point_group.push_back(point_packet_list_temp);
		}
		
		//printf("Finish save %d frame to lvx file.\n", i);
		//lvx_file_handler.SaveFrameToLvxFile(point_packet_list_temp);
	}
	//lvx_file_handler.CloseLvxFile();
	for (i = 0; i < kMaxLidarCount; ++i) {
		if (devices[i].device_state == kDeviceStateSampling) {
			/** Stop the sampling of Livox LiDAR. */
			LidarStopSampling(devices[i].handle, OnStopSampleCallback, nullptr);
		}
	}
	
	for (auto it : point_group)
	{
		for (auto iter = it.begin(); iter != it.end(); iter++)
		{
			if (iter->data_type != 2) continue;
			// std::cout<<"size:"<<(int)iter->pack_size<<std::endl;
			// std::cout<<"device_index:"<<(int)iter->device_index<<std::endl;
			// std::cout<<"version:"<<(int)iter->version<<std::endl;
			// std::cout<<"port_id:"<<(int)iter->port_id<<std::endl;
			// std::cout<<"lidar_index:"<<(int)iter->lidar_index<<std::endl;
			// std::cout<<"rsvd:"<<(int)iter->rsvd<<std::endl;
			// std::cout<<"error_code:"<<(int)iter->error_code<<std::endl;
			// std::cout<<"timestamp:"<<(int)iter->timestamp_type<<std::endl;
			// std::cout<<"data_type:"<<(int)iter->data_type<<std::endl;
			int length = 14;
			int **rawPoint = new int *[96];
			for (int raw = 0; raw < 96; raw++)
			{
				rawPoint[raw] = new int[length];
			}
			//memset((char*)&rawPoint[0][0],0,96*length*sizeof(int));
			for (int flag = 0; flag < length * 96; flag++)
			{
				rawPoint[flag / length][flag%length] = iter->raw_point[flag];
			}
			for (int j = 0; j < 96; j++)
			{

				Eigen::Vector3d point;
				point[2] = (float)(Dec4ToBin(rawPoint[j][0], rawPoint[j][1], rawPoint[j][2], rawPoint[j][3])) / 1000.0;
				point[1] = (float)(Dec4ToBin(rawPoint[j][4], rawPoint[j][5], rawPoint[j][6], rawPoint[j][7])) / 1000.0;
				point[0] = -(float)(Dec4ToBin(rawPoint[j][8], rawPoint[j][9], rawPoint[j][10], rawPoint[j][11])) / 1000.0;
				//point.rgba = rawPoint[j][12];
				//std::cout<<p.x<<", "<<p.y<<", "<<p.z<<std::endl;
				//devices[0].info
				if (iter->device_index ==0 )
				{
					points1.push_back(point);
				}
				else
				{
					points2.push_back(point);
				 }
				//std::cout<<cloud->size()<<std::endl;

			}
			for (int raw2 = 0; raw2 < 96; raw2++)
			{
				delete[]rawPoint[raw2];
			}
			delete[] rawPoint;

		}
	}
	
	/** Uninitialize Livox-SDK. */
	cloud1->points_ = points1;
	cloud2->points_ = points2;
	Uninit();
}


livoxSDK::livoxSDK()
{
	cloud1 = new open3d::geometry::PointCloud();
	cloud2 = new open3d::geometry::PointCloud();
}

livoxSDK::~livoxSDK()
{
}
void livoxSDK::getLeftPointCloud()
{
	//getPointCloud(cloud1, 1, 0);
}
void livoxSDK::getRightPointCloud()
{
	//points1.push_back(point);
}

void livoxSDK::getBothPointCloud()
{
	//task1 = std::thread(std::bind(&livoxSDK::getLeftPointCloud, this));
	//task2 = std::thread(std::bind(&livoxSDK::getRightPointCloud, this));
	//task1.join();
	//task2.join();
	//getLeftPointCloud();
   //getRightPointCloud();
	getPointCloud(cloud1, cloud2, 1);
};