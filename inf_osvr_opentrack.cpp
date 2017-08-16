#pragma comment (lib,"WSock32.Lib")

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/ButtonInterfaceC.h>

// Standard includes
#include <thread>
#include <iostream>
#include <atomic>
#include <mutex>
#include <winsock2.h>

// Third party includes
#include <json/json.h>


// Json header
#include "inf_osvr_opentrack_json.h"

#define DEVICE_NAME "OpenTracker"

#define M_PI 3.14159265358979323846264338327950288

namespace {
	struct TOpenTrackPacket {
		double x = 0;
		double y = 0;
		double z = 0;
		double yaw = 0;
		double pitch = 0;
		double roll = 0;
	};

	volatile bool keep_connection_open = true;
	std::atomic<TOpenTrackPacket> opentrack_data;
	std::mutex msg_lock;
	std::vector<std::string> msg_queue;

	std::thread opentrack_thread;

	void add_msg(std::string msg) {
		msg_lock.lock();
		msg_queue.push_back(msg);
		msg_lock.unlock();
	}

	bool get_last_msg(std::string* s) {
		if (msg_queue.size() == 0) {
			return false;
		}
		else {
			msg_lock.lock();
			(*s) = msg_queue.at(msg_queue.size() - 1);
			msg_queue.pop_back();
			msg_lock.unlock();
			return true;
		}
	}

	void opentrack_connect() {
		bool was_connected = false;

		while (keep_connection_open) {
			SOCKET socketS;

			int iResult;

			WSADATA wsaData;
			iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);

			if (iResult != 0) {
				std::stringstream info;
				info << "WSAStartup Failed with error : " << iResult;
				add_msg(info.str());
				WSACleanup();
			}

			struct sockaddr_in local;
			struct sockaddr_in from;
			int fromlen = sizeof(from);
			local.sin_family = AF_INET;
			local.sin_port = htons(4242);
			local.sin_addr.s_addr = INADDR_ANY;

			socketS = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

			if (socketS == INVALID_SOCKET) {
				std::stringstream info;
				info << "Cannot open UDP socket on port 4242 : " << iResult;
				add_msg(info.str());
				WSACleanup();
				break;
			}

			iResult = bind(socketS, (sockaddr*)&local, sizeof(local));

			if (iResult == SOCKET_ERROR)
			{
				std::stringstream info;
				info << "Cannot bind socket : " << iResult;
				add_msg(info.str());
				WSACleanup();
				break;
			}

			int bytes_read = 0;

			while (keep_connection_open)
			{
				if (!was_connected) {
					was_connected = true;
					add_msg("Waiting on UDP port 4242 for Opentrack data.");
				}
				TOpenTrackPacket OpenTrackPacket;
				memset(&OpenTrackPacket, 0, sizeof(OpenTrackPacket));
				bytes_read = recvfrom(socketS, (char*)(&OpenTrackPacket), sizeof(OpenTrackPacket), 0, (sockaddr*)&from, &fromlen);
				if (bytes_read > 0)
					opentrack_data.store(OpenTrackPacket, std::memory_order_relaxed);
			}

			closesocket(socketS);
		}
		was_connected = false;
	}


	class Open_Tracker_Device {
	public:
		Open_Tracker_Device(OSVR_PluginRegContext ctx) {
			m_ctx = ctx;
			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);
			osvrDeviceTrackerConfigure(opts, &m_tracker);
			m_dev.initAsync(ctx, DEVICE_NAME, opts);
			m_dev.sendJsonDescriptor(inf_osvr_opentrack_json);
			m_dev.registerUpdateCallback(this);
		}

		OSVR_ReturnCode update() {
			std::string next_msg;
			while (get_last_msg(&next_msg)) {
				osvr::pluginkit::log(m_ctx, OSVR_LOGLEVEL_NOTICE, next_msg.c_str());
			}
			TOpenTrackPacket tracker_data = opentrack_data.load(std::memory_order_relaxed);
			
			OSVR_Vec3 pos;
			osvrVec3SetX(&pos, tracker_data.x);
			osvrVec3SetX(&pos, tracker_data.x);
			osvrVec3SetX(&pos, tracker_data.x);

			OSVR_Quaternion rot = ypr_to_quat(tracker_data.pitch * (M_PI/180), tracker_data.roll * (M_PI / 180), tracker_data.yaw * (M_PI / 180));

			m_pose.rotation = rot;
			m_pose.translation = pos;

			osvrDeviceTrackerSendPose(m_dev, m_tracker, &m_pose, 0);

			return OSVR_RETURN_SUCCESS;
		}

		~Open_Tracker_Device() {
			keep_connection_open = false;
		}

	private:
		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_PoseState m_pose;
		OSVR_PluginRegContext m_ctx;

		OSVR_Quaternion ypr_to_quat(double pitch, double roll, double yaw)
		{
			OSVR_Quaternion q;
			double t0 = std::cos(yaw * 0.5);
			double t1 = std::sin(yaw * 0.5);
			double t2 = std::cos(roll * 0.5);
			double t3 = std::sin(roll * 0.5);
			double t4 = std::cos(pitch * 0.5);
			double t5 = std::sin(pitch * 0.5);

			osvrQuatSetW(&q, t0 * t2 * t4 + t1 * t3 * t5);
			osvrQuatSetX(&q, t0 * t3 * t4 - t1 * t2 * t5);
			osvrQuatSetY(&q, t0 * t2 * t5 + t1 * t3 * t4);
			osvrQuatSetZ(&q, t1 * t2 * t4 - t0 * t3 * t5);
			return q;
		}
	};

	class Open_Tracker_Constructor {
	public:
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx, const char* params) {
			opentrack_thread = std::thread(opentrack_connect);
			osvr::pluginkit::registerObjectForDeletion(ctx, new Open_Tracker_Device(ctx));
			return OSVR_RETURN_SUCCESS;
		}
	};
}

OSVR_PLUGIN(inf_osvr_opentrack) {
	osvr::pluginkit::PluginContext context(ctx);
	osvr::pluginkit::registerDriverInstantiationCallback(ctx, DEVICE_NAME, new Open_Tracker_Constructor);
	return OSVR_RETURN_SUCCESS;
}
